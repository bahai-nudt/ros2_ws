#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mapping 公共库（无 CLI）。

作用:
    为点云拼图提供 sins 位姿、雷达外参、ENU 投影与 binary PCD 读写。
    外参与 C++ LoadGlobalConfig 一致：默认 calib.json + R_bv；
    yaml lidar_imu_extrinsic.enable 为 true 时用 R_bl / T_lb_m 覆盖。
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import re
import shutil
import tempfile
from pathlib import Path

import numpy as np

PLOT_RAW_DIR = Path(__file__).resolve().parent.parent / 'plot_raw_data'
_RAW_SPEC = importlib.util.spec_from_file_location(
    'plot_raw_data_common', PLOT_RAW_DIR / 'plot_common.py')
raw_plot = importlib.util.module_from_spec(_RAW_SPEC)
_RAW_SPEC.loader.exec_module(raw_plot)

load_ins_txt = raw_plot.load_ins_txt
require_raw_dir = raw_plot.require_raw_dir

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563

LIDAR_TOPIC_TO_FRAME = {
    '/rslidar/em4_front/raw': 'lidar_front',
    '/rslidar/em4_rear/raw': 'lidar_rear',
    '/rslidar/m1p_left/raw': 'lidar_left',
    '/rslidar/m1p_right/raw': 'lidar_right',
}
IMU_TOPIC_TO_FRAME = {
    '/bewis/imu/data_raw': 'bewis_imu_front',
    '/bynav/imu/data_raw': 'bynav_imu',
    '/shangyu/imu/data_raw': 'shangyu_imu',
}

OUTPUT_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('z', '<f4'),
    ('intensity', '<f4'),
])
TRAJECTORY_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('z', '<f4'),
    ('rgb', '<u4'),
])

DEFAULT_PCD_ROOT = 'mapping'


def require_dir(path, what):
    directory = Path(path).resolve()
    if not directory.is_dir():
        raise FileNotFoundError(f'找不到{what}: {directory}')
    return directory


def require_file(path, what):
    file_path = Path(path).resolve()
    if not file_path.is_file():
        raise FileNotFoundError(f'找不到{what}: {file_path}')
    return file_path


def resolve_map_output(output_dir, output_arg):
    """默认 <output_dir>/mapping/out.pcd；-o 可以是目录或 .pcd 文件。"""
    if output_arg:
        dest = Path(output_arg).resolve()
        if dest.suffix.lower() == '.pcd':
            dest.parent.mkdir(parents=True, exist_ok=True)
            return dest
        dest.mkdir(parents=True, exist_ok=True)
        return dest / 'out.pcd'
    dest_dir = Path(output_dir).resolve() / DEFAULT_PCD_ROOT
    dest_dir.mkdir(parents=True, exist_ok=True)
    return dest_dir / 'out.pcd'


def parse_yaml_string(text, key, config_path):
    match = re.search(
        rf'^\s*{re.escape(key)}:\s*["\']?([^"\'#\n]+)',
        text,
        re.MULTILINE,
    )
    if not match:
        raise ValueError(f'配置文件缺少 {key}: {config_path}')
    return match.group(1).strip()


def parse_lidar_imu_yaml_enable(text):
    match = re.search(
        r'lidar_imu_extrinsic:\s*(?:#.*)?\n(?:[ \t]+.+\n)*?[ \t]+enable:\s*(true|false)',
        text,
        re.IGNORECASE,
    )
    if not match:
        return False
    return match.group(1).lower() == 'true'


def parse_yaml_number_list(text, key, expected, config_path):
    match = re.search(
        rf'^\s*{re.escape(key)}:\s*\[([^\]]+)\]',
        text,
        re.MULTILINE | re.DOTALL,
    )
    if not match:
        raise ValueError(f'配置文件缺少 {key}: {config_path}')
    values = [float(item.strip()) for item in match.group(1).split(',') if item.strip()]
    if len(values) != expected:
        raise ValueError(f'{key} 应有 {expected} 个数，实际 {len(values)}: {config_path}')
    return np.asarray(values, dtype=float)


def load_map_config(config_path):
    config_path = Path(config_path).resolve()
    text = config_path.read_text(encoding='utf-8')
    imu_topic = parse_yaml_string(text, 'imu_topic', config_path)
    lidar_topic = parse_yaml_string(text, 'lidar_topic', config_path)
    r_bv = parse_yaml_number_list(text, 'R_bv', 9, config_path).reshape(3, 3)
    try:
        origin = parse_yaml_number_list(text, 'gloc_origin_lla_deg', 3, config_path)
        origin_source = 'yaml gloc_origin_lla_deg'
    except ValueError:
        origin = None
        origin_source = 'gnsspos 首帧'
    use_yaml_extrinsic = parse_lidar_imu_yaml_enable(text)
    yaml_r_bl = yaml_t_lb = None
    if use_yaml_extrinsic:
        yaml_r_bl = parse_yaml_number_list(text, 'R_bl', 9, config_path).reshape(3, 3)
        yaml_t_lb = parse_yaml_number_list(text, 'T_lb_m', 3, config_path)
    return {
        'path': config_path,
        'imu_topic': imu_topic,
        'lidar_topic': lidar_topic,
        'r_bv': r_bv,
        'origin_lla': origin,
        'origin_source': origin_source,
        'use_yaml_extrinsic': use_yaml_extrinsic,
        'yaml_r_bl': yaml_r_bl,
        'yaml_t_lb': yaml_t_lb,
    }


def candidate_lidar_frame_ids(frame_id):
    aliases = {
        'front_lidar': 'lidar_front',
        'rear_lidar': 'lidar_rear',
        'left_lidar': 'lidar_left',
        'right_lidar': 'lidar_right',
        'lidar_front': 'front_lidar',
        'lidar_rear': 'rear_lidar',
        'lidar_left': 'left_lidar',
        'lidar_right': 'right_lidar',
    }
    candidates = [frame_id]
    if frame_id in aliases:
        candidates.append(aliases[frame_id])
    return candidates


def load_calib_transform(calib, section, frame_ids):
    for frame_id in frame_ids:
        for item in calib.get(section, []):
            if item.get('frameId') != frame_id:
                continue
            matrix = np.asarray(item.get('transformMatrix'), dtype=float)
            if matrix.shape != (4, 4):
                raise ValueError(f'{section}/{frame_id} 的 transformMatrix 不是 4x4')
            return matrix[:3, :3], matrix[:3, 3]
    raise ValueError(f'{section} 中找不到 frameId: {", ".join(frame_ids)}')


def orthogonalize_rotation(rotation):
    u, _, vt = np.linalg.svd(rotation)
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


def load_lidar_extrinsic(raw_dir, config):
    imu_topic = config['imu_topic']
    lidar_topic = config['lidar_topic']
    if imu_topic not in IMU_TOPIC_TO_FRAME:
        raise ValueError(f'不支持的 imu_topic: {imu_topic}')
    if lidar_topic not in LIDAR_TOPIC_TO_FRAME:
        raise ValueError(f'不支持的 lidar_topic: {lidar_topic}')

    lidar_frame = LIDAR_TOPIC_TO_FRAME[lidar_topic]
    if config['use_yaml_extrinsic']:
        r_bl = orthogonalize_rotation(config['yaml_r_bl'])
        t_lb = np.asarray(config['yaml_t_lb'], dtype=float)
        print(f'雷达外参: yaml lidar_imu_extrinsic（R_bl / T_lb_m）')
        return lidar_frame, r_bl, t_lb

    calib_path = Path(raw_dir) / 'calib.json'
    with calib_path.open('r', encoding='utf-8') as file:
        calib = json.load(file)
    r_vl, t_lv = load_calib_transform(
        calib, 'lidar_params', candidate_lidar_frame_ids(lidar_frame))
    _, t_bv = load_calib_transform(calib, 'imu_params', [IMU_TOPIC_TO_FRAME[imu_topic]])
    r_bl = orthogonalize_rotation(config['r_bv'] @ r_vl)
    t_lb = config['r_bv'] @ (t_lv - t_bv)
    print(f'雷达外参: calib.json + R_bv（p_imu = R_bl * p_lidar + T_lb）')
    return lidar_frame, r_bl, t_lb


def load_gnss_origin(raw_dir):
    gnss_path = Path(raw_dir) / 'gnsspos.txt'
    with gnss_path.open('r', encoding='utf-8') as file:
        first = next(csv.DictReader(file), None)
    if first is None:
        raise ValueError(f'GNSSPOS 数据为空: {gnss_path}')
    return np.asarray([float(first['lat']), float(first['lon']), float(first['hgt'])])


def resolve_enu_origin(raw_dir, config):
    if config['origin_lla'] is not None:
        return np.asarray(config['origin_lla'], dtype=float), config['origin_source']
    return load_gnss_origin(raw_dir), 'gnsspos 首帧'


def a2mat(pitch, roll, yaw):
    """复现 pose_converter::a2mat，att = [pitch, roll, yaw] (rad)。"""
    sp, cp = math.sin(pitch), math.cos(pitch)
    sr, cr = math.sin(roll), math.cos(roll)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.asarray([
        [cy * cr - sy * sp * sr, -sy * cp, cy * sr + sy * sp * cr],
        [sy * cr + cy * sp * sr, cy * cp, sy * sr - cy * sp * cr],
        [-cp * sr, sp, cp * cr],
    ])


def lla_to_local_enu(lat_deg, lon_deg, height_m, origin_lla):
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    eccentricity_sq = WGS84_F * (2.0 - WGS84_F)
    prime_vertical = WGS84_A / np.sqrt(1.0 - eccentricity_sq * np.sin(lat) ** 2)
    ecef = np.column_stack((
        (prime_vertical + height_m) * np.cos(lat) * np.cos(lon),
        (prime_vertical + height_m) * np.cos(lat) * np.sin(lon),
        (prime_vertical * (1.0 - eccentricity_sq) + height_m) * np.sin(lat),
    ))
    lat0 = math.radians(float(origin_lla[0]))
    lon0 = math.radians(float(origin_lla[1]))
    height0 = float(origin_lla[2])
    prime_vertical0 = WGS84_A / math.sqrt(1.0 - eccentricity_sq * math.sin(lat0) ** 2)
    ecef0 = np.asarray([
        (prime_vertical0 + height0) * math.cos(lat0) * math.cos(lon0),
        (prime_vertical0 + height0) * math.cos(lat0) * math.sin(lon0),
        (prime_vertical0 * (1.0 - eccentricity_sq) + height0) * math.sin(lat0),
    ])
    ecef_to_enu = np.asarray([
        [-math.sin(lon0), math.cos(lon0), 0.0],
        [
            -math.sin(lat0) * math.cos(lon0),
            -math.sin(lat0) * math.sin(lon0),
            math.cos(lat0),
        ],
        [
            math.cos(lat0) * math.cos(lon0),
            math.cos(lat0) * math.sin(lon0),
            math.sin(lat0),
        ],
    ])
    return (ecef - ecef0) @ ecef_to_enu.T


def matrix_to_quaternion(rotation):
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray([
            0.25 * scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
        ])
    else:
        diagonal = np.diag(rotation)
        axis = int(np.argmax(diagonal))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.asarray([
                (rotation[2, 1] - rotation[1, 2]) / scale,
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
            ])
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.asarray([
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.asarray([
                (rotation[1, 0] - rotation[0, 1]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
            ])
    return quaternion / np.linalg.norm(quaternion)


def load_lidar_poses(sins_path, r_bl, t_lb, origin_lla):
    timestamps = []
    latitudes = []
    longitudes = []
    heights = []
    attitudes = []
    for rec in load_ins_txt(sins_path):
        timestamps.append(rec['time'])
        latitudes.append(rec['latitude'])
        longitudes.append(rec['longitude'])
        heights.append(rec['height'])
        pitch = math.radians(rec['pitch'])
        roll = math.radians(rec['roll'])
        yaw = -math.radians(rec['azimuth'])
        attitudes.append(a2mat(pitch, roll, yaw))

    if len(timestamps) < 2:
        raise ValueError(f'位姿数量不足: {len(timestamps)}')

    pose_time = np.asarray(timestamps)
    latitudes = np.asarray(latitudes)
    longitudes = np.asarray(longitudes)
    heights = np.asarray(heights)
    attitudes = np.asarray(attitudes)
    time_diff = np.diff(pose_time)
    if np.any(time_diff < 0.0):
        order = np.argsort(pose_time, kind='stable')
        pose_time = pose_time[order]
        latitudes = latitudes[order]
        longitudes = longitudes[order]
        heights = heights[order]
        attitudes = attitudes[order]
        time_diff = np.diff(pose_time)
        print('警告: 位姿时间戳存在倒序，已按时间排序')
    keep = np.r_[time_diff != 0.0, True]
    pose_time = pose_time[keep]
    latitudes = latitudes[keep]
    longitudes = longitudes[keep]
    heights = heights[keep]
    attitudes = attitudes[keep]

    imu_position = lla_to_local_enu(latitudes, longitudes, heights, origin_lla)
    lidar_rotation = np.einsum('nij,jk->nik', attitudes, r_bl)
    lidar_position = imu_position + np.einsum('nij,j->ni', attitudes, t_lb)
    lidar_quaternion = np.asarray([
        matrix_to_quaternion(rotation) for rotation in lidar_rotation
    ])
    return pose_time, lidar_position, lidar_quaternion


def load_gnss_trajectory(raw_dir, origin_lla):
    timestamps = []
    latitudes = []
    longitudes = []
    heights = []
    gnss_path = Path(raw_dir) / 'gnsspos.txt'
    with gnss_path.open('r', encoding='utf-8') as file:
        for row in csv.DictReader(file):
            timestamps.append(float(row['timestamp']))
            latitudes.append(float(row['lat']))
            longitudes.append(float(row['lon']))
            heights.append(float(row['hgt']))
    gnss_time = np.asarray(timestamps)
    order = np.argsort(gnss_time, kind='stable')
    gnss_time = gnss_time[order]
    latitudes = np.asarray(latitudes)[order]
    longitudes = np.asarray(longitudes)[order]
    heights = np.asarray(heights)[order]
    time_diff = np.diff(gnss_time)
    keep = np.r_[time_diff != 0.0, True]
    gnss_time = gnss_time[keep]
    gnss_position = lla_to_local_enu(
        latitudes[keep], longitudes[keep], heights[keep], origin_lla)
    return gnss_time, gnss_position


def quaternion_slerp(q0, q1, alpha):
    q1 = q1.copy()
    dot = np.sum(q0 * q1, axis=1)
    opposite = dot < 0.0
    q1[opposite] *= -1.0
    dot = np.clip(np.abs(dot), 0.0, 1.0)
    result = np.empty_like(q0)
    close = dot > 0.9995
    if np.any(close):
        result[close] = (
            (1.0 - alpha[close, None]) * q0[close]
            + alpha[close, None] * q1[close]
        )
    far = ~close
    if np.any(far):
        angle = np.arccos(dot[far])
        sin_angle = np.sin(angle)
        weight0 = np.sin((1.0 - alpha[far]) * angle) / sin_angle
        weight1 = np.sin(alpha[far] * angle) / sin_angle
        result[far] = weight0[:, None] * q0[far] + weight1[:, None] * q1[far]
    return result / np.linalg.norm(result, axis=1)[:, None]


def rotate_by_quaternion(quaternion, points):
    vector = quaternion[:, 1:]
    cross1 = np.cross(vector, points)
    cross2 = np.cross(vector, cross1)
    return points + 2.0 * (quaternion[:, :1] * cross1 + cross2)


def transform_points(pose_time, pose_position, pose_quaternion, point_time, points):
    right = np.searchsorted(pose_time, point_time, side='right')
    right = np.clip(right, 1, len(pose_time) - 1)
    left = right - 1
    interval = pose_time[right] - pose_time[left]
    alpha = (point_time - pose_time[left]) / interval
    position = (
        (1.0 - alpha[:, None]) * pose_position[left]
        + alpha[:, None] * pose_position[right]
    )
    quaternion = quaternion_slerp(
        pose_quaternion[left], pose_quaternion[right], alpha)
    return rotate_by_quaternion(quaternion, points) + position


def load_binary_pcd(path):
    header = {}
    with Path(path).open('rb') as file:
        while True:
            line = file.readline()
            if not line:
                raise ValueError(f'PCD 缺少 DATA 行: {path}')
            text = line.decode('ascii').strip()
            if not text or text.startswith('#'):
                continue
            key, *values = text.split()
            header[key.upper()] = values
            if key.upper() == 'DATA':
                break
        if header['DATA'][0].lower() != 'binary':
            raise ValueError(f'仅支持 DATA binary PCD: {path}')
        fields = header['FIELDS']
        sizes = [int(value) for value in header['SIZE']]
        types = header['TYPE']
        counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError(f'PCD 字段定义长度不一致: {path}')
        if any(count != 1 for count in counts):
            raise ValueError(f'暂不支持 COUNT > 1 的 PCD: {path}')
        type_prefix = {'F': 'f', 'U': 'u', 'I': 'i'}
        dtype_fields = []
        for field, size, value_type in zip(fields, sizes, types):
            if value_type not in type_prefix or size not in (1, 2, 4, 8):
                raise ValueError(f'不支持的 PCD 字段类型: {field} {value_type}{size}')
            dtype_fields.append((field, f'<{type_prefix[value_type]}{size}'))
        point_count = int(header.get('POINTS', ['0'])[0])
        points = np.fromfile(file, dtype=np.dtype(dtype_fields), count=point_count)
        if len(points) != point_count:
            raise ValueError(
                f'PCD 点数不完整: 期望 {point_count}，实际 {len(points)}: {path}')
        return points


def write_pcd_header(file, point_count):
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {point_count}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {point_count}\n'
        'DATA binary\n'
    )
    file.write(header.encode('ascii'))


def write_trajectory_pcd(output_path, pose_time, pose_position, start_time, end_time,
                         rgb=0x00FF0000):
    selected = (pose_time >= start_time) & (pose_time <= end_time)
    trajectory_position = pose_position[selected]
    if len(trajectory_position) == 0:
        raise ValueError('所选时间段没有轨迹点')
    trajectory = np.empty(len(trajectory_position), dtype=TRAJECTORY_DTYPE)
    trajectory['x'] = trajectory_position[:, 0]
    trajectory['y'] = trajectory_position[:, 1]
    trajectory['z'] = trajectory_position[:, 2]
    trajectory['rgb'] = rgb
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z rgb\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {len(trajectory)}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(trajectory)}\n'
        'DATA binary\n'
    )
    with output_path.open('wb') as file:
        file.write(header.encode('ascii'))
        trajectory.tofile(file)
    print(f'轨迹完成: 输出 {len(trajectory)} 个点到 {output_path}')


def pcd_time_from_name(path):
    try:
        return int(Path(path).stem) / 1e6
    except ValueError:
        return None


def list_lidar_pcds(pcd_dir):
    return sorted(Path(pcd_dir).glob('*.pcd'), key=lambda path: int(path.stem))


def build_map(sins_path, raw_dir, config_path, output_path, stride, min_range,
              start_sec, end_sec, trajectory_output, gnss_trajectory_output):
    config = load_map_config(config_path)
    lidar_frame, r_bl, t_lb = load_lidar_extrinsic(raw_dir, config)
    pcd_dir = Path(raw_dir) / lidar_frame
    pcd_paths = list_lidar_pcds(pcd_dir)
    if not pcd_paths:
        raise ValueError(f'没有找到 PCD: {pcd_dir}')

    origin, origin_source = resolve_enu_origin(raw_dir, config)
    pose_time, pose_position, pose_quaternion = load_lidar_poses(
        sins_path, r_bl, t_lb, origin)
    map_start_time = pose_time[0] + start_sec
    map_end_time = pose_time[-1] if end_sec is None else min(
        pose_time[0] + end_sec, pose_time[-1])
    if map_start_time >= map_end_time:
        raise ValueError('所选时间段不在位姿时间范围内')

    print(
        f'ENU 原点（{origin_source}）: '
        f'lat={origin[0]:.12f}, lon={origin[1]:.12f}, h={origin[2]:.3f}'
    )
    print(f'T_lb [m]: {t_lb.tolist()}')
    print(f'雷达目录: {pcd_dir}')
    print(f'位姿时间范围: {pose_time[0]:.6f} ~ {pose_time[-1]:.6f}')
    print(f'地图时间范围: {map_start_time:.6f} ~ {map_end_time:.6f}')

    if trajectory_output:
        write_trajectory_pcd(
            trajectory_output, pose_time, pose_position,
            map_start_time, map_end_time)
    if gnss_trajectory_output:
        gnss_time, gnss_position = load_gnss_trajectory(raw_dir, origin)
        write_trajectory_pcd(
            gnss_trajectory_output, gnss_time, gnss_position,
            map_start_time, map_end_time, rgb=0x0000FF00)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temp_file = tempfile.NamedTemporaryFile(
        prefix=f'.{output_path.stem}_',
        suffix='.bin',
        dir=output_path.parent,
        delete=False,
    )
    temp_path = Path(temp_file.name)
    point_count = 0
    used_frames = 0
    scan_margin_s = 0.25

    try:
        with temp_file:
            for frame_index, pcd_path in enumerate(pcd_paths):
                file_time = pcd_time_from_name(pcd_path)
                if file_time is not None and (
                        file_time < map_start_time - scan_margin_s
                        or file_time > map_end_time + scan_margin_s):
                    continue

                cloud = load_binary_pcd(pcd_path)
                time_field = 'timestamp' if 'timestamp' in cloud.dtype.names else 'time'
                required = {'x', 'y', 'z', 'intensity', time_field}
                if not required.issubset(cloud.dtype.names):
                    raise ValueError(f'PCD 缺少字段 {sorted(required)}: {pcd_path}')

                x = cloud['x']
                y = cloud['y']
                z = cloud['z']
                intensity = cloud['intensity']
                point_time = cloud[time_field].astype(np.float64, copy=False)
                valid = (
                    np.isfinite(x)
                    & np.isfinite(y)
                    & np.isfinite(z)
                    & np.isfinite(intensity)
                    & np.isfinite(point_time)
                    & (x * x + y * y + z * z > min_range * min_range)
                    & (point_time >= map_start_time)
                    & (point_time <= map_end_time)
                )
                selected = np.flatnonzero(valid)[frame_index % stride::stride]
                if selected.size == 0:
                    continue

                points_lidar = np.column_stack((
                    x[selected], y[selected], z[selected],
                )).astype(np.float64)
                points_world = transform_points(
                    pose_time, pose_position, pose_quaternion,
                    point_time[selected], points_lidar)

                output_points = np.empty(selected.size, dtype=OUTPUT_DTYPE)
                output_points['x'] = points_world[:, 0]
                output_points['y'] = points_world[:, 1]
                output_points['z'] = points_world[:, 2]
                output_points['intensity'] = intensity[selected]
                output_points.tofile(temp_file)
                point_count += selected.size
                used_frames += 1
                if used_frames == 1 or used_frames % 50 == 0:
                    print(
                        f'\r处理帧 {frame_index + 1}/{len(pcd_paths)}, '
                        f'累计点数 {point_count}',
                        end='',
                        flush=True,
                    )

        if point_count == 0:
            raise ValueError('没有点落在有效位姿时间范围内')

        with output_path.open('wb') as output_file:
            write_pcd_header(output_file, point_count)
            with temp_path.open('rb') as binary_file:
                shutil.copyfileobj(binary_file, output_file, length=16 * 1024 * 1024)
        print(
            f'\n完成: 使用 {used_frames}/{len(pcd_paths)} 帧，'
            f'输出 {point_count} 点到 {output_path}'
        )
    finally:
        temp_path.unlink(missing_ok=True)
