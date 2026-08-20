#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plot_raw_data 公共库（无 CLI）。

作用:
    为绘图脚本提供 raw_data 校验、IMU / INS / GLOC 文本读取，以及 GNSS 对比用的
    RFU 杆臂 / 双天线航向补偿（plot_compare_inspvax_gnss 与 plot_compare_sins_gnss 共用）。
    gloc.txt 与 gloc_result.txt 同一套 CSV，由 load_gloc_compare_rows 读取。

IMU  `{brand}.imu.txt`  空格 11 列，时间戳为微秒:
    timestamp_us qx qy qz qw gx gy gz ax ay az

INS / sins_result  空格 11 列，时间轴用 obs_timestamp 微秒:
    timestamp lat lon height roll pitch azimuth vn ve vu obs_timestamp
"""

import bisect
import csv
import json
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

IMU_FIELDS = ('gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z')


def imu_brand_from_path(path):
    stem = Path(path).stem
    if stem.endswith('.imu'):
        return stem[:-4]
    return stem


def ins_brand_from_path(path):
    stem = Path(path).stem
    if stem.endswith('.ins'):
        return stem[:-4]
    return stem


def list_imu_files(raw_dir):
    return sorted(Path(raw_dir).glob('*.imu.txt'))


def list_ins_files(raw_dir):
    return sorted(Path(raw_dir).glob('*.ins.txt'))


DEFAULT_PLOT_ROOT = 'plot_raw_data'

# 成对对比脚本共用子目录名（raw_data 与 plot_result 只是父目录不同）。
COMPARE_INS_GNSS_DIR = 'plot_compare_inspvax_gnss'
COMPARE_GLOC_GNSS_DIR = 'plot_compare_gloc_gnss'
COMPARE_INSPVAX_DIR = 'plot_compare_inspvax'


def require_raw_dir(path):
    raw_dir = Path(path).resolve()
    if not raw_dir.is_dir():
        raise FileNotFoundError(f'需要解码后的 raw_data 目录: {raw_dir}')
    return raw_dir


def resolve_output_dir(raw_dir, output_arg, default_name):
    if output_arg:
        output_dir = Path(output_arg).resolve()
    else:
        output_dir = Path(raw_dir).resolve().parent / DEFAULT_PLOT_ROOT / default_name
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def resolve_imu_paths(raw_dir, brand=None):
    """返回要绘制的 IMU 文件列表。brand 为 None 时返回目录内全部 *.imu.txt。"""
    raw_dir = require_raw_dir(raw_dir)
    if brand:
        name = brand if brand.endswith('.txt') else f'{brand}.imu.txt'
        imu_path = raw_dir / name
        if not imu_path.is_file():
            raise FileNotFoundError(f'IMU 文件不存在: {imu_path}')
        return [imu_path]

    imu_files = list_imu_files(raw_dir)
    if not imu_files:
        raise FileNotFoundError(f'目录中未找到 *.imu.txt: {raw_dir}')
    return imu_files


def resolve_ins_path(raw_dir):
    raw_dir = require_raw_dir(raw_dir)
    ins_files = list_ins_files(raw_dir)
    if not ins_files:
        raise FileNotFoundError(f'目录中未找到 *.ins.txt: {raw_dir}')
    if len(ins_files) > 1:
        names = ', '.join(item.name for item in ins_files)
        raise RuntimeError(f'目录中有多个 *.ins.txt（一组 raw_data 应只有一路 INS）: {names}')
    return ins_files[0]


def load_imu_txt(txt_path):
    samples = []
    with open(txt_path, 'r', encoding='utf-8') as file:
        for line in file:
            text = line.strip()
            if not text or text.startswith('#'):
                continue
            parts = text.split()
            if len(parts) < 11:
                continue
            samples.append({
                't': int(parts[0]) / 1e6,
                'gyro_x': float(parts[5]),
                'gyro_y': float(parts[6]),
                'gyro_z': float(parts[7]),
                'accel_x': float(parts[8]),
                'accel_y': float(parts[9]),
                'accel_z': float(parts[10]),
            })
    samples.sort(key=lambda item: item['t'])
    return samples


def parse_ins_line(line):
    text = line.strip()
    if not text or text.startswith('#'):
        return None
    parts = text.split()
    if len(parts) != 11:
        return None
    obs_us = int(parts[10])
    return {
        'timestamp_us': int(parts[0]),
        'latitude': float(parts[1]),
        'longitude': float(parts[2]),
        'height': float(parts[3]),
        'roll': float(parts[4]),
        'pitch': float(parts[5]),
        'azimuth': float(parts[6]),
        'north_vel': float(parts[7]),
        'east_vel': float(parts[8]),
        'up_vel': float(parts[9]),
        'obs_timestamp_us': obs_us,
        'time': obs_us / 1e6,
    }


def load_ins_txt(txt_path):
    rows = []
    with open(txt_path, 'r', encoding='utf-8') as file:
        for line in file:
            row = parse_ins_line(line)
            if row is not None:
                rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# GNSS 对比共用：RFU 杆臂、双天线航向、位置/速度/航向图
# plot_compare_inspvax_gnss 与 plot_compare_sins_gnss 只在读 INS 文件路径上不同。
# ---------------------------------------------------------------------------

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WIE = 7.2921151467e-5

GNSS_COLOR = '#E74C3C'
INS_COLOR = '#2980B9'

IMU_TOPIC_TO_IDS = {
    '/bewis/imu/data_raw': ('bewis', 'bewis_front_arm_value'),
    '/bynav/imu/data_raw': ('bynav', 'bynav_arm_value'),
    '/shangyu/imu/data_raw': ('shangyu', 'shangyu_arm_value'),
}
DUAL_ANTENNA_TO_VEHICLE_DEG = 90.0


def load_imu_topic_from_config(config_path):
    pattern = re.compile(r'^\s*imu_topic:\s*["\']?([^"\'\s#]+)', re.MULTILINE)
    text = Path(config_path).read_text(encoding='utf-8')
    match = pattern.search(text)
    if not match:
        raise ValueError(f'未在 {config_path} 中找到 imu_topic')
    return match.group(1).strip()


def load_heading_offset_from_config(config_path):
    pattern = re.compile(r'^\s*heading_offset_deg:\s*([-\d.eE+]+)', re.MULTILINE)
    text = Path(config_path).read_text(encoding='utf-8')
    match = pattern.search(text)
    if not match:
        return 0.0
    return float(match.group(1))


def apply_heading_for_inspvax(heading_rows, offset_deg):
    """把双天线航向转到车头航向（与 INSPVAX / sins_result 的 azimuth 对齐）。

    heading_offset_deg=0：左主右从，基线朝右；180：主从对调。
    滤波 LoadHeading 只做 raw - offset；对比 azimuth 时再减 90°
    （基线航向相对车头）。heading_plot = raw - offset - 90。
    """
    total_deg = offset_deg + DUAL_ANTENNA_TO_VEHICLE_DEG
    corrected = []
    for row in heading_rows:
        updated = dict(row)
        updated['heading'] = (row['heading'] - total_deg) % 360.0
        corrected.append(updated)
    return corrected, total_deg


def load_lever_arm_from_calib(calib_path, arm_value_key):
    with open(calib_path, 'r', encoding='utf-8') as file:
        calib = json.load(file)
    for item in calib.get('arm_value_params', []):
        if arm_value_key in item and len(item[arm_value_key]) >= 3:
            return np.array(item[arm_value_key][:3], dtype=float)
        if item.get('name') == arm_value_key:
            values = item.get('values', [])
            if len(values) >= 3:
                return np.array(values[:3], dtype=float)
    raise ValueError(f'未在 {calib_path} 中找到 {arm_value_key}')


def askew(vec):
    return np.array([
        [0.0, -vec[2], vec[1]],
        [vec[2], 0.0, -vec[0]],
        [-vec[1], vec[0], 0.0],
    ])


def earth_radii(lat_rad):
    sin_lat = math.sin(lat_rad)
    e2 = WGS84_F * (2.0 - WGS84_F)
    denom = math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    rn = WGS84_A / denom
    rm = WGS84_A * (1.0 - e2) / (denom ** 3)
    return rm, rn


def cnb_rfu_from_inspvax(roll_deg, pitch_deg, azimuth_deg):
    """姿态 → Cnb：b 系 RFU，n 系 ENU。azimuth 为真北顺时针航向。"""
    psi = math.radians(azimuth_deg)
    theta = math.radians(pitch_deg)
    phi = math.radians(roll_deg)
    cos_psi, sin_psi = math.cos(psi), math.sin(psi)
    r_heading = np.array([
        [cos_psi, sin_psi, 0.0],
        [-sin_psi, cos_psi, 0.0],
        [0.0, 0.0, 1.0],
    ])
    cos_p, sin_p = math.cos(theta), math.sin(theta)
    r_pitch = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cos_p, -sin_p],
        [0.0, sin_p, cos_p],
    ])
    cos_r, sin_r = math.cos(phi), math.sin(phi)
    r_roll = np.array([
        [cos_r, 0.0, sin_r],
        [0.0, 1.0, 0.0],
        [-sin_r, 0.0, cos_r],
    ])
    return r_heading @ r_pitch @ r_roll


def pos_imu_to_antenna(lat_deg, lon_deg, alt_m, cnb, lever):
    lat_rad = math.radians(lat_deg)
    rm, rn = earth_radii(lat_rad)
    lever_n = cnb @ lever
    dlat = lever_n[1] / (rm + alt_m)
    dlon = lever_n[0] / ((rn + alt_m) * math.cos(lat_rad))
    return (
        math.degrees(lat_rad + dlat),
        math.degrees(math.radians(lon_deg) + dlon),
        alt_m + lever_n[2],
    )


def vel_imu_to_antenna(ve, vn, vu, lat_deg, cnb, wib, lever):
    lat_rad = math.radians(lat_deg)
    wnie = np.array([0.0, WIE * math.cos(lat_rad), WIE * math.sin(lat_rad)])
    web = wib - cnb.T @ wnie
    return np.array([ve, vn, vu]) + cnb @ (askew(web) @ lever)


def nearest_imu_gyro(imu_times, gyros, query_time):
    if not imu_times:
        return np.zeros(3)
    idx = bisect.bisect_left(imu_times, query_time)
    candidates = []
    if idx > 0:
        candidates.append(idx - 1)
    if idx < len(imu_times):
        candidates.append(idx)
    best_i = min(candidates, key=lambda i: abs(imu_times[i] - query_time))
    return gyros[best_i]


def apply_lever_arm_to_ins(ins_rows, lever, imu_samples, rotate_bewis):
    imu_times = []
    gyros = []
    for sample in imu_samples:
        gx, gy, gz = sample['gyro_x'], sample['gyro_y'], sample['gyro_z']
        if rotate_bewis:
            gx, gy, gz = -gy, gx, gz
        imu_times.append(sample['t'])
        gyros.append(np.array([gx, gy, gz], dtype=float))

    corrected = []
    for row in ins_rows:
        cnb = cnb_rfu_from_inspvax(row['roll'], row['pitch'], row['azimuth'])
        lat, lon, hgt = pos_imu_to_antenna(
            row['lat'], row['lon'], row['hgt'], cnb, lever)
        wib = nearest_imu_gyro(imu_times, gyros, row['time'])
        ve, vn, vu = vel_imu_to_antenna(
            row['ve'], row['vn'], row['vu'], row['lat'], cnb, wib, lever)
        updated = dict(row)
        updated['lat'] = lat
        updated['lon'] = lon
        updated['hgt'] = hgt
        updated['ve'] = float(ve)
        updated['vn'] = float(vn)
        updated['vu'] = float(vu)
        corrected.append(updated)
    return corrected


def try_enable_lever_arm(raw_dir, config_path, ins_rows, ins_name='INS'):
    """--config 开启杆臂补偿。calib.json 缺失时警告并返回原数据。"""
    config_path = Path(config_path).resolve()
    if not config_path.is_file():
        print(f'错误: 配置文件不存在: {config_path}')
        return ins_rows, False, 1

    calib_path = Path(raw_dir) / 'calib.json'
    if not calib_path.is_file():
        print(f'警告: 已指定 --config 开启杆臂补偿，但未找到 {calib_path}')
        return ins_rows, False, 0

    try:
        imu_topic = load_imu_topic_from_config(config_path)
        if imu_topic not in IMU_TOPIC_TO_IDS:
            print(f'警告: 不支持的 imu_topic: {imu_topic}，杆臂补偿未应用')
            return ins_rows, False, 0
        brand, arm_key = IMU_TOPIC_TO_IDS[imu_topic]
        lever = load_lever_arm_from_calib(calib_path, arm_key)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f'警告: 杆臂补偿读取失败，未应用: {exc}')
        return ins_rows, False, 0

    imu_samples = []
    imu_path = Path(raw_dir) / f'{brand}.imu.txt'
    if imu_path.is_file():
        imu_samples = load_imu_txt(imu_path)
    else:
        print(f'警告: 未找到 {imu_path.name}，速度杆臂按角速度=0 补偿')

    rotate_bewis = '/bewis/' in imu_topic
    corrected = apply_lever_arm_to_ins(ins_rows, lever, imu_samples, rotate_bewis)
    print(f'杆臂补偿: 开启（config={config_path.name}, imu_topic={imu_topic}）')
    print(f'GNSS 主天线杆臂 T_gb {arm_key} [m]: {lever.tolist()}')
    print(f'已将 {ins_name} 位置/速度转换到 GNSS 主天线；姿态为刚体，航向不变')
    return corrected, True, 0


def apply_compare_config(raw_dir, config_path, ins_rows, heading_rows, ins_name='INS'):
    """传入 yaml 时做杆臂 + 双天线航向补偿；未传则原样返回。"""
    if not config_path:
        return ins_rows, heading_rows, False, None, 0
    ins_rows, lever_applied, lever_rc = try_enable_lever_arm(
        raw_dir, config_path, ins_rows, ins_name=ins_name)
    if lever_rc:
        return ins_rows, heading_rows, False, None, lever_rc
    heading_offset = load_heading_offset_from_config(config_path)
    heading_rows, total_deg = apply_heading_for_inspvax(heading_rows, heading_offset)
    print(
        f'航向补偿: GNSS heading -= {heading_offset:.3f} (heading_offset_deg) '
        f'+ {DUAL_ANTENNA_TO_VEHICLE_DEG:.0f} (双天线基线→车头) = {total_deg:.3f} deg'
    )
    return ins_rows, heading_rows, lever_applied, heading_offset, 0


def load_gnsspos(csv_path):
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as file:
        for row in csv.DictReader(file):
            if int(row['pos_type']) != 50:
                continue
            rows.append({
                'time': float(row['timestamp']),
                'lat': float(row['lat']),
                'lon': float(row['lon']),
                'hgt': float(row['hgt']),
            })
    rows.sort(key=lambda item: item['time'])
    return rows


def load_gnssvel(csv_path):
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as file:
        for row in csv.DictReader(file):
            trk_rad = math.radians(float(row['trk_gnd']))
            hor_speed = float(row['hor_speed'])
            rows.append({
                'time': float(row['timestamp']),
                've': hor_speed * math.sin(trk_rad),
                'vn': hor_speed * math.cos(trk_rad),
                'vu': float(row['ver_speed']),
            })
    rows.sort(key=lambda item: item['time'])
    return rows


def load_heading(csv_path):
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as file:
        for row in csv.DictReader(file):
            if int(row['pos_type']) != 50:
                continue
            rows.append({
                'time': float(row['timestamp']),
                'heading': float(row['heading']),
            })
    rows.sort(key=lambda item: item['time'])
    return rows


def load_gloc_compare_rows(csv_path):
    """gloc.txt / gloc_result.txt 同一套 CSV → 对比用行。

    时间用 obs_time；速度为车体发布的 ENU；yaw 为弧度，azimuth 为对应的度。
    GLOC 是车体系原点，不做 IMU 杆臂。
    """
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as file:
        for row in csv.DictReader(file):
            yaw = float(row['yaw'])
            rows.append({
                'time': float(row['obs_time']),
                'lat': float(row['latitude']),
                'lon': float(row['longitude']),
                'hgt': float(row['altitude']),
                've': float(row['linear_velocity_x']),
                'vn': float(row['linear_velocity_y']),
                'vu': float(row['linear_velocity_z']),
                'yaw': yaw,
                'azimuth': math.degrees(yaw) % 360.0,
            })
    rows.sort(key=lambda item: item['time'])
    return rows


def load_ins_compare_rows(txt_path):
    """11 列 INS / sins_result → 对比用行（时间用 obs_timestamp）。"""
    rows = []
    for rec in load_ins_txt(txt_path):
        rows.append({
            'time': rec['time'],
            'lat': rec['latitude'],
            'lon': rec['longitude'],
            'hgt': rec['height'],
            've': rec['east_vel'],
            'vn': rec['north_vel'],
            'vu': rec['up_vel'],
            'roll': rec['roll'],
            'pitch': rec['pitch'],
            'azimuth': rec['azimuth'],
        })
    rows.sort(key=lambda item: item['time'])
    return rows


def require_gnss_compare_files(raw_dir):
    raw_dir = Path(raw_dir)
    required = {
        'gnsspos': raw_dir / 'gnsspos.txt',
        'gnssvel': raw_dir / 'gnssvel.txt',
        'heading': raw_dir / 'heading.txt',
    }
    for name, path in required.items():
        if not path.is_file():
            raise FileNotFoundError(f'文件不存在: {path} ({name})')
    return (
        load_gnsspos(required['gnsspos']),
        load_gnssvel(required['gnssvel']),
        load_heading(required['heading']),
    )


def estimate_heading_bias(heading_rows, ins_rows, t0, window_s=5.0, ins_name='INS'):
    """用前几秒估计 INS azimuth − GNSS heading 常值偏差。"""
    ins_times = [row['time'] for row in ins_rows]
    diffs = []
    for gnss_hdg in heading_rows:
        idx = bisect.bisect_left(ins_times, gnss_hdg['time'])
        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(ins_times):
            candidates.append(idx)
        best_i = min(candidates, key=lambda i: abs(ins_times[i] - gnss_hdg['time']))
        ins_hdg = ins_rows[best_i]['azimuth']
        diff = (ins_hdg - gnss_hdg['heading'] + 180) % 360 - 180
        diffs.append((gnss_hdg['time'], diff))
    early_diffs = [diff for t, diff in diffs if t - t0 <= window_s]
    if early_diffs:
        used_diffs = early_diffs
        window_desc = f'前 {window_s:g} 秒'
    else:
        print('时间不够，取全部时间')
        used_diffs = [diff for _, diff in diffs]
        window_desc = '全部时间'
    if not used_diffs:
        return None
    heading_bias = sum(used_diffs) / len(used_diffs)
    print(
        f'航向系统性偏差 ({ins_name} - GNSS, {window_desc}): '
        f'{heading_bias:.3f} deg，已补偿到 {ins_name}'
    )
    return heading_bias


def rel_time(rows, t0):
    return [row['time'] - t0 for row in rows]


def plot_position(ref_rows, ins_rows, output_path, t0, ins_label='INS', ref_label='GNSS'):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Position: {ins_label} vs {ref_label}', fontsize=14)

    ref_t = rel_time(ref_rows, t0)
    ins_t = rel_time(ins_rows, t0)

    ax = axes[0, 0]
    ax.plot(ref_t, [row['lat'] for row in ref_rows], color=GNSS_COLOR,
            linewidth=0.8, alpha=0.8, label=ref_label)
    ax.plot(ins_t, [row['lat'] for row in ins_rows], color=INS_COLOR,
            linewidth=0.8, alpha=0.8, label=ins_label)
    ax.set_ylabel('Latitude (deg)')
    ax.set_title('Latitude vs Time')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

    ax = axes[0, 1]
    ax.plot(ref_t, [row['lon'] for row in ref_rows], color=GNSS_COLOR,
            linewidth=0.8, alpha=0.8, label=ref_label)
    ax.plot(ins_t, [row['lon'] for row in ins_rows], color=INS_COLOR,
            linewidth=0.8, alpha=0.8, label=ins_label)
    ax.set_ylabel('Longitude (deg)')
    ax.set_title('Longitude vs Time')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

    ax = axes[1, 0]
    ax.plot(ref_t, [row['hgt'] for row in ref_rows], color=GNSS_COLOR,
            linewidth=0.8, alpha=0.8, label=ref_label)
    ax.plot(ins_t, [row['hgt'] for row in ins_rows], color=INS_COLOR,
            linewidth=0.8, alpha=0.8, label=ins_label)
    ax.set_ylabel('Height (m)')
    ax.set_xlabel('Time (s, relative to first sample)')
    ax.set_title('Height vs Time')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

    ax = axes[1, 1]
    ref_lon = [row['lon'] for row in ref_rows]
    ref_lat = [row['lat'] for row in ref_rows]
    ins_lon = [row['lon'] for row in ins_rows]
    ins_lat = [row['lat'] for row in ins_rows]
    ax.plot(ref_lon, ref_lat, color=GNSS_COLOR, linewidth=0.8, alpha=0.8, label=ref_label)
    ax.plot(ins_lon, ins_lat, color=INS_COLOR, linewidth=0.8, alpha=0.8, label=ins_label)
    marker_kw = dict(s=50, zorder=5, edgecolors='white', linewidths=0.6)
    ax.scatter(ref_lon[0], ref_lat[0], color=GNSS_COLOR, marker='o', **marker_kw)
    ax.scatter(ref_lon[-1], ref_lat[-1], color=GNSS_COLOR, marker='s', **marker_kw)
    ax.scatter(ins_lon[0], ins_lat[0], color=INS_COLOR, marker='o', **marker_kw)
    ax.scatter(ins_lon[-1], ins_lat[-1], color=INS_COLOR, marker='s', **marker_kw)
    ax.scatter([], [], color='gray', s=50, marker='o', label='Start')
    ax.scatter([], [], color='gray', s=50, marker='s', label='End')
    ax.set_xlabel('Longitude (deg)')
    ax.set_ylabel('Latitude (deg)')
    ax.set_title('Trajectory (Lon-Lat)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

    fig.tight_layout()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_velocity(ref_rows, ins_rows, output_path, t0, ins_label='INS', ref_label='GNSS'):
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f'Velocity (ENU): {ins_label} vs {ref_label}', fontsize=14)

    ref_t = rel_time(ref_rows, t0)
    ins_t = rel_time(ins_rows, t0)
    series = [
        ('Ve (m/s)', 've'),
        ('Vn (m/s)', 'vn'),
        ('Vu (m/s)', 'vu'),
    ]
    for ax, (ylabel, key) in zip(axes, series):
        ax.plot(ref_t, [row[key] for row in ref_rows], color=GNSS_COLOR,
                linewidth=0.8, alpha=0.8, label=ref_label)
        ax.plot(ins_t, [row[key] for row in ins_rows], color=INS_COLOR,
                linewidth=0.8, alpha=0.8, label=ins_label)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)

    axes[-1].set_xlabel('Time (s, relative to first sample)')
    fig.tight_layout()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_heading(heading_rows, ins_rows, output_path, t0, heading_bias=None,
                 ins_label='INS', heading_offset=None, ref_label='GNSS'):
    fig, ax = plt.subplots(figsize=(14, 5))
    extras = []
    if heading_offset is not None:
        extras.append(
            f'GNSS offset {heading_offset:.2f}+{DUAL_ANTENNA_TO_VEHICLE_DEG:.0f} deg')
    if heading_bias is not None:
        extras.append(f'{ins_label} bias compensated: {heading_bias:.2f} deg')
    title = f'Heading: {ins_label} vs {ref_label}'
    if extras:
        title = f'{title} ({", ".join(extras)})'
    fig.suptitle(title, fontsize=14)

    heading_t = rel_time(heading_rows, t0)
    ins_t = rel_time(ins_rows, t0)

    def row_heading_deg(row):
        if 'heading' in row:
            return row['heading'] % 360
        return row['azimuth'] % 360

    gnss_label = f'{ref_label} heading' if ref_label == 'GNSS' else f'{ref_label} azimuth'
    if heading_offset is not None:
        total = heading_offset + DUAL_ANTENNA_TO_VEHICLE_DEG
        gnss_label = f'GNSS heading (to vehicle, {total:.1f} deg)'
    ax.plot(heading_t, [row_heading_deg(row) for row in heading_rows], color=GNSS_COLOR,
            linewidth=0.8, alpha=0.8, label=gnss_label)
    if heading_bias is not None:
        ins_hdg = [(row['azimuth'] - heading_bias) % 360 for row in ins_rows]
        ins_hdg_label = f'{ins_label} azimuth (bias compensated)'
    else:
        ins_hdg = [row['azimuth'] % 360 for row in ins_rows]
        ins_hdg_label = f'{ins_label} azimuth'
    ax.plot(ins_t, ins_hdg, color=INS_COLOR,
            linewidth=0.8, alpha=0.8, label=ins_hdg_label)
    ax.set_xlabel('Time (s, relative to first sample)')
    ax.set_ylabel('Heading (deg)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

    fig.tight_layout()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_attitude(ref_rows, ins_rows, output_path, t0, ins_label='SINS', ref_label='INSPVAX'):
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle(f'Attitude: {ins_label} vs {ref_label}', fontsize=14)
    ref_t = rel_time(ref_rows, t0)
    ins_t = rel_time(ins_rows, t0)
    series = [
        ('Roll (deg)', 'roll'),
        ('Pitch (deg)', 'pitch'),
    ]
    for ax, (ylabel, key) in zip(axes, series):
        ax.plot(ref_t, [row[key] for row in ref_rows], color=GNSS_COLOR,
                linewidth=0.8, alpha=0.8, label=ref_label)
        ax.plot(ins_t, [row[key] for row in ins_rows], color=INS_COLOR,
                linewidth=0.8, alpha=0.8, label=ins_label)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
    axes[-1].set_xlabel('Time (s, relative to first sample)')
    fig.tight_layout()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def save_ins_gnss_comparisons(
        output_dir, gnsspos_rows, gnssvel_rows, heading_rows, ins_rows, t0,
        ins_label, heading_bias=None, heading_offset=None):
    outputs = [
        ('position_comparison.png', lambda path: plot_position(
            gnsspos_rows, ins_rows, path, t0, ins_label)),
        ('velocity_comparison.png', lambda path: plot_velocity(
            gnssvel_rows, ins_rows, path, t0, ins_label)),
        ('heading_comparison.png', lambda path: plot_heading(
            heading_rows, ins_rows, path, t0, heading_bias, ins_label, heading_offset)),
    ]
    for filename, plot_func in outputs:
        output_path = Path(output_dir) / filename
        plot_func(output_path)
        print(f'已保存: {output_path}')


def save_ins_pair_comparisons(
        output_dir, ref_rows, ins_rows, t0, ins_label='SINS', ref_label='INSPVAX'):
    outputs = [
        ('position_comparison.png', lambda path: plot_position(
            ref_rows, ins_rows, path, t0, ins_label, ref_label)),
        ('velocity_comparison.png', lambda path: plot_velocity(
            ref_rows, ins_rows, path, t0, ins_label, ref_label)),
        ('heading_comparison.png', lambda path: plot_heading(
            ref_rows, ins_rows, path, t0, None, ins_label, None, ref_label)),
        ('attitude_comparison.png', lambda path: plot_attitude(
            ref_rows, ins_rows, path, t0, ins_label, ref_label)),
    ]
    for filename, plot_func in outputs:
        output_path = Path(output_dir) / filename
        plot_func(output_path)
        print(f'已保存: {output_path}')

