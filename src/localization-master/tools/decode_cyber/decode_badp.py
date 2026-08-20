#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""调用 /breton_pilot 已安装的 BADP 解析脚本解码 IMU / INSPVAX / calib / LiDAR。"""

import json
import os
import subprocess
import sys
from pathlib import Path

BRETON_PILOT = Path('/breton_pilot')
BADP_PARSE_DIR = BRETON_PILOT / 'scripts' / 'recorder' / 'parse'
CALIB_TOPIC = '/calibration/sensor_parameters'

LIDAR_TOPICS = {
    'lidar_front': '/rslidar/em4_front/raw',
    'lidar_rear': '/rslidar/em4_rear/raw',
    'lidar_left': '/rslidar/m1p_left/raw',
    'lidar_right': '/rslidar/m1p_right/raw',
}
LIDAR_DECODER_KEYS = frozenset(LIDAR_TOPICS)
LIDAR_TOPIC_BY_KEY = LIDAR_TOPICS
LIDAR_KEY_BY_TOPIC = {topic: key for key, topic in LIDAR_TOPICS.items()}
LIDAR_OUTPUT_DIRS = {topic: key for key, topic in LIDAR_TOPICS.items()}


def _ensure_badp_parse_path():
    parse_path = str(BADP_PARSE_DIR)
    if parse_path not in sys.path:
        sys.path.insert(0, parse_path)


def _reshape_transform_matrix(values):
    if not isinstance(values, list):
        return values
    if len(values) != 16:
        return values
    if not all(isinstance(value, (int, float)) for value in values):
        return values
    return [values[index:index + 4] for index in range(0, 16, 4)]


def _convert_calib_payload_keys(payload):
    if not isinstance(payload, dict):
        return payload

    key_map = {
        'frame_id': 'frameId',
        'target_frame_id': 'targetFrameId',
        'transform_matrix': 'transformMatrix',
    }

    def _convert(obj):
        if isinstance(obj, dict):
            converted = {}
            for key, value in obj.items():
                new_key = key_map.get(key, key)
                converted[new_key] = _convert(value)
            if 'transformMatrix' in converted:
                converted['transformMatrix'] = _reshape_transform_matrix(
                    converted['transformMatrix'])
            return converted
        if isinstance(obj, list):
            return [_convert(item) for item in obj]
        return obj

    return _convert(payload)


def _find_decode_rslidar():
    exe = BRETON_PILOT / 'bin' / 'decode_rslidar'
    if exe.is_file() and os.access(exe, os.X_OK):
        return exe
    from shutil import which
    path = which('decode_rslidar')
    if path:
        return Path(path)
    return None


def _load_existing_json_list(json_path, list_key):
    if not json_path.is_file():
        return []
    try:
        with open(json_path, 'r', encoding='utf-8') as file:
            payload = json.load(file)
    except (OSError, json.JSONDecodeError):
        return []
    if not isinstance(payload, dict):
        return []
    rows = payload.get(list_key)
    return rows if isinstance(rows, list) else []


def _save_imu_files(output_dir, brand, data_list):
    from parse_imu import BewisIMUParser, BynavIMUParser, IMUDataParser, ShangyuIMUParser

    brand_lower = brand.lower()
    if brand_lower == 'bynav':
        imu_parser = BynavIMUParser()
    elif brand_lower == 'shangyu':
        imu_parser = ShangyuIMUParser()
    elif brand_lower == 'bewis':
        imu_parser = BewisIMUParser()
    else:
        print(f'  跳过 IMU：不支持的 brand={brand}')
        return False

    output_path = os.path.join(str(output_dir), f'{brand_lower}.imu.json')
    parser = IMUDataParser(imu_parser, verbose=False)
    parser.imu_data_list = data_list
    if not parser.imu_data_list:
        return False
    parser.save_to_json(output_path)
    parser.save_to_txt(output_path)
    print(f'  IMU ({brand}): {len(parser.imu_data_list)} 条 -> {brand_lower}.imu.txt')
    return True


def _save_ins_files(output_dir, brand, data_list):
    from parse_ins import BynavINSParser, INSDataParser, ShangyuINSParser

    brand_lower = brand.lower()
    if brand_lower == 'bynav':
        ins_parser = BynavINSParser()
    elif brand_lower == 'shangyu':
        ins_parser = ShangyuINSParser()
    else:
        print(f'  跳过 INSPVAX：不支持的 brand={brand}')
        return False

    output_path = os.path.join(str(output_dir), f'{brand_lower}.ins.json')
    parser = INSDataParser(ins_parser, verbose=False)
    parser.ins_data_list = data_list
    if not parser.ins_data_list:
        return False
    parser.save_to_json(output_path)
    parser.save_to_txt(output_path)
    print(f'  INSPVAX ({brand}): {len(parser.ins_data_list)} 条 -> {brand_lower}.ins.txt')
    return True


def decode_imu(record_path, output_dir, brand, append=False):
    _ensure_badp_parse_path()
    try:
        from parse_imu import read_imu
    except ImportError as exc:
        print(f'错误: 无法导入 BADP parse_imu: {exc}')
        print('请先 source /breton_pilot/scripts/env_setup.sh')
        return False

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    brand_lower = brand.lower()

    try:
        rows = read_imu(str(record_path), str(output_dir), imu_type=brand_lower,
                        return_data=True, verbose=False)
    except TypeError:
        rows = read_imu(str(record_path), str(output_dir), imu_type=brand_lower,
                        return_data=True)

    if not isinstance(rows, list) or not rows:
        print(f'  跳过 IMU ({brand})：record 中无数据')
        return False

    if append:
        json_path = output_dir / f'{brand_lower}.imu.json'
        rows = _load_existing_json_list(json_path, 'imu_data') + rows

    return _save_imu_files(output_dir, brand_lower, rows)


def decode_inspvax(record_path, output_dir, brand, append=False):
    _ensure_badp_parse_path()
    try:
        from parse_ins import read_ins
    except ImportError as exc:
        print(f'错误: 无法导入 BADP parse_ins: {exc}')
        print('请先 source /breton_pilot/scripts/env_setup.sh')
        return False

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    brand_lower = brand.lower()

    try:
        rows = read_ins(str(record_path), str(output_dir), ins_type=brand_lower,
                        return_data=True, verbose=False)
    except TypeError:
        rows = read_ins(str(record_path), str(output_dir), ins_type=brand_lower,
                        return_data=True)

    if not isinstance(rows, list) or not rows:
        print(f'  跳过 INSPVAX ({brand})：record 中无数据')
        return False

    if append:
        json_path = output_dir / f'{brand_lower}.ins.json'
        rows = _load_existing_json_list(json_path, 'ins_data') + rows

    return _save_ins_files(output_dir, brand_lower, rows)


def decode_calib(record_path, output_dir, counts=None):
    if counts is not None and counts.get(CALIB_TOPIC, 0) <= 0:
        print(f'  跳过标定参数（record 中未发现 {CALIB_TOPIC}）')
        return False

    _ensure_badp_parse_path()
    try:
        from parse_calib import read_calib
    except ImportError as exc:
        print(f'错误: 无法导入 BADP parse_calib: {exc}')
        print('请先 source /breton_pilot/scripts/env_setup.sh')
        return False

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        records = read_calib(str(record_path), str(output_dir), return_data=True, verbose=False)
    except TypeError:
        records = read_calib(str(record_path), str(output_dir), return_data=True)

    if not isinstance(records, list) or not records:
        print('  跳过标定参数：record 中无 /calibration/sensor_parameters 数据')
        return False

    latest = None
    for record in records:
        if not isinstance(record, dict):
            continue
        payload = record.get('sensor_calibration')
        if isinstance(payload, dict):
            latest = payload

    if not isinstance(latest, dict):
        print('  跳过标定参数：无有效 sensor_calibration')
        return False

    calib_json = output_dir / 'calib.json'
    normalized = _convert_calib_payload_keys(latest)
    with open(calib_json, 'w', encoding='utf-8') as file:
        json.dump(normalized, file, indent=4, ensure_ascii=False)
    print(f'  标定参数: {len(records)} 条，已输出 {calib_json}')
    return True


def decode_lidar(record_path, output_dir, topics, dense_points=False):
    if not topics:
        print('  跳过雷达（record 中未发现 rslidar RawPacket topic）')
        return False

    exe = _find_decode_rslidar()
    if exe is None:
        print('错误: 找不到 decode_rslidar，请先 source /breton_pilot/scripts/env_setup.sh')
        return False

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    cmd = [str(exe), str(record_path), str(output_dir)]
    for topic in topics:
        cmd.extend(['--topic', topic])
    if dense_points:
        cmd.append('--dense-points')

    print('  调用:', ' '.join(cmd))
    result = subprocess.run(cmd, cwd=str(BRETON_PILOT), check=False)
    if result.returncode != 0:
        print(f'错误: decode_rslidar 退出码 {result.returncode}')
        return False

    ok = True
    for topic in topics:
        out_dir = LIDAR_OUTPUT_DIRS.get(topic, topic.strip('/').replace('/', '_'))
        lidar_dir = output_dir / out_dir
        pcd_count = len(list(lidar_dir.glob('*.pcd'))) if lidar_dir.is_dir() else 0
        print(f'  PCD: {out_dir}/ {pcd_count} 帧')
        if pcd_count == 0:
            ok = False
    return ok
