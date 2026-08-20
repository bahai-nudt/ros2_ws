#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cyber Record /localization/gloc 解码库（由 decode_main.py 调用）。

从 Cyber Record 解码 badp.localization.Gloc，字段与 gloc_result.txt 一致。
依赖: source Cyber RT / protobuf 环境，可导入 modules.msgs.localization.gloc_pb2.Gloc。
"""

import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

GLOC_TOPIC = '/localization/gloc'

CSV_HEADER = (
    'timestamp,obs_time,state,'
    'position_x,position_y,position_z,'
    'orientation_w,orientation_x,orientation_y,orientation_z,'
    'linear_velocity_x,linear_velocity_y,linear_velocity_z,'
    'linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,'
    'angular_velocity_x,angular_velocity_y,angular_velocity_z,'
    'latitude,longitude,altitude,'
    'roll,pitch,yaw,velocity\n'
)

FIELD_NAMES = [
    'timestamp', 'obs_time', 'state',
    'position_x', 'position_y', 'position_z',
    'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z',
    'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
    'latitude', 'longitude', 'altitude',
    'roll', 'pitch', 'yaw', 'velocity',
]


def create_record_reader(record_path):
    from cyber.python.cyber_py3 import record
    path = str(record_path)
    try:
        return record.RecordReader(path)
    except SystemError:
        return record.RecordReader(path.encode('utf-8'))


def load_gloc_protobuf():
    try:
        from modules.msgs.localization.gloc_pb2 import Gloc
    except ImportError as exc:
        print(f'错误: 导入 Gloc protobuf 失败: {exc}')
        sys.exit(1)
    return Gloc


def gloc_message_to_row(msg):
    return {
        'timestamp': msg.timestamp,
        'obs_time': msg.obs_time,
        'state': int(msg.state),
        'position_x': msg.position_x,
        'position_y': msg.position_y,
        'position_z': msg.position_z,
        'orientation_w': msg.orientation_w,
        'orientation_x': msg.orientation_x,
        'orientation_y': msg.orientation_y,
        'orientation_z': msg.orientation_z,
        'linear_velocity_x': msg.linear_velocity_x,
        'linear_velocity_y': msg.linear_velocity_y,
        'linear_velocity_z': msg.linear_velocity_z,
        'linear_acceleration_x': msg.linear_acceleration_x,
        'linear_acceleration_y': msg.linear_acceleration_y,
        'linear_acceleration_z': msg.linear_acceleration_z,
        'angular_velocity_x': msg.angular_velocity_x,
        'angular_velocity_y': msg.angular_velocity_y,
        'angular_velocity_z': msg.angular_velocity_z,
        'latitude': msg.latitude,
        'longitude': msg.longitude,
        'altitude': msg.altitude,
        'roll': msg.roll,
        'pitch': msg.pitch,
        'yaw': msg.yaw,
        'velocity': msg.velocity,
    }


def format_gloc_line(msg):
    row = gloc_message_to_row(msg)
    return (
        f'{row["timestamp"]:.6f},{row["obs_time"]:.6f},{row["state"]},'
        f'{row["position_x"]:.6f},{row["position_y"]:.6f},{row["position_z"]:.6f},'
        f'{row["orientation_w"]:.9f},{row["orientation_x"]:.9f},'
        f'{row["orientation_y"]:.9f},{row["orientation_z"]:.9f},'
        f'{row["linear_velocity_x"]:.6f},{row["linear_velocity_y"]:.6f},'
        f'{row["linear_velocity_z"]:.6f},'
        f'{row["linear_acceleration_x"]:.6f},{row["linear_acceleration_y"]:.6f},'
        f'{row["linear_acceleration_z"]:.6f},'
        f'{row["angular_velocity_x"]:.6f},{row["angular_velocity_y"]:.6f},'
        f'{row["angular_velocity_z"]:.6f},'
        f'{row["latitude"]:.12f},{row["longitude"]:.12f},{row["altitude"]:.6f},'
        f'{row["roll"]:.6f},{row["pitch"]:.6f},{row["yaw"]:.6f},'
        f'{row["velocity"]:.6f}\n'
    )


def parse_gloc_message(msg_content, msg_cls=None):
    if msg_cls is None:
        msg_cls = load_gloc_protobuf()
    msg = msg_cls()
    msg.ParseFromString(msg_content)
    return gloc_message_to_row(msg)


def collect_gloc_messages(record_path, topic=GLOC_TOPIC):
    reader = create_record_reader(record_path)
    records = []
    for channel_name, msg_content, _, _ in reader.read_messages():
        if channel_name != topic or msg_content is None:
            continue
        records.append((len(records) + 1, msg_content))
    return records


def decode_gloc_messages(records, max_workers=1, msg_cls=None):
    if msg_cls is None:
        msg_cls = load_gloc_protobuf()

    rows = []
    parse_errors = 0
    max_workers = max(1, max_workers)

    if max_workers <= 1 or len(records) <= 1:
        for _seq, msg_content in records:
            try:
                rows.append(parse_gloc_message(msg_content, msg_cls))
            except Exception:
                parse_errors += 1
        rows.sort(key=lambda item: item['obs_time'])
        return rows, parse_errors

    parsed = []
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_seq = {
            executor.submit(parse_gloc_message, msg_content, msg_cls): seq
            for seq, msg_content in records
        }
        for future in as_completed(future_to_seq):
            seq = future_to_seq[future]
            try:
                parsed.append((seq, future.result()))
            except Exception:
                parse_errors += 1
    parsed.sort(key=lambda item: item[0])
    rows = [row for _, row in parsed]
    return rows, parse_errors


def decode_gloc_channel(record_path, topic=GLOC_TOPIC, max_workers=1):
    records = collect_gloc_messages(record_path, topic)
    if not records:
        return [], 0
    return decode_gloc_messages(records, max_workers=max_workers)


def row_to_csv_line(row):
    return (
        f'{row["timestamp"]:.6f},{row["obs_time"]:.6f},{row["state"]},'
        f'{row["position_x"]:.6f},{row["position_y"]:.6f},{row["position_z"]:.6f},'
        f'{row["orientation_w"]:.9f},{row["orientation_x"]:.9f},'
        f'{row["orientation_y"]:.9f},{row["orientation_z"]:.9f},'
        f'{row["linear_velocity_x"]:.6f},{row["linear_velocity_y"]:.6f},'
        f'{row["linear_velocity_z"]:.6f},'
        f'{row["linear_acceleration_x"]:.6f},{row["linear_acceleration_y"]:.6f},'
        f'{row["linear_acceleration_z"]:.6f},'
        f'{row["angular_velocity_x"]:.6f},{row["angular_velocity_y"]:.6f},'
        f'{row["angular_velocity_z"]:.6f},'
        f'{row["latitude"]:.12f},{row["longitude"]:.12f},{row["altitude"]:.6f},'
        f'{row["roll"]:.6f},{row["pitch"]:.6f},{row["yaw"]:.6f},'
        f'{row["velocity"]:.6f}\n'
    )


def write_gloc_csv(rows, output_path, append=False):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    mode = 'a' if append and output_path.exists() else 'w'
    with open(output_path, mode, encoding='utf-8') as file:
        if mode == 'w':
            file.write(CSV_HEADER)
        for row in rows:
            file.write(row_to_csv_line(row))


def decode(record_path, topic, output_dir, max_workers=1, append=False):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rows, parse_errors = decode_gloc_channel(record_path, topic=topic, max_workers=max(1, max_workers))
    if not rows:
        print(f'  跳过 Gloc（record 中未发现 {topic}）')
        return False

    output_file = output_dir / 'gloc.txt'
    write_gloc_csv(rows, output_file, append=append)
    label = f'Gloc ({topic})'
    if parse_errors > 0:
        print(f'  {label}: {len(rows)} 条，解析失败 {parse_errors} 条，已输出 {output_file}')
    else:
        print(f'  {label}: {len(rows)} 条，已输出 {output_file}')
    return True

