#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GNSS 解码（自研）：pos / vel / heading -> CSV。

BADP parse_gnss 的 vel/heading txt 有 bug，此处自行从 Cyber Record 解码。
"""

import sys
from pathlib import Path

import decode_common as common

POS_NAME = 'GNSS POS'
POS_HEADER = 'timestamp,sol_status,pos_type,lat,lon,hgt,lat_std,lon_std,hgt_std\n'

VEL_NAME = 'GNSS VEL'
VEL_HEADER = 'timestamp,vel_type,hor_speed,trk_gnd,ver_speed\n'

HEADING_NAME = 'Heading'
HEADING_HEADER = 'timestamp,sol_status,pos_type,length,heading,heading_std\n'


def _load_bestgnsspos():
    try:
        from modules.msgs.drivers.bynav.bestgnsspos_pb2 import BESTGNSSPOS
    except ImportError as exc:
        print(f'错误: 导入 BESTGNSSPOS protobuf 失败: {exc}')
        sys.exit(1)
    return BESTGNSSPOS


def _load_bestvel():
    try:
        from modules.msgs.drivers.bynav.bestvel_pb2 import BESTVEL
    except ImportError as exc:
        print(f'错误: 导入 BESTVEL protobuf 失败: {exc}')
        sys.exit(1)
    return BESTVEL


def _load_heading2():
    try:
        from modules.msgs.drivers.bynav.heading2_pb2 import HEADING2
    except ImportError as exc:
        print(f'错误: 导入 HEADING2 protobuf 失败: {exc}')
        sys.exit(1)
    return HEADING2


def _parse_pos(msg_content, msg_cls):
    msg = msg_cls()
    msg.ParseFromString(msg_content)
    obs_time = common.gps_week_to_unix(msg.nov_header.gps_week_number,
                                       msg.nov_header.gps_week_milliseconds)
    sol_status = msg.sol_status.status if hasattr(msg.sol_status, 'status') else 0
    pos_type = msg.pos_type.type if hasattr(msg.pos_type, 'type') else 0
    return (f'{obs_time:.6f},{sol_status},{pos_type},'
            f'{msg.lat:.12f},{msg.lon:.12f},{msg.hgt:.6f},'
            f'{msg.lat_stdev:.6f},{msg.lon_stdev:.6f},{msg.hgt_stdev:.6f}\n')


def _parse_vel(msg_content, msg_cls):
    msg = msg_cls()
    msg.ParseFromString(msg_content)
    obs_time = common.gps_week_to_unix(msg.nov_header.gps_week_number,
                                       msg.nov_header.gps_week_milliseconds)
    vel_type = msg.pos_type.type if hasattr(msg.pos_type, 'type') else 0
    return (f'{obs_time:.6f},{vel_type},{msg.hor_speed:.6f},'
            f'{msg.trk_gnd:.6f},{msg.ver_speed:.6f}\n')


def _parse_heading(msg_content, msg_cls):
    msg = msg_cls()
    msg.ParseFromString(msg_content)
    obs_time = common.gps_week_to_unix(msg.nov_header.gps_week_number,
                                       msg.nov_header.gps_week_milliseconds)
    sol_status = msg.sol_status.status if hasattr(msg.sol_status, 'status') else 0
    pos_type = msg.pos_type.type if hasattr(msg.pos_type, 'type') else 0
    return (f'{obs_time:.6f},{sol_status},{pos_type},'
            f'{msg.length:.6f},{msg.heading:.6f},{msg.heading_stdev:.6f}\n')


def _decode_channel(record_path, topic, output_file, header, name, parse_func, msg_cls,
                    max_workers, append):
    output_file = Path(output_file)
    records = common.collect_channel_messages(record_path, topic)
    if not records:
        print(f'  跳过 {name}（record 中未发现 {topic}）')
        return False

    lines, parse_errors = common.parse_records(records, parse_func, msg_cls, max_workers)
    mode = 'a' if append and output_file.exists() else 'w'
    with open(output_file, mode, encoding='utf-8') as file:
        if mode == 'w':
            file.write(header)
        file.writelines(lines)
    common.print_decode_summary(name, len(lines), parse_errors, output_file)
    return len(lines) > 0


def decode_pos(record_path, topic, output_dir, max_workers=1, append=False):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return _decode_channel(record_path, topic, output_dir / 'gnsspos.txt', POS_HEADER, POS_NAME,
                           _parse_pos, _load_bestgnsspos(), max_workers, append)


def decode_vel(record_path, topic, output_dir, max_workers=1, append=False):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return _decode_channel(record_path, topic, output_dir / 'gnssvel.txt', VEL_HEADER, VEL_NAME,
                           _parse_vel, _load_bestvel(), max_workers, append)


def decode_heading(record_path, topic, output_dir, max_workers=1, append=False):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return _decode_channel(record_path, topic, output_dir / 'heading.txt', HEADING_HEADER,
                           HEADING_NAME, _parse_heading, _load_heading2(), max_workers, append)
