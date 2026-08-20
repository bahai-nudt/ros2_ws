#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cyber Record 解码总入口（BADP 薄封装 + 自研 GNSS）。

GNSS pos/vel/heading 走本仓库 decode_gnss.py（CSV）。
IMU / INSPVAX / calib / LiDAR 调用 /breton_pilot 已安装 BADP 工具。
GLOC 仍走本仓库 decode_gloc.py。

=============================================================================
快速上手
=============================================================================

0) 环境（解码前执行一次）:
    source /breton_pilot/scripts/env_setup.sh

1) 查看 record channel 与解码类型:
    python3 tools/decode_cyber/decode_main.py /abs/path/to/record.00000 --list

2) 全量解码（默认输出 <record 目录>/raw_data/）:
    python3 tools/decode_cyber/decode_main.py /abs/path/to/record.00000

3) 指定类型与输出目录:
    python3 tools/decode_cyber/decode_main.py /abs/path/to/record.00000 \\
        --topics imu,gnsspos,inspvax,calib,lidar_front \\
        --output /abs/path/to/raw_data

=============================================================================
--topics 解码类型
=============================================================================

    单个: imu | gnsspos | gnssvel | heading | inspvax | gloc | calib |
          lidar_front | lidar_rear | lidar_left | lidar_right
    简写: lidar_all（等价于上述四路雷达全部解码）
    全部: all

=============================================================================
输出契约（默认 <record>/raw_data/）
=============================================================================

IMU (--topics imu)           BADP: {bynav,shangyu,bewis}.imu.txt
INSPVAX (--topics inspvax)   BADP: {bynav,shangyu}.ins.txt
calib (--topics calib)       BADP: calib.json
LiDAR (--topics lidar_*)     BADP: lidar_front/*.pcd 等（一路一个 decoder key）
GNSS pos/vel/heading         本仓库 CSV: gnsspos.txt gnssvel.txt heading.txt
GLOC (--topics gloc)         本仓库: gloc.txt
"""

import argparse
import os
import sys
from collections import namedtuple
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import decode_badp
import decode_common
import decode_gloc
import decode_gnss

BRETON_PILOT = Path('/breton_pilot')
CALIB_TOPIC = decode_badp.CALIB_TOPIC
GLOC_TOPIC_PREFIX = '/localization/gloc'

ALL_DECODER_KEYS = frozenset({
    'gnsspos', 'gnssvel', 'heading', 'imu', 'inspvax', 'gloc', 'calib',
}) | decode_badp.LIDAR_DECODER_KEYS

LIDAR_DECODER_KEYS = decode_badp.LIDAR_DECODER_KEYS

MatchedTopic = namedtuple('MatchedTopic', [
    'decoder_key', 'channel', 'brand', 'proto', 'output_file', 'count',
])


def topic_brand(channel):
    parts = channel.strip('/').split('/')
    return parts[0] if parts else 'unknown'


def read_record_info(record_path):
    reader = decode_common.create_record_reader(record_path)
    channels = list(reader.get_channellist())
    msg_types = {}
    for channel in channels:
        try:
            msg_types[channel] = reader.get_messagetype(channel)
        except Exception:
            msg_types[channel] = '<unknown>'

    counts = {}
    for channel_name, msg_content, _datatype, _timestamp in reader.read_messages():
        if msg_content is None:
            continue
        counts[channel_name] = counts.get(channel_name, 0) + 1

    return channels, counts, msg_types


def is_camera_channel(channel, msg_type):
    text = f'{channel} {msg_type}'.lower()
    return '/camera/' in text or '/h265' in text or 'compressedimage' in text


def is_rslidar_raw_channel(channel, msg_type):
    msg_type_lower = (msg_type or '').lower()
    return ('badp.drivers.rslidar.rawpacket' in msg_type_lower or
            msg_type_lower.endswith('.rawpacket') or
            (channel.startswith('/rslidar/') and channel.endswith('/raw')))


def match_channel(channel, count, msg_type):
    brand = topic_brand(channel)

    if channel.endswith('/bestgnsspos'):
        return MatchedTopic('gnsspos', channel, brand, 'bynav.BESTGNSSPOS',
                            'gnsspos.txt', count)
    if channel.endswith('/bestvel'):
        return MatchedTopic('gnssvel', channel, brand, 'bynav.BESTVEL',
                            'gnssvel.txt', count)
    if channel.endswith('/heading2'):
        return MatchedTopic('heading', channel, brand, 'bynav.HEADING2',
                            'heading.txt', count)
    if channel.endswith('/inspvax'):
        return MatchedTopic('inspvax', channel, brand, 'bynav.INSPVAX',
                            f'{brand}.ins.txt', count)
    if channel.endswith('/imu/data_raw'):
        if '/bewis/' in channel:
            imu_brand = 'bewis'
            proto = 'bewis.IMU'
        else:
            imu_brand = brand
            proto = 'bynav.IMU'
        return MatchedTopic('imu', channel, imu_brand, proto,
                            f'{imu_brand}.imu.txt', count)
    if channel == CALIB_TOPIC:
        return MatchedTopic('calib', channel, '', 'calibration.SensorCalibration',
                            'calib.json', count)
    if channel.startswith(GLOC_TOPIC_PREFIX):
        return MatchedTopic('gloc', channel, '', 'localization.Gloc', 'gloc.txt', count)
    if is_rslidar_raw_channel(channel, msg_type):
        decoder_key = decode_badp.LIDAR_KEY_BY_TOPIC.get(channel)
        if not decoder_key:
            return None
        out_dir = decode_badp.LIDAR_OUTPUT_DIRS[channel]
        return MatchedTopic(decoder_key, channel, '', 'decode_rslidar',
                            f'{out_dir}/*.pcd', count)
    return None


def match_channels(channels, counts, msg_types):
    matched = []
    for channel in channels:
        count = counts.get(channel, 0)
        if count <= 0:
            continue
        item = match_channel(channel, count, msg_types.get(channel, ''))
        if item:
            matched.append(item)
    return matched


def parse_topics_arg(topics_str):
    if topics_str.strip().lower() == 'all':
        return set(ALL_DECODER_KEYS)
    selected = {item.strip().lower() for item in topics_str.split(',') if item.strip()}
    if 'lidar_all' in selected:
        selected.discard('lidar_all')
        selected |= LIDAR_DECODER_KEYS
    unknown = selected - ALL_DECODER_KEYS
    if unknown:
        print(f'错误: 未知 decoder key: {", ".join(sorted(unknown))}')
        lidar_keys = ', '.join(sorted(LIDAR_DECODER_KEYS))
        print(f'合法值: {", ".join(sorted(ALL_DECODER_KEYS))}, lidar_all, all')
        print(f'  lidar_all 等价于: {lidar_keys}')
        sys.exit(1)
    return selected


def filter_matched(matched, selected_keys):
    return [item for item in matched if item.decoder_key in selected_keys]


def print_match_list(record_path, matched, counts, msg_types):
    print(f'Record: {record_path}')
    matched_channels = {item.channel for item in matched}
    for item in sorted(matched, key=lambda x: x.channel):
        print(f'  {item.channel:40s} ({item.count:5d}) -> '
              f'{item.decoder_key} / {item.proto} -> {item.output_file}')

    for channel in sorted(counts.keys()):
        if counts.get(channel, 0) <= 0 or channel in matched_channels:
            continue
        if is_camera_channel(channel, msg_types.get(channel, '')):
            continue
        print(f'  {channel:40s} ({counts[channel]:5d}) -> (不支持)')


def pick_best_gloc(matched):
    gloc_items = [item for item in matched if item.decoder_key == 'gloc']
    if not gloc_items:
        return None
    return max(gloc_items, key=lambda item: item.count)


def list_record_files(dir_path):
    records = []
    for entry in sorted(dir_path.iterdir()):
        if not entry.is_file():
            continue
        if '.record' not in entry.name:
            continue
        records.append(entry)
    return records


def run_decoders(record_path, decode_dir, filtered, matched, selected_keys, counts, workers,
                 append=False, dense_points=False):
    if 'imu' in selected_keys:
        print('\n=== 解码 IMU (BADP) ===')
        for item in [i for i in filtered if i.decoder_key == 'imu']:
            decode_badp.decode_imu(record_path, decode_dir, item.brand, append)

    if 'gnsspos' in selected_keys:
        print('\n=== 解码 GNSS POS ===')
        for item in [i for i in filtered if i.decoder_key == 'gnsspos']:
            decode_gnss.decode_pos(record_path, item.channel, decode_dir, workers, append)

    if 'gnssvel' in selected_keys:
        print('\n=== 解码 GNSS VEL ===')
        for item in [i for i in filtered if i.decoder_key == 'gnssvel']:
            decode_gnss.decode_vel(record_path, item.channel, decode_dir, workers, append)

    if 'heading' in selected_keys:
        print('\n=== 解码 Heading ===')
        for item in [i for i in filtered if i.decoder_key == 'heading']:
            decode_gnss.decode_heading(record_path, item.channel, decode_dir, workers, append)

    if 'inspvax' in selected_keys:
        print('\n=== 解码 INSPVAX (BADP) ===')
        inspvax_items = [item for item in filtered if item.decoder_key == 'inspvax']
        if not inspvax_items:
            print('  跳过 INSPVAX（record 中未发现 inspvax topic）')
        for item in inspvax_items:
            decode_badp.decode_inspvax(record_path, decode_dir, item.brand, append)

    if 'gloc' in selected_keys:
        print('\n=== 解码 Gloc ===')
        gloc_item = pick_best_gloc(matched)
        if gloc_item:
            decode_gloc.decode(record_path, gloc_item.channel, decode_dir, workers, append)
        else:
            print('  跳过 Gloc（record 中未发现 /localization/gloc topic）')

    if 'calib' in selected_keys:
        print('\n=== 解码标定参数 (BADP) ===')
        decode_badp.decode_calib(record_path, decode_dir, counts)

    selected_lidar_keys = selected_keys & LIDAR_DECODER_KEYS
    if selected_lidar_keys:
        print('\n=== 解码 RoboSense 雷达 (BADP) ===')
        lidar_topics = sorted({
            item.channel for item in filtered if item.decoder_key in selected_lidar_keys
        })
        if not lidar_topics:
            print(f'  跳过雷达（record 中未发现: {", ".join(sorted(selected_lidar_keys))}）')
        else:
            decode_badp.decode_lidar(record_path, decode_dir, lidar_topics, dense_points)


def decode_one_record(record_path, decode_dir, selected_keys, workers, append=False,
                      dense_points=False):
    print('=== 扫描 Cyber Record ===')
    channels, counts, msg_types = read_record_info(record_path)
    matched = match_channels(channels, counts, msg_types)
    filtered = filter_matched(matched, selected_keys)

    print(f'Record: {record_path}')
    print(f'解码类型: {", ".join(sorted(selected_keys))}')
    print(f'匹配 channel: {len(filtered)} 个')

    run_decoders(record_path, decode_dir, filtered, matched, selected_keys, counts, workers,
                 append=append, dense_points=dense_points)

    camera_channels = [channel for channel in counts
                       if is_camera_channel(channel, msg_types.get(channel, ''))]
    if camera_channels:
        print('\n=== 跳过相机 ===')
        for channel in sorted(camera_channels):
            print(f'  检测到相机 channel，不解码: {channel}')


def main():
    parser = argparse.ArgumentParser(description='Cyber Record 解码（BADP 薄封装 + 自研 GNSS）')
    parser.add_argument('record_file', help='Cyber Record 文件或含多个 record 的目录绝对路径')
    parser.add_argument('--topics', default='all',
                        help='解码类型，逗号分隔或 all（默认 all）')
    parser.add_argument('--output', default=None,
                        help='输出目录，默认 <record_dir>/raw_data/')
    parser.add_argument('--list', action='store_true',
                        help='列出 channel 与 decoder 匹配关系并退出')
    parser.add_argument('--workers', type=int, default=min(os.cpu_count() or 1, 8),
                        help='GNSS/GLOC protobuf 并发线程数，默认 min(cpu_count, 8)')
    parser.add_argument('--dense-points', action='store_true',
                        help='LiDAR 解码时透传 --dense-points 给 decode_rslidar')
    args = parser.parse_args()

    if not BRETON_PILOT.is_dir():
        print(f'错误: BADP 安装目录不存在: {BRETON_PILOT}')
        print('请确认 BADP 已安装且路径正确；新终端需执行:')
        print('  source /breton_pilot/scripts/env_setup.sh')
        return 1

    input_path = Path(args.record_file).resolve()
    if not input_path.exists():
        print(f'错误: 路径不存在: {input_path}')
        return 1

    if input_path.is_file():
        record_paths = [input_path]
        base_dir = input_path.parent
    elif input_path.is_dir():
        record_paths = list_record_files(input_path)
        if not record_paths:
            print(f'错误: 目录中未找到 .record 文件: {input_path}')
            return 1
        base_dir = input_path
    else:
        print(f'错误: 不是文件或目录: {input_path}')
        return 1

    if args.list:
        if len(record_paths) > 1:
            print(f'目录: {input_path}（{len(record_paths)} 个 record，按文件名排序）')
            for index, record_path in enumerate(record_paths, start=1):
                print(f'  [{index}/{len(record_paths)}] {record_path.name}')
            print()
        for record_path in record_paths:
            channels, counts, msg_types = read_record_info(record_path)
            matched = match_channels(channels, counts, msg_types)
            print_match_list(record_path, matched, counts, msg_types)
            if len(record_paths) > 1:
                print()
        return 0

    selected_keys = parse_topics_arg(args.topics)

    if args.output:
        decode_dir = Path(args.output).resolve()
    else:
        decode_dir = base_dir / 'raw_data'
    decode_dir.mkdir(parents=True, exist_ok=True)

    workers = max(1, args.workers)
    print(f'输出目录: {decode_dir}')
    if len(record_paths) > 1:
        print(f'批量模式: {len(record_paths)} 个 record')

    for index, record_path in enumerate(record_paths):
        if len(record_paths) > 1:
            print(f'\n========== [{index + 1}/{len(record_paths)}] {record_path.name} ==========')
        decode_one_record(record_path, decode_dir, selected_keys, workers,
                          append=(index > 0), dense_points=args.dense_points)

    print('\n=== 解码完成 ===')
    print(f'解码结果: {decode_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
