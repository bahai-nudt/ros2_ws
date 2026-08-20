#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 SINS 位姿把雷达 PCD 投到 ENU，生成全局地图。

作用:
    读 output/sins_result.txt 与 raw_data/<lidar_frame>/*.pcd。
    按点时间戳插值雷达位姿（含帧内运动补偿），写出一张 ENU binary PCD。
    外参与滤波一致：默认 calib.json + R_bv；yaml 打开 lidar_imu_extrinsic 则覆盖。
    ENU 原点优先 yaml gloc_origin_lla_deg，否则 gnsspos 首帧。

用法:
    python3 tools/mapping/build_map.py <output_dir> <raw_dir> --config config/parameter_yankuang.yaml
    python3 tools/mapping/build_map.py <output_dir> <raw_dir> --config ... --end-sec 30 --trajectory

参数:
    output_dir              解算输出目录（含 sins_result.txt）
    raw_dir                 解码 raw_data（含 calib.json 与雷达 PCD）
    --config YAML           滤波配置（必填：imu_topic / lidar_topic / R_bv）
    -o, --output PATH       输出 PCD 或目录；默认 <output_dir>/mapping/out.pcd
    --trajectory            从 sins_result.txt 取雷达轨迹，写 trajectory.pcd（红）
    --gnss-trajectory       从 gnsspos.txt 取天线轨迹，写 gnss_trajectory.pcd（绿）
    --stride N              每 N 个有效点留 1 个，默认 30
    --min-range M           剔除近距离点（米），默认 5
    --start-sec / --end-sec 相对 sins 首帧的时间窗

输入:
    <output_dir>/sins_result.txt
    <raw_dir>/calib.json
    <raw_dir>/gnsspos.txt
    <raw_dir>/<lidar_front|... >/*.pcd

输出:
    <output_dir>/mapping/out.pcd
"""

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import pcd_common


def main():
    parser = argparse.ArgumentParser(description='用 SINS 位姿生成 ENU 点云地图')
    parser.add_argument('output_dir', help='解算输出目录（含 sins_result.txt）')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument(
        '--config', metavar='YAML', required=True,
        help='滤波配置 yaml（imu_topic / lidar_topic / R_bv）',
    )
    parser.add_argument(
        '-o', '--output',
        help='输出 PCD 或目录（默认 <output_dir>/mapping/out.pcd）',
    )
    parser.add_argument(
        '--trajectory', action='store_true',
        help='从 sins_result.txt 写出雷达轨迹 PCD（trajectory.pcd，红）',
    )
    parser.add_argument(
        '--gnss-trajectory', action='store_true',
        help='从 gnsspos.txt 写出 GNSS 天线轨迹 PCD（gnss_trajectory.pcd，绿）',
    )
    parser.add_argument('--stride', type=int, default=30, help='抽稀步长，默认 30')
    parser.add_argument('--min-range', type=float, default=5.0, help='近点阈值（米），默认 5')
    parser.add_argument('--start-sec', type=float, default=0.0, help='相对首帧开始秒，默认 0')
    parser.add_argument('--end-sec', type=float, default=None, help='相对首帧结束秒，默认到末尾')
    args = parser.parse_args()

    if args.stride <= 0:
        parser.error('--stride 必须大于 0')
    if args.min_range < 0.0:
        parser.error('--min-range 不能小于 0')
    if args.start_sec < 0.0:
        parser.error('--start-sec 不能小于 0')
    if args.end_sec is not None and args.end_sec <= args.start_sec:
        parser.error('--end-sec 必须大于 --start-sec')

    try:
        output_dir = pcd_common.require_dir(args.output_dir, '解算输出目录')
        raw_dir = pcd_common.require_raw_dir(args.raw_dir)
        sins_path = pcd_common.require_file(output_dir / 'sins_result.txt', 'sins_result.txt')
        config_path = pcd_common.require_file(args.config, '配置 yaml')
        pcd_common.require_file(raw_dir / 'calib.json', 'calib.json')
    except FileNotFoundError as exc:
        print(f'错误: {exc}')
        return 1

    output_path = pcd_common.resolve_map_output(output_dir, args.output)
    traj_path = output_path.with_name('trajectory.pcd') if args.trajectory else None
    gnss_traj_path = (
        output_path.with_name('gnss_trajectory.pcd') if args.gnss_trajectory else None)

    try:
        pcd_common.build_map(
            sins_path, raw_dir, config_path, output_path,
            args.stride, args.min_range, args.start_sec, args.end_sec,
            traj_path, gnss_traj_path)
    except (OSError, ValueError) as exc:
        print(f'错误: {exc}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
