#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比 SINS 解算结果与 GNSS 的位置 / 速度 / 航向。

作用:
    读 output/sins_result.txt（与 {brand}.ins.txt 同为 11 列），叠画 gnsspos /
    gnssvel / heading。GNSS 位置和航向只保留 pos_type=50。
    传入 --config 时开启杆臂补偿，并补偿 GNSS 航向后再与 SINS azimuth 对比：
    heading_plot = raw - heading_offset_deg - 90。
    杆臂与航向逻辑与 plot_compare_inspvax_gnss 完全相同（RFU Cnb，不是 EKF a2mat）。
    默认输出子目录与 plot_compare_inspvax_gnss 相同：plot_compare_inspvax_gnss。

用法:
    python3 tools/plot_result/plot_compare_sins_gnss.py <output_dir> <raw_dir>
    python3 tools/plot_result/plot_compare_sins_gnss.py <output_dir> <raw_dir> \\
        --config config/parameter_yankuang.yaml
    python3 tools/plot_result/plot_compare_sins_gnss.py <output_dir> <raw_dir> \\
        --compensate-heading-bias
    python3 tools/plot_result/plot_compare_sins_gnss.py <output_dir> <raw_dir> -o /path/to/out_dir

参数:
    output_dir                   解算输出目录（含 sins_result.txt）
    raw_dir                      解码后的 raw_data 目录
    --config YAML                滤波配置；传入则开启杆臂补偿，并按 heading_offset_deg 与双天线基线 90° 补偿航向
    --compensate-heading-bias    用前几秒估计 SINS−GNSS 航向常值偏差并补到 SINS
    -o, --output DIR             输出目录；默认 <output_dir>/plot_result/plot_compare_inspvax_gnss
                                 （与 plot_compare_inspvax_gnss 同一子目录名）

输入:
    <output_dir>/sins_result.txt
    <raw_dir>/gnsspos.txt
    <raw_dir>/gnssvel.txt
    <raw_dir>/heading.txt
    <raw_dir>/calib.json         仅 --config 时使用

输出:
    position_comparison.png
    velocity_comparison.png
    heading_comparison.png
"""

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import plot_common


def main():
    parser = argparse.ArgumentParser(description='对比 SINS 解算结果与 GNSS')
    parser.add_argument('output_dir', help='解算输出目录（含 sins_result.txt）')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument(
        '--config',
        metavar='YAML',
        help='滤波配置 yaml；传入则开启杆臂补偿，并按 heading_offset_deg 与双天线基线 90° 补偿 GNSS 航向',
    )
    parser.add_argument(
        '--compensate-heading-bias',
        action='store_true',
        help='估计 SINS-GNSS 航向系统性偏差并补偿到 SINS（双天线安装方向差异）',
    )
    parser.add_argument(
        '-o', '--output',
        help='输出目录（默认 <output_dir>/plot_result/plot_compare_inspvax_gnss）',
    )
    args = parser.parse_args()

    try:
        output_dir = plot_common.require_dir(args.output_dir, '解算输出目录')
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
        sins_path = plot_common.require_file(output_dir / 'sins_result.txt', 'sins_result.txt')
        gnsspos_rows, gnssvel_rows, heading_rows = plot_common.require_gnss_compare_files(raw_dir)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'错误: {exc}')
        return 1

    sins_rows = plot_common.load_ins_compare_rows(sins_path)
    if not gnsspos_rows or not gnssvel_rows or not heading_rows or not sins_rows:
        print('错误: 输入文件无有效数据')
        return 1

    print(f'读取 GNSS POS:   {len(gnsspos_rows)} 条 (pos_type=50)')
    print(f'读取 GNSS VEL:   {len(gnssvel_rows)} 条')
    print(f'读取 GNSS HDG:   {len(heading_rows)} 条 (pos_type=50)')
    print(f'读取 SINS:       {len(sins_rows)} 条 ({sins_path.name})')

    sins_rows, heading_rows, lever_applied, heading_offset, rc = (
        plot_common.apply_compare_config(
            raw_dir, args.config, sins_rows, heading_rows, ins_name='SINS'))
    if rc:
        return rc
    ins_label = 'SINS (antenna)' if lever_applied else 'SINS'

    t0 = min(
        gnsspos_rows[0]['time'],
        gnssvel_rows[0]['time'],
        heading_rows[0]['time'],
        sins_rows[0]['time'],
    )

    heading_bias = None
    if args.compensate_heading_bias:
        heading_bias = plot_common.estimate_heading_bias(
            heading_rows, sins_rows, t0, ins_name='SINS')

    plot_dir = plot_common.resolve_output_dir(
        output_dir, args.output, plot_common.COMPARE_INS_GNSS_DIR)
    plot_common.save_ins_gnss_comparisons(
        plot_dir, gnsspos_rows, gnssvel_rows, heading_rows, sins_rows, t0,
        ins_label, heading_bias, heading_offset)
    return 0


if __name__ == '__main__':
    sys.exit(main())
