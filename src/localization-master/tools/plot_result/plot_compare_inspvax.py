#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比本仓库 SINS 解算与解码 INSPVAX。

作用:
    读 output/sins_result.txt 与 raw_data/{brand}.ins.txt（同一套 11 列）。
    两边都是 IMU 中心，不做杆臂、不做航向偏移。
    叠画位置 / 速度 / 航向 / 横滚俯仰，并打印最近邻 RMSE。

用法:
    python3 tools/plot_result/plot_compare_inspvax.py <output_dir> <raw_dir>
    python3 tools/plot_result/plot_compare_inspvax.py <output_dir> <raw_dir> -o /path/to/out_dir

参数:
    output_dir         解算输出目录（含 sins_result.txt）
    raw_dir            解码后的 raw_data 目录（含 *.ins.txt）
    -o, --output DIR   输出目录；默认 <output_dir>/plot_result/plot_compare_inspvax

输入:
    <output_dir>/sins_result.txt
    <raw_dir>/{brand}.ins.txt

输出:
    position_comparison.png
    velocity_comparison.png
    heading_comparison.png
    attitude_comparison.png
"""

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import plot_common


def main():
    parser = argparse.ArgumentParser(description='对比 SINS 解算结果与解码 INSPVAX')
    parser.add_argument('output_dir', help='解算输出目录（含 sins_result.txt）')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument(
        '-o', '--output',
        help='输出目录（默认 <output_dir>/plot_result/plot_compare_inspvax）',
    )
    args = parser.parse_args()

    try:
        output_dir = plot_common.require_dir(args.output_dir, '解算输出目录')
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
        sins_path = plot_common.require_file(output_dir / 'sins_result.txt', 'sins_result.txt')
        ins_path = plot_common.resolve_ins_path(raw_dir)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'错误: {exc}')
        return 1

    sins_rows = plot_common.load_ins_compare_rows(sins_path)
    inspvax_rows = plot_common.load_ins_compare_rows(ins_path)
    if not sins_rows or not inspvax_rows:
        print('错误: 输入文件无有效数据')
        return 1

    print(f'读取 INSPVAX:    {len(inspvax_rows)} 条 ({ins_path.name})')
    print(f'读取 SINS:       {len(sins_rows)} 条 ({sins_path.name})')

    t0 = min(inspvax_rows[0]['time'], sins_rows[0]['time'])
    plot_common.print_ins_pair_metrics(inspvax_rows, sins_rows)

    plot_dir = plot_common.resolve_output_dir(
        output_dir, args.output, plot_common.COMPARE_INSPVAX_DIR)
    plot_common.save_ins_pair_comparisons(
        plot_dir, inspvax_rows, sins_rows, t0,
        ins_label='SINS', ref_label='INSPVAX')
    return 0


if __name__ == '__main__':
    sys.exit(main())
