#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比车端 GLOC 与 GNSS 的位置 / 速度 / 航向。

作用:
    叠画 gloc.txt 与 gnsspos / gnssvel / heading。
    GNSS 位置和航向只保留 pos_type=50。GLOC 是车体系发布位姿，不做 IMU 杆臂。
    读取 / 出图与 plot_compare_gloc_gnss（plot_result）共用 plot_common；
    仅输入文件是 raw_data/gloc.txt。

用法:
    python3 tools/plot_raw_data/plot_compare_gloc_gnss.py <raw_dir>
    python3 tools/plot_raw_data/plot_compare_gloc_gnss.py <raw_dir> -o /path/to/out_dir

参数:
    raw_dir            解码后的 raw_data 目录（必填）
    -o, --output DIR   输出目录；默认 <raw_dir>/../plot_raw_data/plot_compare_gloc_gnss

输入:
    <raw_dir>/gnsspos.txt
    <raw_dir>/gnssvel.txt
    <raw_dir>/heading.txt
    <raw_dir>/gloc.txt

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
    parser = argparse.ArgumentParser(description='对比 Gloc 与 GNSS 解码结果')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument('-o', '--output', help='输出目录（默认 <raw_dir>/../plot_raw_data/plot_compare_gloc_gnss）')
    args = parser.parse_args()

    try:
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
        gloc_path = raw_dir / 'gloc.txt'
        if not gloc_path.is_file():
            raise FileNotFoundError(f'文件不存在: {gloc_path} (gloc)')
        gnsspos_rows, gnssvel_rows, heading_rows = plot_common.require_gnss_compare_files(raw_dir)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'错误: {exc}')
        return 1

    gloc_rows = plot_common.load_gloc_compare_rows(gloc_path)
    if not gnsspos_rows or not gnssvel_rows or not heading_rows or not gloc_rows:
        print('错误: 输入文件无有效数据')
        return 1

    print(f'读取 GNSS POS:   {len(gnsspos_rows)} 条 (pos_type=50)')
    print(f'读取 GNSS VEL:   {len(gnssvel_rows)} 条')
    print(f'读取 GNSS HDG:   {len(heading_rows)} 条 (pos_type=50)')
    print(f'读取 Gloc:       {len(gloc_rows)} 条 ({gloc_path.name})')

    t0 = min(
        gnsspos_rows[0]['time'],
        gnssvel_rows[0]['time'],
        heading_rows[0]['time'],
        gloc_rows[0]['time'],
    )

    output_dir = plot_common.resolve_output_dir(
        raw_dir, args.output, plot_common.COMPARE_GLOC_GNSS_DIR)
    plot_common.save_ins_gnss_comparisons(
        output_dir, gnsspos_rows, gnssvel_rows, heading_rows, gloc_rows, t0,
        'Gloc')
    return 0


if __name__ == '__main__':
    sys.exit(main())
