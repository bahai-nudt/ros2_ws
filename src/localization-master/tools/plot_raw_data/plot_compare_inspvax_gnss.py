#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比 INSPVAX 与 GNSS 的位置 / 速度 / 航向。

作用:
    用 {brand}.ins.txt 当组合惯导参考，叠画 gnsspos / gnssvel / heading。
    GNSS 位置和航向只保留 pos_type=50。一组 raw_data 只读一路 *.ins.txt。
    传入 --config 时开启杆臂补偿，并补偿 GNSS 航向后再与 INSPVAX 对比：
    heading_plot = raw - heading_offset_deg - 90。
    heading_offset_deg=0 表示左主右从（基线朝右），180 表示主从对调；
    无论哪种都再减 90°，把双天线基线航向转到车头，才能和 INSPVAX azimuth 对齐。
    杆臂：用 imu_topic 从 calib.json 取 GNSS 主天线杆臂，把 INSPVAX 位置/速度转到天线处。
    坐标系转换 / 杆臂 / 航向与 plot_compare_sins_gnss 共用 plot_common。

用法:
    python3 tools/plot_raw_data/plot_compare_inspvax_gnss.py <raw_dir>
    python3 tools/plot_raw_data/plot_compare_inspvax_gnss.py <raw_dir> --config config/parameter_yankuang.yaml
    python3 tools/plot_raw_data/plot_compare_inspvax_gnss.py <raw_dir> --compensate-heading-bias
    python3 tools/plot_raw_data/plot_compare_inspvax_gnss.py <raw_dir> -o /path/to/out_dir

参数:
    raw_dir                      解码后的 raw_data 目录（必填）
    --config YAML                滤波配置；传入则开启杆臂补偿，并按 heading_offset_deg 与双天线基线 90° 补偿航向
    --compensate-heading-bias    用前几秒估计 INSPVAX−GNSS 航向常值偏差并补到 INS
    -o, --output DIR             输出目录；默认 <raw_dir>/../plot_raw_data/plot_compare_inspvax_gnss

输入:
    <raw_dir>/gnsspos.txt
    <raw_dir>/gnssvel.txt
    <raw_dir>/heading.txt
    <raw_dir>/{brand}.ins.txt
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
    parser = argparse.ArgumentParser(description='对比 INSPVAX 与 GNSS 解码结果')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument(
        '--config',
        metavar='YAML',
        help='滤波配置 yaml；传入则开启杆臂补偿，并按 heading_offset_deg 与双天线基线 90° 补偿 GNSS 航向',
    )
    parser.add_argument(
        '--compensate-heading-bias',
        action='store_true',
        help='估计 INSPVAX-GNSS 航向系统性偏差并补偿到 INSPVAX（双天线安装方向差异）',
    )
    parser.add_argument('-o', '--output', help='输出目录（默认 <raw_dir>/../plot_raw_data/plot_compare_inspvax_gnss）')
    args = parser.parse_args()

    try:
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
        ins_path = plot_common.resolve_ins_path(raw_dir)
        gnsspos_rows, gnssvel_rows, heading_rows = plot_common.require_gnss_compare_files(raw_dir)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'错误: {exc}')
        return 1

    inspvax_rows = plot_common.load_ins_compare_rows(ins_path)
    if not gnsspos_rows or not gnssvel_rows or not heading_rows or not inspvax_rows:
        print('错误: 输入文件无有效数据')
        return 1

    print(f'读取 GNSS POS:   {len(gnsspos_rows)} 条 (pos_type=50)')
    print(f'读取 GNSS VEL:   {len(gnssvel_rows)} 条')
    print(f'读取 GNSS HDG:   {len(heading_rows)} 条 (pos_type=50)')
    print(f'读取 INSPVAX:    {len(inspvax_rows)} 条 ({ins_path.name})')

    inspvax_rows, heading_rows, lever_applied, heading_offset, rc = (
        plot_common.apply_compare_config(
            raw_dir, args.config, inspvax_rows, heading_rows, ins_name='INSPVAX'))
    if rc:
        return rc
    ins_label = 'INSPVAX (antenna)' if lever_applied else 'INSPVAX'

    t0 = min(
        gnsspos_rows[0]['time'],
        gnssvel_rows[0]['time'],
        heading_rows[0]['time'],
        inspvax_rows[0]['time'],
    )

    heading_bias = None
    if args.compensate_heading_bias:
        heading_bias = plot_common.estimate_heading_bias(
            heading_rows, inspvax_rows, t0, ins_name='INSPVAX')

    output_dir = plot_common.resolve_output_dir(
        raw_dir, args.output, plot_common.COMPARE_INS_GNSS_DIR)
    plot_common.save_ins_gnss_comparisons(
        output_dir, gnsspos_rows, gnssvel_rows, heading_rows, inspvax_rows, t0,
        ins_label, heading_bias, heading_offset)
    return 0


if __name__ == '__main__':
    sys.exit(main())
