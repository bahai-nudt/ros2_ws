#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制 GNSS 位置 / 速度 / 航向时间序列。

作用:
    读取 raw_data 下自研 CSV（gnsspos / gnssvel / heading），按字段分组出图。
    缺哪个文件就跳过哪组。横轴为 timestamp 相对首帧的秒。

用法:
    python3 tools/plot_raw_data/plot_gnss.py <raw_dir>
    python3 tools/plot_raw_data/plot_gnss.py <raw_dir> -o /path/to/out_dir

参数:
    raw_dir            解码后的 raw_data 目录（必填）
    -o, --output DIR   输出目录；默认 <raw_dir>/../plot_raw_data/plot_gnss

输入:
    <raw_dir>/gnsspos.txt
    <raw_dir>/gnssvel.txt
    <raw_dir>/heading.txt

输出:
    gnsspos_timeseries_{status,lla,std}.png
    gnssvel_timeseries_{status,speed}.png
    heading_timeseries_{status,heading}.png
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import plot_common

TIME_FIELD = 'timestamp'
INT_FIELDS = frozenset({'sol_status', 'pos_type', 'vel_type'})

DATASETS = [
    ('gnsspos', 'gnsspos.txt', [
        ('status', 'GNSS POS Status', [
            ('pos_type', 'pos_type'),
        ]),
        ('lla', 'GNSS POS LLA', [
            ('lat', 'lat (deg)'),
            ('lon', 'lon (deg)'),
            ('hgt', 'hgt (m)'),
        ]),
        ('std', 'GNSS POS Std', [
            ('lat_std', 'lat_std (m)'),
            ('lon_std', 'lon_std (m)'),
            ('hgt_std', 'hgt_std (m)'),
        ]),
    ]),
    ('gnssvel', 'gnssvel.txt', [
        ('status', 'GNSS VEL Status', [
            ('vel_type', 'vel_type'),
        ]),
        ('speed', 'GNSS VEL Speed', [
            ('hor_speed', 'hor_speed (m/s)'),
            ('trk_gnd', 'trk_gnd (deg)'),
            ('ver_speed', 'ver_speed (m/s)'),
        ]),
    ]),
    ('heading', 'heading.txt', [
        ('status', 'Heading Status', [
            ('pos_type', 'pos_type'),
            ('length', 'length (m)'),
        ]),
        ('heading', 'Heading', [
            ('heading', 'heading (deg)'),
            ('heading_std', 'heading_std (deg)'),
        ]),
    ]),
]


def load_rows_from_csv(csv_path):
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        field_names = reader.fieldnames or []
        for row in reader:
            parsed = {}
            for field in field_names:
                if field in INT_FIELDS:
                    parsed[field] = int(float(row[field]))
                else:
                    parsed[field] = float(row[field])
            rows.append(parsed)
    rows.sort(key=lambda item: item[TIME_FIELD])
    return rows


def plot_group(rows, fields, output_path, group_title, source_title):
    t = np.array([row[TIME_FIELD] for row in rows], dtype=float)
    t_rel = t - t[0]
    n_fields = len(fields)
    fig, axes = plt.subplots(n_fields, 1, figsize=(14, 2.5 * n_fields), sharex=True)
    if n_fields == 1:
        axes = [axes]

    fig.suptitle(f'{group_title} — {source_title}', fontsize=14)
    for ax, (field, label) in zip(axes, fields):
        values = np.array([row[field] for row in rows], dtype=float)
        ax.plot(t_rel, values, linewidth=0.6, color='#1f77b4')
        ax.set_ylabel(label, fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('timestamp (s, relative to first sample)')
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_dataset(rows, source_name, groups, output_dir, source_title):
    if not rows:
        raise RuntimeError(f'无有效 {source_name} 数据')

    rows.sort(key=lambda item: item[TIME_FIELD])
    saved = []
    for suffix, group_title, fields in groups:
        output_path = output_dir / f'{source_name}_timeseries_{suffix}.png'
        plot_group(rows, fields, output_path, group_title, source_title)
        saved.append(output_path)
    return saved


def main():
    parser = argparse.ArgumentParser(description='绘制 GNSS 各字段时间序列（分组 PNG）')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument('-o', '--output', help='输出目录（默认 <raw_dir>/../plot_raw_data/plot_gnss）')
    args = parser.parse_args()

    try:
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
    except FileNotFoundError as exc:
        print(f'错误: {exc}')
        return 1

    output_dir = plot_common.resolve_output_dir(raw_dir, args.output, 'plot_gnss')

    saved = []
    for source_name, filename, groups in DATASETS:
        csv_path = raw_dir / filename
        if not csv_path.is_file():
            print(f'跳过: 文件不存在 {csv_path}')
            continue
        rows = load_rows_from_csv(csv_path)
        print(f'读取: {csv_path} ({len(rows)} 条)')
        saved.extend(plot_dataset(rows, source_name, groups, output_dir, csv_path.name))

    if not saved:
        print('错误: 未找到可绘制的 GNSS 文件')
        return 1

    for path in saved:
        print(f'已保存: {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
