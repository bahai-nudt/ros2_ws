#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制 INSPVAX（组合惯导解）位置 / 速度 / 姿态时间序列。

作用:
    读取 raw_data 下唯一的 {brand}.ins.txt（BADP 11 列）。一组 raw_data 只应有一路 INS。
    横轴为 obs_timestamp 相对首帧的秒。

用法:
    python3 tools/plot_raw_data/plot_inspvax.py <raw_dir>
    python3 tools/plot_raw_data/plot_inspvax.py <raw_dir> -o /path/to/out_dir

参数:
    raw_dir            解码后的 raw_data 目录（必填）
    -o, --output DIR   输出目录；默认 <raw_dir>/../plot_raw_data/plot_inspvax

输入:
    <raw_dir>/{brand}.ins.txt

输出:
    <brand>_ins_timeseries_lla.png
    <brand>_ins_timeseries_velocity.png
    <brand>_ins_timeseries_attitude.png
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import plot_common

TIME_FIELD = 'time'
PLOT_GROUPS = [
    ('lla', 'INSPVAX LLA', [
        ('latitude', 'lat (deg)'),
        ('longitude', 'lon (deg)'),
        ('height', 'hgt (m)'),
    ]),
    ('velocity', 'INSPVAX Velocity (m/s)', [
        ('north_vel', 'north_vel'),
        ('east_vel', 'east_vel'),
        ('up_vel', 'up_vel'),
    ]),
    ('attitude', 'INSPVAX Attitude (deg)', [
        ('roll', 'roll'),
        ('pitch', 'pitch'),
        ('azimuth', 'azimuth'),
    ]),
]


def plot_inspvax_group(rows, fields, output_path, group_title, source_title):
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

    axes[-1].set_xlabel('obs_timestamp (s, relative to first sample)')
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_inspvax_timeseries(rows, output_prefix, source_title):
    if not rows:
        raise RuntimeError('无有效 INSPVAX 数据')

    rows.sort(key=lambda item: item[TIME_FIELD])
    saved = []
    for suffix, group_title, fields in PLOT_GROUPS:
        output_path = Path(f'{output_prefix}_{suffix}.png')
        plot_inspvax_group(rows, fields, output_path, group_title, source_title)
        saved.append(output_path)
    return saved


def main():
    parser = argparse.ArgumentParser(description='绘制 INSPVAX 位置/速度/姿态时间序列（分组 PNG）')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument('-o', '--output', help='输出目录（默认 <raw_dir>/../plot_raw_data/plot_inspvax）')
    args = parser.parse_args()

    try:
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
        txt_path = plot_common.resolve_ins_path(raw_dir)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'错误: {exc}')
        return 1

    rows = plot_common.load_ins_txt(txt_path)
    brand = plot_common.ins_brand_from_path(txt_path)
    output_dir = plot_common.resolve_output_dir(raw_dir, args.output, 'plot_inspvax')
    output_prefix = output_dir / f'{brand}_ins_timeseries'
    print(f'读取: {txt_path} ({len(rows)} 条)')

    saved = plot_inspvax_timeseries(rows, output_prefix, source_title=txt_path.name)
    for path in saved:
        print(f'已保存: {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
