#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制车端 GLOC 各字段时间序列。

作用:
    读取 raw_data/gloc.txt（本仓库 decode_gloc CSV），按时间/位姿/速度/LLA 等分组出图。
    横轴为 obs_time 相对首帧的秒。

用法:
    python3 tools/plot_raw_data/plot_gloc.py <raw_dir>
    python3 tools/plot_raw_data/plot_gloc.py <raw_dir> -o /path/to/out_dir

参数:
    raw_dir            解码后的 raw_data 目录（必填）
    -o, --output DIR   输出目录；默认 <raw_dir>/../plot_raw_data/plot_gloc

输入:
    <raw_dir>/gloc.txt

输出:
    gloc_timeseries_{time_state,position,orientation,linear_velocity,
                     linear_acceleration,angular_velocity,lla,rpy,velocity}.png
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

PLOT_GROUPS = [
    ('time_state', 'Time / State', [
        ('timestamp', 'timestamp'),
        ('obs_time', 'obs_time (s)'),
        ('state', 'state'),
    ]),
    ('position', 'Position (m)', [
        ('position_x', 'x'),
        ('position_y', 'y'),
        ('position_z', 'z'),
    ]),
    ('orientation', 'Orientation (quat)', [
        ('orientation_w', 'w'),
        ('orientation_x', 'x'),
        ('orientation_y', 'y'),
        ('orientation_z', 'z'),
    ]),
    ('linear_velocity', 'Linear Velocity (m/s)', [
        ('linear_velocity_x', 'x'),
        ('linear_velocity_y', 'y'),
        ('linear_velocity_z', 'z'),
    ]),
    ('linear_acceleration', 'Linear Acceleration (m/s²)', [
        ('linear_acceleration_x', 'x'),
        ('linear_acceleration_y', 'y'),
        ('linear_acceleration_z', 'z'),
    ]),
    ('angular_velocity', 'Angular Velocity (rad/s)', [
        ('angular_velocity_x', 'x'),
        ('angular_velocity_y', 'y'),
        ('angular_velocity_z', 'z'),
    ]),
    ('lla', 'LLA', [
        ('latitude', 'latitude (deg)'),
        ('longitude', 'longitude (deg)'),
        ('altitude', 'altitude (m)'),
    ]),
    ('rpy', 'Roll / Pitch / Yaw (rad)', [
        ('roll', 'roll (rad)'),
        ('pitch', 'pitch (rad)'),
        ('yaw', 'yaw (rad)'),
    ]),
    ('velocity', 'Velocity (m/s)', [
        ('velocity', 'velocity (m/s)'),
    ]),
]


def load_rows_from_csv(csv_path):
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        for row in reader:
            rows.append({
                field: float(row[field]) if field != 'state' else int(float(row[field]))
                for field in FIELD_NAMES
            })
    rows.sort(key=lambda item: item['obs_time'])
    return rows


def plot_gloc_group(rows, fields, output_path, group_title, source_title):
    t = np.array([row['obs_time'] for row in rows], dtype=float)
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

    axes[-1].set_xlabel('obs_time (s, relative to first sample)')
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_gloc_timeseries(rows, output_prefix, source_title):
    if not rows:
        raise RuntimeError('无有效 Gloc 数据')

    rows.sort(key=lambda item: item['obs_time'])
    saved = []
    for suffix, group_title, fields in PLOT_GROUPS:
        output_path = Path(f'{output_prefix}_{suffix}.png')
        plot_gloc_group(rows, fields, output_path, group_title, source_title)
        saved.append(output_path)
    return saved


def main():
    parser = argparse.ArgumentParser(description='绘制 Gloc 各字段时间序列（分组 PNG）')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument('-o', '--output', help='输出目录（默认 <raw_dir>/../plot_raw_data/plot_gloc）')
    args = parser.parse_args()

    try:
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
    except FileNotFoundError as exc:
        print(f'错误: {exc}')
        return 1

    csv_path = raw_dir / 'gloc.txt'
    if not csv_path.is_file():
        print(f'错误: 文件不存在: {csv_path}')
        return 1

    rows = load_rows_from_csv(csv_path)
    output_dir = plot_common.resolve_output_dir(raw_dir, args.output, 'plot_gloc')
    output_prefix = output_dir / 'gloc_timeseries'
    print(f'读取: {csv_path} ({len(rows)} 条)')

    saved = plot_gloc_timeseries(rows, output_prefix, source_title=csv_path.name)
    for path in saved:
        print(f'已保存: {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
