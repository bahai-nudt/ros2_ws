#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制 IMU 角速度 / 线加速度时间序列。

作用:
    读取 raw_data 下的 {brand}.imu.txt（BADP 11 列空格格式），每个品牌一张图。
    左列陀螺 XYZ，右列加计 XYZ；横轴为相对首帧的秒。

用法:
    python3 tools/plot_raw_data/plot_imu.py <raw_dir>
    python3 tools/plot_raw_data/plot_imu.py <raw_dir> --imu shangyu
    python3 tools/plot_raw_data/plot_imu.py <raw_dir> -o /path/to/out_dir

参数:
    raw_dir            解码后的 raw_data 目录（必填）
    --imu <brand>      只画指定品牌，如 shangyu / bynav / bewis；默认画目录内全部 *.imu.txt
    -o, --output DIR   输出目录；默认 <raw_dir>/../plot_raw_data/plot_imu

输入:
    <raw_dir>/{brand}.imu.txt

输出:
    <output>/imu_<brand>_timeseries.png
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import plot_common

LEFT_SERIES = (
    ('gyro_x', 'Gyro X (rad/s)'),
    ('gyro_y', 'Gyro Y (rad/s)'),
    ('gyro_z', 'Gyro Z (rad/s)'),
)
RIGHT_SERIES = (
    ('accel_x', 'Accel X (m/s²)'),
    ('accel_y', 'Accel Y (m/s²)'),
    ('accel_z', 'Accel Z (m/s²)'),
)


def plot_imu_timeseries(samples, output_path, title):
    t0 = samples[0]['t']
    t = np.array([s['t'] - t0 for s in samples], dtype=float)
    data = {field: np.array([s[field] for s in samples], dtype=float)
            for field in plot_common.IMU_FIELDS}

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle(title, fontsize=14)

    for row, (field, ylabel) in enumerate(LEFT_SERIES):
        ax = axes[row, 0]
        ax.plot(t, data[field], linewidth=0.6, color='#1f77b4')
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)

    for row, (field, ylabel) in enumerate(RIGHT_SERIES):
        ax = axes[row, 1]
        ax.plot(t, data[field], linewidth=0.6, color='#ff7f0e')
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)

    axes[2, 0].set_xlabel('Time (s, relative to first sample)')
    axes[2, 1].set_xlabel('Time (s, relative to first sample)')
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='绘制 IMU 时序（左陀螺 / 右加计；默认全部品牌）')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument('--imu', help='只画指定品牌，如 shangyu（默认画目录内全部 *.imu.txt）')
    parser.add_argument('-o', '--output', help='输出目录（默认 <raw_dir>/../plot_raw_data/plot_imu）')
    args = parser.parse_args()

    try:
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
        imu_paths = plot_common.resolve_imu_paths(raw_dir, args.imu)
        output_dir = plot_common.resolve_output_dir(raw_dir, args.output, 'plot_imu')
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'错误: {exc}')
        return 1

    saved = 0
    for imu_path in imu_paths:
        samples = plot_common.load_imu_txt(imu_path)
        if not samples:
            print(f'错误: 无有效 IMU 数据: {imu_path}')
            return 1

        brand = plot_common.imu_brand_from_path(imu_path)
        output_path = output_dir / f'imu_{brand}_timeseries.png'

        print(f'读取: {imu_path}（{len(samples)} 条）')
        plot_imu_timeseries(samples, output_path, title=f'IMU Timeseries: {brand}')
        print(f'已保存: {output_path}')
        saved += 1

    return 0 if saved else 1


if __name__ == '__main__':
    sys.exit(main())
