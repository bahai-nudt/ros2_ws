#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比 raw_data 中两路 IMU 的角速度 / 线加速度。

作用:
    需要目录内至少两个 {brand}.imu.txt（如 shangyu + bewis）。
    默认参考路为非 bewis（RFU），bewis 视为 FLU。
    出两张图：原始坐标系叠画；以及把 bewis 按 R_bv 转到 RFU 后再比。
    FLU→RFU：x_rfu = -y_flu, y_rfu = x_flu, z_rfu = z_flu。

用法:
    python3 tools/plot_raw_data/plot_compare_imu.py <raw_dir>
    python3 tools/plot_raw_data/plot_compare_imu.py <raw_dir> \\
        --imu-a shangyu.imu.txt --imu-b bewis.imu.txt
    python3 tools/plot_raw_data/plot_compare_imu.py <raw_dir> -o /path/to/out_dir

参数:
    raw_dir            解码后的 raw_data 目录（必填）
    --imu-a FILE       参考 IMU 文件名；默认自动选非 bewis 的 *.imu.txt
    --imu-b FILE       对比 IMU 文件名；默认自动选 bewis.imu.txt 或第二个文件
    -o, --output DIR   输出目录；默认 <raw_dir>/../plot_raw_data/plot_compare_imu

输入:
    <raw_dir>/{brand}.imu.txt（至少两路）

输出:
    imu_comparison_raw.png    原始坐标系
    imu_comparison_rfu.png    bewis FLU→RFU 后对比
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import plot_common

GYRO_FIELDS = ('gyro_x', 'gyro_y', 'gyro_z')
ACCEL_FIELDS = ('accel_x', 'accel_y', 'accel_z')
GYRO_LABELS = ('Gyro X', 'Gyro Y', 'Gyro Z')
ACCEL_LABELS = ('Accel X', 'Accel Y', 'Accel Z')
IMU_FIELDS = plot_common.IMU_FIELDS


def is_bewis_brand(brand):
    return brand == 'bewis'


def discover_imu_files(raw_dir, imu_a=None, imu_b=None):
    if imu_a and imu_b:
        path_a = raw_dir / imu_a
        path_b = raw_dir / imu_b
        if not path_a.is_file():
            raise FileNotFoundError(f'IMU 文件不存在: {path_a}')
        if not path_b.is_file():
            raise FileNotFoundError(f'IMU 文件不存在: {path_b}')
        return path_a, path_b

    imu_files = plot_common.list_imu_files(raw_dir)
    if len(imu_files) < 2:
        raise FileNotFoundError(
            f'目录中需至少 2 个 *.imu.txt，当前找到 {len(imu_files)} 个: {raw_dir}'
        )
    if len(imu_files) == 2:
        return imu_files[0], imu_files[1]

    bewis_files = [path for path in imu_files
                   if is_bewis_brand(plot_common.imu_brand_from_path(path))]
    ref_files = [path for path in imu_files
                 if not is_bewis_brand(plot_common.imu_brand_from_path(path))]
    if len(bewis_files) == 1 and len(ref_files) >= 1:
        return ref_files[0], bewis_files[0]

    raise RuntimeError(
        '目录中存在多个 *.imu.txt，请用 --imu-a / --imu-b 指定要对比的两个文件:\n'
        + '\n'.join(f'  - {path.name}' for path in imu_files)
    )


def flu_to_rfu_sample(sample):
    """bewis FLU -> 参考 IMU RFU，与 data_reader.cpp / 矿区 parameter_*.yaml R_bv 一致。"""
    return {
        't': sample['t'],
        'gyro_x': -sample['gyro_y'],
        'gyro_y': sample['gyro_x'],
        'gyro_z': sample['gyro_z'],
        'accel_x': -sample['accel_y'],
        'accel_y': sample['accel_x'],
        'accel_z': sample['accel_z'],
    }


def transform_samples_flu_to_rfu(samples):
    return [flu_to_rfu_sample(sample) for sample in samples]


def samples_to_arrays(samples):
    if not samples:
        return None
    t0 = samples[0]['t']
    t = np.array([s['t'] - t0 for s in samples], dtype=float)
    data = {field: np.array([s[field] for s in samples], dtype=float)
            for field in IMU_FIELDS}
    return t, data


def plot_imu_comparison(ref_samples, other_samples, output_path, title,
                        ref_label='ref', other_label='other'):
    ref = samples_to_arrays(ref_samples)
    other = samples_to_arrays(other_samples)
    if ref is None and other is None:
        raise RuntimeError('两个 IMU 文件均无有效数据')

    fig, axes = plt.subplots(6, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(title, fontsize=14)

    series = [
        (GYRO_FIELDS[0], GYRO_LABELS[0], 'rad/s'),
        (GYRO_FIELDS[1], GYRO_LABELS[1], 'rad/s'),
        (GYRO_FIELDS[2], GYRO_LABELS[2], 'rad/s'),
        (ACCEL_FIELDS[0], ACCEL_LABELS[0], 'm/s²'),
        (ACCEL_FIELDS[1], ACCEL_LABELS[1], 'm/s²'),
        (ACCEL_FIELDS[2], ACCEL_LABELS[2], 'm/s²'),
    ]

    for ax, (field, axis_title, unit) in zip(axes, series):
        if ref is not None:
            t_ref, data_ref = ref
            ax.plot(t_ref, data_ref[field], label=ref_label, linewidth=0.6, alpha=0.85)
        if other is not None:
            t_other, data_other = other
            ax.plot(t_other, data_other[field], label=other_label,
                    linewidth=0.6, alpha=0.85)
        ax.set_ylabel(f'{axis_title}\n({unit})')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)

    axes[-1].set_xlabel('Time (s, relative to first sample)')
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='对比 raw_data 中双 IMU txt')
    parser.add_argument('raw_dir', help='解码后的 raw_data 目录')
    parser.add_argument('--imu-a', help='参考 IMU 文件名（默认自动选择非 bewis 的 *.imu.txt）')
    parser.add_argument('--imu-b', help='对比 IMU 文件名（默认自动选择 bewis.imu.txt 或第二个文件）')
    parser.add_argument('-o', '--output', help='输出目录（默认 <raw_dir>/../plot_raw_data/plot_compare_imu）')
    args = parser.parse_args()

    try:
        raw_dir = plot_common.require_raw_dir(args.raw_dir)
        path_a, path_b = discover_imu_files(raw_dir, args.imu_a, args.imu_b)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'错误: {exc}')
        return 1

    brand_a = plot_common.imu_brand_from_path(path_a)
    brand_b = plot_common.imu_brand_from_path(path_b)

    print(f'raw_data: {raw_dir}')
    print(f'读取 {path_a.name} ...')
    samples_a = plot_common.load_imu_txt(path_a)
    print(f'  {len(samples_a)} 条')

    print(f'读取 {path_b.name} ...')
    samples_b = plot_common.load_imu_txt(path_b)
    print(f'  {len(samples_b)} 条')

    if not samples_a and not samples_b:
        print('错误: 未读取到任何 IMU 数据')
        return 1

    output_dir = plot_common.resolve_output_dir(raw_dir, args.output, 'plot_compare_imu')
    raw_output = output_dir / 'imu_comparison_raw.png'
    rfu_output = output_dir / 'imu_comparison_rfu.png'

    if is_bewis_brand(brand_a) and not is_bewis_brand(brand_b):
        ref_samples, other_samples = samples_b, samples_a
        ref_brand, other_brand = brand_b, brand_a
    else:
        ref_samples, other_samples = samples_a, samples_b
        ref_brand, other_brand = brand_a, brand_b

    other_is_bewis = is_bewis_brand(other_brand)
    other_raw_label = f'{other_brand} (FLU)' if other_is_bewis else other_brand

    plot_imu_comparison(
        ref_samples, other_samples, raw_output,
        title=f'IMU Comparison (raw): {ref_brand} RFU vs {other_brand}'
              + (' FLU' if other_is_bewis else ''),
        ref_label=f'{ref_brand} (RFU)',
        other_label=other_raw_label,
    )
    print(f'已保存: {raw_output}')

    other_rfu_samples = (transform_samples_flu_to_rfu(other_samples)
                         if other_is_bewis else other_samples)
    other_rfu_label = f'{other_brand} (FLU->RFU)' if other_is_bewis else other_brand

    plot_imu_comparison(
        ref_samples, other_rfu_samples, rfu_output,
        title=f'IMU Comparison (RFU): {ref_brand} vs {other_brand}'
              + (' FLU->RFU' if other_is_bewis else ''),
        ref_label=f'{ref_brand} (RFU)',
        other_label=other_rfu_label,
    )
    print(f'已保存: {rfu_output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
