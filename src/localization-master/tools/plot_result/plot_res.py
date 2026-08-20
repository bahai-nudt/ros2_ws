#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制解算 log.txt 中的量测残差时间序列。

作用:
    解析块状 [EVENT] 日志的 GNSS_POS / GNSS_VEL / HEADING / LIDAR，
    画验前 residual_pre 与验后 residual_post。

用法:
    python3 tools/plot_result/plot_res.py <output_dir>
    python3 tools/plot_result/plot_res.py <log.txt>
    python3 tools/plot_result/plot_res.py <output_dir> -o /path/to/out_dir

参数:
    path               解算输出目录或 log.txt
    -o, --output DIR   输出目录；默认 <output_dir>/plot_result/plot_res

输入:
    <output_dir>/log.txt

输出:
    residual_pos.png
    residual_vel.png
    residual_heading.png
    residual_lidar.png     无 LIDAR 事件则跳过
"""

import argparse
import math
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import plot_common

EVENT_RE = re.compile(
    r'^\[EVENT \d+\] type=(\w+) event_ts=([\d.]+) sensor_ts=([\d.]+)'
)
VEC3_IN_BRACKETS_RE = re.compile(r'\[([^\]]+)\]')
RES_RAD_RE = re.compile(r'res_rad=([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)')
SUMMARY_MEAN_RE = re.compile(r'res_abs_mean=([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)')

MEAS_TYPES = ('GNSS_POS', 'GNSS_VEL', 'HEADING', 'LIDAR')
COLORS = ('#E74C3C', '#2980B9', '#27AE60')


def parse_vec3(text):
    return [float(item.strip()) for item in text.split(',')]


def parse_residual_line(line):
    if 'invalid' in line:
        return None
    if 'res_m=' in line:
        match = VEC3_IN_BRACKETS_RE.search(line.split('res_m=', 1)[1])
        if match:
            return parse_vec3(match.group(1))
    if 'res_mps=' in line:
        match = VEC3_IN_BRACKETS_RE.search(line.split('res_mps=', 1)[1])
        if match:
            return parse_vec3(match.group(1))
    if 'res_rad=' in line:
        match = RES_RAD_RE.search(line)
        if match:
            return float(match.group(1))
    if 'res=' in line:
        match = VEC3_IN_BRACKETS_RE.search(line.split('res=', 1)[1])
        if match:
            return parse_vec3(match.group(1))
    return None


def load_block_events(log_path):
    events = []
    current = None
    with open(log_path, 'r', encoding='utf-8') as file:
        for raw_line in file:
            line = raw_line.rstrip('\n')
            event_match = EVENT_RE.match(line)
            if event_match:
                if current is not None:
                    events.append(current)
                current = {
                    'type': event_match.group(1),
                    'timestamp': float(event_match.group(2)),
                    'fused': 0,
                    'pre': None,
                    'post': None,
                    'lidar_abs_mean': math.nan,
                }
                continue
            if current is None:
                continue
            if line.startswith('  update: fused='):
                current['fused'] = int(line.split('=', 1)[1].strip())
            elif line.startswith('  residual_pre:'):
                current['pre'] = parse_residual_line(line)
            elif line.startswith('  residual_post:'):
                current['post'] = parse_residual_line(line)
            elif line.startswith('  residual_summary:') and current['type'] == 'LIDAR':
                match = SUMMARY_MEAN_RE.search(line)
                if match:
                    current['lidar_abs_mean'] = float(match.group(1))
        if current is not None:
            events.append(current)
    return events


def meas_events(events, event_type, fused_only=True):
    rows = [event for event in events if event['type'] == event_type]
    if fused_only:
        rows = [event for event in rows if event['fused'] == 1]
    return [event for event in rows if event['pre'] is not None or event['post'] is not None]


def scale_value(value, scale):
    if value is None:
        return None
    if isinstance(value, list):
        return [item * scale for item in value]
    return value * scale


def extract_component(values, idx):
    out = []
    for value in values:
        if value is None:
            out.append(math.nan)
        elif isinstance(value, list):
            out.append(value[idx] if len(value) > idx else math.nan)
        else:
            out.append(value if idx == 0 else math.nan)
    return out


def plot_component(ax, t_rel, pre_series, post_series, ylabel, title):
    pre = np.array(pre_series, dtype=float)
    post = np.array(post_series, dtype=float)
    if np.isfinite(pre).any():
        ax.plot(t_rel, pre, color=COLORS[1], linewidth=0.8, alpha=0.8, linestyle='-', label='pre')
    if np.isfinite(post).any():
        ax.plot(t_rel, post, color=COLORS[0], linewidth=0.8, alpha=0.8, linestyle='--', label='post')
    ax.axhline(0.0, color='#7F8C8D', linewidth=0.8, linestyle=':')
    ax.set_ylabel(ylabel, fontsize=11)
    ax.set_title(title, fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.5)
    if ax.lines:
        ax.legend(fontsize=9, loc='best')


def plot_group(rows, labels, unit, output_path, t0, value_scale=1.0):
    if not rows:
        print(f'警告: 无可绘制的残差事件，跳过 {output_path.name}')
        return False

    t_rel = [row['timestamp'] - t0 for row in rows]
    pre_vals = [scale_value(row['pre'], value_scale) for row in rows]
    post_vals = [scale_value(row['post'], value_scale) for row in rows]

    fig, axes = plt.subplots(len(labels), 1, figsize=(14, 3.0 * len(labels)), sharex=True)
    if len(labels) == 1:
        axes = [axes]

    fig.suptitle(f'{rows[0]["type"]} residual vs time ({unit})', fontsize=14)
    for ax, label, idx in zip(axes, labels, range(len(labels))):
        plot_component(
            ax, t_rel,
            extract_component(pre_vals, idx),
            extract_component(post_vals, idx),
            f'{label} ({unit})', label,
        )
    axes[-1].set_xlabel('Time (s, relative to first meas event)')
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return True


def plot_lidar(rows, output_path, t0):
    if not rows:
        print('警告: 无 LIDAR 残差事件，跳过 residual_lidar.png')
        return False

    t_rel = [row['timestamp'] - t0 for row in rows]
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('LIDAR residual vs time (m)', fontsize=14)
    for idx, label in enumerate(('sample_0', 'sample_1', 'sample_2')):
        series = [
            row['pre'][idx] if row['pre'] is not None and len(row['pre']) > idx else math.nan
            for row in rows
        ]
        axes[idx].plot(t_rel, series, color=COLORS[1], linewidth=0.7, alpha=0.85, label='pre')
        axes[idx].axhline(0.0, color='#7F8C8D', linewidth=0.8, linestyle=':')
        axes[idx].set_ylabel(f'{label} (m)')
        axes[idx].grid(True, linestyle='--', alpha=0.5)
        axes[idx].legend(fontsize=9, loc='best')

    axes[3].plot(t_rel, [row['lidar_abs_mean'] for row in rows],
                 color='#E67E22', linewidth=0.8, alpha=0.9, label='abs_mean')
    axes[3].axhline(0.0, color='#7F8C8D', linewidth=0.8, linestyle=':')
    axes[3].set_ylabel('abs_mean (m)')
    axes[3].set_xlabel('Time (s, relative to first meas event)')
    axes[3].grid(True, linestyle='--', alpha=0.5)
    axes[3].legend(fontsize=9, loc='best')
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return True


def resolve_log_and_output(path_arg, output_arg):
    path = Path(path_arg).resolve()
    if path.is_file():
        log_path = path
        default_root = path.parent
    elif path.is_dir():
        log_path = path / 'log.txt'
        default_root = path
    else:
        raise FileNotFoundError(f'找不到路径: {path}')
    if not log_path.is_file():
        raise FileNotFoundError(f'找不到 log.txt: {log_path}')
    output_dir = plot_common.resolve_output_dir(default_root, output_arg, 'plot_res')
    return log_path, output_dir


def main():
    parser = argparse.ArgumentParser(description='绘制 log.txt 量测残差')
    parser.add_argument('path', help='解算输出目录或 log.txt')
    parser.add_argument('-o', '--output', help='输出目录（默认 <output_dir>/plot_result/plot_res）')
    args = parser.parse_args()

    try:
        log_path, output_dir = resolve_log_and_output(args.path, args.output)
    except FileNotFoundError as exc:
        print(f'错误: {exc}')
        return 1

    events = load_block_events(log_path)
    meas = [event for event in events if event['type'] in MEAS_TYPES]
    if not meas:
        print(f'错误: {log_path} 中未找到量测事件块')
        return 1

    t0 = min(event['timestamp'] for event in meas)
    print(f'日志: {log_path}')
    print(f'输出目录: {output_dir}')
    for event_type in MEAS_TYPES:
        rows = meas_events(events, event_type)
        pre_count = sum(1 for event in rows if event['pre'] is not None)
        post_count = sum(1 for event in rows if event['post'] is not None)
        print(f'  {event_type}: fused={len(rows)}, pre={pre_count}, post={post_count}')

    saved = []
    jobs = [
        (meas_events(events, 'GNSS_POS'), ('North', 'East', 'Up'), 'm',
         'residual_pos.png', 1.0),
        (meas_events(events, 'GNSS_VEL'), ('East', 'North', 'Up'), 'm/s',
         'residual_vel.png', 1.0),
        (meas_events(events, 'HEADING'), ('Heading',), 'deg',
         'residual_heading.png', 180.0 / math.pi),
    ]
    for rows, labels, unit, filename, scale in jobs:
        output_path = output_dir / filename
        if plot_group(rows, labels, unit, output_path, t0, value_scale=scale):
            saved.append(output_path)
            print(f'已保存: {output_path}')

    lidar_path = output_dir / 'residual_lidar.png'
    if plot_lidar(meas_events(events, 'LIDAR'), lidar_path, t0):
        saved.append(lidar_path)
        print(f'已保存: {lidar_path}')

    return 0 if saved else 1


if __name__ == '__main__':
    sys.exit(main())
