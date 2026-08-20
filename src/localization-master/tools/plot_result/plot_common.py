#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plot_result 公共库（无 CLI）。

作用:
    为解算结果绘图脚本提供 output 路径解析，以及从 plot_raw_data.plot_common
    再导出 GNSS 读取 / RFU 杆臂 / 航向补偿（与 plot_compare_inspvax_gnss 同一套）。
    不能用 sys.path 直接 import plot_common：会和本文件同名冲突。
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np

RAW_PLOT_DIR = Path(__file__).resolve().parent.parent / 'plot_raw_data'
_RAW_SPEC = importlib.util.spec_from_file_location(
    'plot_raw_data_common', RAW_PLOT_DIR / 'plot_common.py')
raw_plot = importlib.util.module_from_spec(_RAW_SPEC)
_RAW_SPEC.loader.exec_module(raw_plot)

load_ins_txt = raw_plot.load_ins_txt
load_imu_txt = raw_plot.load_imu_txt
require_raw_dir = raw_plot.require_raw_dir
resolve_ins_path = raw_plot.resolve_ins_path
load_gnsspos = raw_plot.load_gnsspos
load_gnssvel = raw_plot.load_gnssvel
load_heading = raw_plot.load_heading
load_ins_compare_rows = raw_plot.load_ins_compare_rows
load_gloc_compare_rows = raw_plot.load_gloc_compare_rows
require_gnss_compare_files = raw_plot.require_gnss_compare_files
try_enable_lever_arm = raw_plot.try_enable_lever_arm
apply_heading_for_inspvax = raw_plot.apply_heading_for_inspvax
apply_compare_config = raw_plot.apply_compare_config
load_heading_offset_from_config = raw_plot.load_heading_offset_from_config
DUAL_ANTENNA_TO_VEHICLE_DEG = raw_plot.DUAL_ANTENNA_TO_VEHICLE_DEG
plot_position = raw_plot.plot_position
plot_velocity = raw_plot.plot_velocity
plot_heading = raw_plot.plot_heading
plot_attitude = raw_plot.plot_attitude
save_ins_gnss_comparisons = raw_plot.save_ins_gnss_comparisons
save_ins_pair_comparisons = raw_plot.save_ins_pair_comparisons
estimate_heading_bias = raw_plot.estimate_heading_bias
earth_radii = raw_plot.earth_radii
COMPARE_INS_GNSS_DIR = raw_plot.COMPARE_INS_GNSS_DIR
COMPARE_GLOC_GNSS_DIR = raw_plot.COMPARE_GLOC_GNSS_DIR
COMPARE_INSPVAX_DIR = raw_plot.COMPARE_INSPVAX_DIR

DEFAULT_PLOT_ROOT = 'plot_result'


def require_dir(path, what):
    directory = Path(path).resolve()
    if not directory.is_dir():
        raise FileNotFoundError(f'找不到{what}: {directory}')
    return directory


def require_file(path, what):
    file_path = Path(path).resolve()
    if not file_path.is_file():
        raise FileNotFoundError(f'找不到{what}: {file_path}')
    return file_path


def resolve_output_dir(output_dir, output_arg, default_name):
    if output_arg:
        dest = Path(output_arg).resolve()
    else:
        dest = Path(output_dir).resolve() / DEFAULT_PLOT_ROOT / default_name
    dest.mkdir(parents=True, exist_ok=True)
    return dest


def latlon_diff_m(lat_deg, lon_deg, alt_m, lat2_deg, lon2_deg, alt2_m):
    lat_rad = math.radians(lat_deg)
    rm, rn = earth_radii(lat_rad)
    dlat = math.radians(lat2_deg - lat_deg)
    dlon = math.radians(lon2_deg - lon_deg)
    de = dlon * (rn + alt_m) * math.cos(lat_rad)
    dn = dlat * (rm + alt_m)
    du = alt2_m - alt_m
    return de, dn, du


def wrap_angle_deg(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def match_nearest(ref_rows, query_rows, label=''):
    if not ref_rows or not query_rows:
        return [], {'count': 0, 'mean_dt': 0.0, 'max_dt': 0.0}

    ref_times = [row['time'] for row in ref_rows]
    pairs = []
    dts = []
    for query in query_rows:
        idx = np.searchsorted(ref_times, query['time'])
        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(ref_times):
            candidates.append(idx)
        best_i = min(candidates, key=lambda i: abs(ref_times[i] - query['time']))
        dt = abs(ref_times[best_i] - query['time'])
        pairs.append((ref_rows[best_i], query))
        dts.append(dt)
    stats = {
        'count': len(pairs),
        'mean_dt': float(np.mean(dts)),
        'max_dt': float(np.max(dts)),
    }
    if label:
        print(f'{label}: 匹配 {stats["count"]} 条, '
              f'平均时间差 {stats["mean_dt"]:.4f}s, 最大 {stats["max_dt"]:.4f}s')
    return pairs, stats


def compute_metric(values):
    arr = np.asarray(values, dtype=float)
    abs_arr = np.abs(arr)
    return {
        'bias': float(np.mean(arr)),
        'mae': float(np.mean(abs_arr)),
        'rmse': float(np.sqrt(np.mean(arr ** 2))),
        'p95': float(np.percentile(abs_arr, 95)),
        'max': float(np.max(abs_arr)),
        'count': len(arr),
    }


def print_ins_pair_metrics(ref_rows, ins_rows, ref_label='INSPVAX', ins_label='SINS'):
    """按最近邻把 ins 对齐到 ref，打印位置 / 速度 / 姿态 RMSE。"""
    pairs, _ = match_nearest(ins_rows, ref_rows, label=f'{ins_label} vs {ref_label} 匹配')
    if not pairs:
        print('警告: 无匹配样本，跳过误差统计')
        return

    de_list, dn_list, du_list = [], [], []
    dve, dvn, dvu = [], [], []
    droll, dpitch, daz = [], [], []
    for ins, ref in pairs:
        de, dn, du = latlon_diff_m(
            ref['lat'], ref['lon'], ref['hgt'],
            ins['lat'], ins['lon'], ins['hgt'])
        de_list.append(de)
        dn_list.append(dn)
        du_list.append(du)
        dve.append(ins['ve'] - ref['ve'])
        dvn.append(ins['vn'] - ref['vn'])
        dvu.append(ins['vu'] - ref['vu'])
        droll.append(wrap_angle_deg(ins['roll'] - ref['roll']))
        dpitch.append(wrap_angle_deg(ins['pitch'] - ref['pitch']))
        daz.append(wrap_angle_deg(ins['azimuth'] - ref['azimuth']))

    horiz = np.hypot(de_list, dn_list)

    def line(name, metric, unit):
        print(
            f'  {name:<12} RMSE {metric["rmse"]:.4f} {unit}  '
            f'P95 {metric["p95"]:.4f}  bias {metric["bias"]:+.4f}  N={metric["count"]}'
        )

    print(f'{ins_label} − {ref_label}（IMU 中心，最近邻）')
    line('水平', compute_metric(horiz), 'm')
    line('高程', compute_metric(du_list), 'm')
    line('Ve', compute_metric(dve), 'm/s')
    line('Vn', compute_metric(dvn), 'm/s')
    line('Vu', compute_metric(dvu), 'm/s')
    line('Roll', compute_metric(droll), 'deg')
    line('Pitch', compute_metric(dpitch), 'deg')
    line('Azimuth', compute_metric(daz), 'deg')
