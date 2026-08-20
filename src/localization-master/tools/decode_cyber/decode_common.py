#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""decode_data 公共工具（Record 读取、并发解析、GPS 周秒转换）。"""

from concurrent.futures import ThreadPoolExecutor, as_completed

GPS_EPOCH_OFFSET = 315964800.0
LEAP_SECONDS = 18.0


def gps_week_to_unix(gps_week, gps_ms):
    return gps_week * 604800.0 + gps_ms / 1000.0 + GPS_EPOCH_OFFSET - LEAP_SECONDS


def create_record_reader(record_path):
    from cyber.python.cyber_py3 import record
    path = str(record_path)
    try:
        return record.RecordReader(path)
    except SystemError:
        return record.RecordReader(path.encode('utf-8'))


def collect_channel_messages(record_path, topic):
    records = []
    reader = create_record_reader(record_path)
    for channel_name, msg_content, _, _ in reader.read_messages():
        if channel_name != topic or msg_content is None:
            continue
        records.append((len(records) + 1, msg_content))
    return records


def parse_records(records, parse_func, msg_cls, max_workers):
    parsed = []
    parse_errors = 0

    if max_workers <= 1 or len(records) <= 1:
        for seq, msg_content in records:
            try:
                parsed.append((seq, parse_func(msg_content, msg_cls)))
            except Exception:
                parse_errors += 1
        return [line for _, line in parsed], parse_errors

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_seq = {
            executor.submit(parse_func, msg_content, msg_cls): seq
            for seq, msg_content in records
        }
        for future in as_completed(future_to_seq):
            seq = future_to_seq[future]
            try:
                parsed.append((seq, future.result()))
            except Exception:
                parse_errors += 1

    parsed.sort(key=lambda item: item[0])
    return [line for _, line in parsed], parse_errors


def print_decode_summary(name, count, parse_errors, output_file):
    if parse_errors > 0:
        print(f'  {name}: {count} 条，解析失败 {parse_errors} 条，已输出 {output_file}')
    else:
        print(f'  {name}: {count} 条，已输出 {output_file}')
