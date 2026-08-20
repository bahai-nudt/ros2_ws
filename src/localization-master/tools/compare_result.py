#!/usr/bin/env python3
"""逐行逐列比对两个 CSV 结果文件。"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare two MSF result CSV files")
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--tol", type=float, default=0.0,
                        help="Absolute tolerance per numeric column (0 = exact string match)")
    return parser.parse_args()


def is_number(token: str) -> bool:
    try:
        float(token)
        return True
    except ValueError:
        return False


def compare_file(baseline: Path, candidate: Path, tol: float) -> int:
    b_lines = baseline.read_text(encoding="utf-8").splitlines()
    c_lines = candidate.read_text(encoding="utf-8").splitlines()

    if len(b_lines) != len(c_lines):
        print(f"line count mismatch: baseline={len(b_lines)} candidate={len(c_lines)}")
        return 1

    # 逐列数值误差统计：max_abs / min_abs / sum_abs / sum_sq / count
    col_stats: list[dict[str, float]] = []

    for i, (b_line, c_line) in enumerate(zip(b_lines, c_lines), start=1):
        if tol > 0.0:
            b_cols = b_line.split(",")
            c_cols = c_line.split(",")
            if len(b_cols) != len(c_cols):
                print(f"line {i}: column count mismatch {len(b_cols)} vs {len(c_cols)}")
                return 1
            ok = True
            for j, (bv, cv) in enumerate(zip(b_cols, c_cols)):
                if is_number(bv) and is_number(cv):
                    diff = abs(float(bv) - float(cv))
                    while len(col_stats) <= j:
                        col_stats.append({"max_abs": 0.0, "min_abs": float("inf"),
                                          "sum_abs": 0.0, "sum_sq": 0.0, "count": 0.0})
                    stat = col_stats[j]
                    stat["max_abs"] = max(stat["max_abs"], diff)
                    stat["min_abs"] = min(stat["min_abs"], diff)
                    stat["sum_abs"] += diff
                    stat["sum_sq"] += diff * diff
                    stat["count"] += 1.0
                    if diff <= tol:
                        continue
                    ok = False
                    print(f"line {i} col {j}: baseline={bv!r} candidate={cv!r}")
                    break
                if bv == cv:
                    continue
                ok = False
                print(f"line {i} col {j}: baseline={bv!r} candidate={cv!r}")
                break
            if ok:
                continue
            return 1
        if b_line == c_line:
            continue
        print(f"line {i} mismatch:\n  baseline : {b_line}\n  candidate: {c_line}")
        return 1

    print(f"OK: {baseline.name} matches {candidate.name} ({len(b_lines)} lines)")

    if tol > 0.0 and col_stats:
        print("\nnumeric error stats (absolute):")
        overall_max = 0.0
        overall_max_col = -1
        for j, stat in enumerate(col_stats):
            if stat["count"] <= 0.0:
                continue
            mean_abs = stat["sum_abs"] / stat["count"]
            rms = (stat["sum_sq"] / stat["count"]) ** 0.5
            print(f"  col {j:2d}: max={stat['max_abs']:.6e} "
                  f"min={stat['min_abs']:.6e} mean={mean_abs:.6e} rms={rms:.6e} n={int(stat['count'])}")
            if stat["max_abs"] > overall_max:
                overall_max = stat["max_abs"]
                overall_max_col = j
        print(f"  overall max abs error: {overall_max:.6e} @ col {overall_max_col}")

    return 0


def main() -> int:
    args = parse_args()
    if not args.baseline.exists() or not args.candidate.exists():
        print("input file missing", file=sys.stderr)
        return 2
    return compare_file(args.baseline, args.candidate, args.tol)


if __name__ == "__main__":
    raise SystemExit(main())
