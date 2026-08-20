#!/usr/bin/env bash
# demo 组合层一键脚本：选择子项目 -> 配置/编译/运行。
# 只做编排，不复制构建逻辑；构建一律走 cmake。
#
# 用法：
#   ./demo/process.sh list
#   ./demo/process.sh gnss_ins [--config=<yaml>] [--output=<dir>]
#   ./demo/process.sh lio [--config=<yaml>] [--output=<dir>]
#   ./demo/process.sh gnss_lidar_ins [--config=<yaml>] [--output=<dir>]
#   ./demo/process.sh gnss_lio_ins [--config=<yaml>] [--output=<dir>]
#   ./demo/process.sh test [CTest 正则]
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
BUILD_DIR="${ROOT}/build"
JOBS=${JOBS:-$(nproc 2>/dev/null || echo 4)}
# 编译类型：默认 Release（-O0 下 Eigen 小矩阵运算会慢约 20 倍），可用 BUILD_TYPE=Debug 覆盖
BUILD_TYPE=${BUILD_TYPE:-Release}

# 配置：$1 = LOC_BUILD_DEMO_APPS, $2 = LOC_BUILD_DEMO_TESTS
configure() {
    cmake -S "$ROOT" -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DLOC_BUILD_DEMO=ON \
        -DLOC_BUILD_DEMO_APPS="$1" \
        -DLOC_BUILD_DEMO_TESTS="$2"
}

case "${1:-}" in
    list)
        echo "可用子项目："
        echo "  gnss_ins        GNSS/INS 组合导航 demo（--config=<yaml> [--output=<dir>]）"
        echo "  lio             LIO demo（--config=<yaml> [--output=<dir>]）"
        echo "  gnss_lidar_ins  GNSS/INS/LiDAR 点面 IEKF demo（--config=<yaml> [--output=<dir>]）"
        echo "  gnss_lio_ins    GNSS/LIO/INS pose_meas demo（--config=<yaml> [--output=<dir>]）"
        echo "  test            单元测试（可选参数：CTest 正则，如 zupt）"
        ;;
    gnss_ins)
        shift
        configure ON OFF
        cmake --build "$BUILD_DIR" --target gnss_ins -j"$JOBS"
        "$BUILD_DIR/bin/gnss_ins" "$@"
        ;;
    lio)
        shift
        configure ON OFF
        cmake --build "$BUILD_DIR" --target lio -j"$JOBS"
        "$BUILD_DIR/bin/lio" "$@"
        ;;
    gnss_lidar_ins)
        shift
        configure ON OFF
        cmake --build "$BUILD_DIR" --target gnss_lidar_ins -j"$JOBS"
        "$BUILD_DIR/bin/gnss_lidar_ins" "$@"
        ;;
    gnss_lio_ins)
        shift
        configure ON OFF
        cmake --build "$BUILD_DIR" --target gnss_lio_ins -j"$JOBS"
        "$BUILD_DIR/bin/gnss_lio_ins" "$@"
        ;;
    test)
        shift
        configure OFF ON
        cmake --build "$BUILD_DIR" -j"$JOBS"
        if [ "$#" -gt 0 ]; then
            ctest --test-dir "$BUILD_DIR" -R "$1" --output-on-failure
        else
            ctest --test-dir "$BUILD_DIR" --output-on-failure
        fi
        ;;
    *)
        echo "用法: $0 <list|gnss_ins|lio|gnss_lidar_ins|gnss_lio_ins|test> [参数...]" >&2
        echo "  list                          列出可用子项目" >&2
        echo "  gnss_ins [--config=<yaml>] [--output=<dir>]  编译并运行 GNSS/INS demo" >&2
        echo "  lio [--config=<yaml>] [--output=<dir>]      编译并运行 LIO demo" >&2
        echo "  gnss_lidar_ins [--config=<yaml>] [--output=<dir>] 编译并运行 GNSS/INS/LiDAR 点面 IEKF demo" >&2
        echo "  gnss_lio_ins [--config=<yaml>] [--output=<dir>] 编译并运行 GNSS/LIO/INS pose_meas demo" >&2
        echo "  test [CTest 正则]             编译并运行单元测试" >&2
        exit 1
        ;;
esac
