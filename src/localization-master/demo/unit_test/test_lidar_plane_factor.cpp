#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "lidar_plane_factor.h"
#include "lidar_types.h"
#include "local_map.h"
#include "math/constants.h"
#include "math/coordinate.h"
#include "math/pose_converter.h"
#include "pcd_preprocess.h"
#include "types/nav_state.h"

namespace {

using msf::BlockId;
using msf::IterationContext;
using msf::MeasFactor;
using msf::NominalState;
using msf::StateLayout;
using msf::earth;
using msf::lidar::LidarPlaneFactor;
using msf::lidar::LocalMap;
using msf::lidar::PointCloudXYZI;
using msf::lidar::PointType;

bool Near(double actual, double expected, double tol = 1.0e-6) {
    return std::fabs(actual - expected) <= tol;
}

PointType MakePoint(float x, float y, float z) {
    PointType p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.0f;
    return p;
}

void SetupConfig(msf::GlobalConfig& config) {
    config._local_map_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);
    config._calibration._R_bl = Eigen::Matrix3d::Identity();
    config._calibration._T_lb_m = Eigen::Vector3d::Zero();
    config._lidar._nearest_points = 5;
    config._lidar._nearest_search_max_sq_dist = 10.0;
    config._lidar._plane_fit_threshold_m = 0.1;
    config._lidar._map_voxel_size = 0.5;
    config._lidar._frame_voxel_size = 0.5;
    config._lidar._cube_len = 1000.0;
    config._lidar._det_range = 100.0;
    config._lidar._iterated_update_noise_var = 0.001;
    config._lidar._residual_score_scale = 0.9;
    config._lidar._residual_score_min = 0.9;
    config._lidar._min_effective_features = 5;
}

PointCloudXYZI::Ptr MakePlaneZ1() {
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    cloud->push_back(MakePoint(0.0f, 0.0f, 1.0f));
    cloud->push_back(MakePoint(1.0f, 0.0f, 1.0f));
    cloud->push_back(MakePoint(-1.0f, 0.0f, 1.0f));
    cloud->push_back(MakePoint(0.0f, 1.0f, 1.0f));
    cloud->push_back(MakePoint(0.0f, -1.0f, 1.0f));
    return cloud;
}

void SetupNominal(NominalState& nominal) {
    nominal._t_cur = 100.0;
    nominal._pos = Eigen::Vector3d(30.0 * msf::constants::_D2R,
                                   120.0 * msf::constants::_D2R,
                                   100.0);
    nominal._vn = Eigen::Vector3d::Zero();
    nominal._an = Eigen::Vector3d::Zero();
    nominal._wib = Eigen::Vector3d::Zero();
    nominal._Cnb = Eigen::Matrix3d::Identity();
    nominal._Cbn = nominal._Cnb.transpose();
    nominal._qnb = msf::t_quat();
}

// 有效量测块：5 个平面约束，H/z/R_diag 尺寸正确，z≈0（点恰在平面上）
bool TestBuildMeasBlock() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud = MakePlaneZ1();
    map->Build(cloud);

    NominalState nominal;
    SetupNominal(nominal);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    scan._end_time = 100.1;

    LidarPlaneFactor factor(*map, cloud, scan, config);
    earth eth;
    eth.Update(nominal._pos, nominal._vn);
    const StateLayout layout = StateLayout::FromStateDim(15);

    IterationContext ctx;
    ctx._iteration = 0;
    ctx._last_converged = false;

    const auto block = factor.BuildMeasBlock(nominal, eth, layout, ctx);
    if (!block._valid || block._H.rows() != 5 || block._z.size() != 5 ||
        block._R_diag.size() != 5) {
        return false;
    }
    for (int i = 0; i < block._R_diag.size(); ++i) {
        if (!Near(block._R_diag(i), 0.001)) {
            return false;
        }
    }
    for (int i = 0; i < block._z.size(); ++i) {
        if (!Near(block._z(i), 0.0, 1.0e-5)) {
            return false;
        }
    }

    // 第 1 行对应 p_I=(1,0,1)：有效 H_rot=(0,1,0)，H_pos=(0,0,1)（POST_MSF 符号）
    const int rot = layout.Offset(BlockId::Rotation);
    const int pos = layout.Offset(BlockId::Position);
    return Near(block._H(1, rot + 0), 0.0) &&
           Near(block._H(1, rot + 1), 1.0) &&
           Near(block._H(1, rot + 2), 0.0) &&
           Near(block._H(1, pos + 0), 0.0) &&
           Near(block._H(1, pos + 1), 0.0) &&
           Near(block._H(1, pos + 2), 1.0);
}

// 数值雅可比：对比 H 与残差对误差状态的小扰动导数
bool TestNumericJacobian() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud = MakePlaneZ1();
    map->Build(cloud);

    NominalState nominal;
    SetupNominal(nominal);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    scan._end_time = 100.1;

    LidarPlaneFactor factor(*map, cloud, scan, config);
    earth eth;
    eth.Update(nominal._pos, nominal._vn);
    const StateLayout layout = StateLayout::FromStateDim(15);

    IterationContext ctx0;
    ctx0._iteration = 0;
    ctx0._last_converged = false;
    const auto base = factor.BuildMeasBlock(nominal, eth, layout, ctx0);
    if (!base._valid || base._H.rows() != 5) {
        return false;
    }

    constexpr int kRow = 1;  // p_I=(1,0,1)
    IterationContext ctxNoRematch;
    ctxNoRematch._iteration = 1;
    ctxNoRematch._last_converged = false;

    for (int j = 0; j < 15; ++j) {
        if (j >= 3 && j < 6) {
            continue;  // 速度块不参与 LiDAR 残差
        }
        if (j == 6 || j == 7) {
            continue;  // lat/lon 的 ENU 有限差分受 GeographicLib 非线性干扰，解析 H 已单独验证为 0
        }
        if (j >= 9) {
            continue;  // 零偏/杆臂不参与 LiDAR 残差
        }
        // 旋转误差用 1e-6 rad；高度误差是米，用 1e-6 m
        const double eps = 1.0e-6;
        NominalState pert = nominal;
        if (j < 3) {
            Eigen::Vector3d rv = Eigen::Vector3d::Zero();
            rv(j) = eps;
            pert._qnb = msf::pose_converter::rv2q(rv) * pert._qnb;
            msf::t_quat::normlize(pert._qnb);
            pert._Cnb = msf::pose_converter::q2mat(pert._qnb);
            pert._Cbn = pert._Cnb.transpose();
            pert._att = msf::pose_converter::m2att(pert._Cnb);
        } else if (j >= 6) {
            Eigen::Vector3d dpos = Eigen::Vector3d::Zero();
            dpos(j - 6) = eps;
            pert._pos -= dpos;  // 误差 dpos=+eps 对应名义位置 -eps
        }

        earth eth_pert;
        eth_pert.Update(pert._pos, pert._vn);
        const auto block_pert = factor.BuildMeasBlock(pert, eth_pert, layout, ctxNoRematch);
        if (!block_pert._valid) {
            return false;
        }
        const double numerical = (block_pert._z(kRow) - base._z(kRow)) / eps;
        const double analytic = base._H(kRow, j);
        if (!Near(numerical, -analytic, 1.0e-3)) {
            std::cerr << "jac col " << j << ": numerical=" << numerical
                      << " analytic=" << analytic << std::endl;
            return false;
        }
    }
    return true;
}

// 全状态数值雅可比：旋转/零块中心差分；位置通过 ENU 扰动避免 lat/lon 非线性噪声
bool TestNumericAllColumns() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud = MakePlaneZ1();
    map->Build(cloud);

    NominalState nominal;
    SetupNominal(nominal);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    scan._end_time = 100.1;

    LidarPlaneFactor factor(*map, cloud, scan, config);
    earth eth;
    eth.Update(nominal._pos, nominal._vn);
    const StateLayout layout = StateLayout::FromStateDim(15);

    IterationContext ctx0;
    ctx0._iteration = 0;
    ctx0._last_converged = false;
    const auto base = factor.BuildMeasBlock(nominal, eth, layout, ctx0);
    if (!base._valid) {
        return false;
    }

    constexpr int kRow = 1;
    IterationContext ctxNoRematch;
    ctxNoRematch._iteration = 1;
    ctxNoRematch._last_converged = false;

    auto residual = [&](const NominalState& n) -> double {
        earth eth_tmp;
        eth_tmp.Update(n._pos, n._vn);
        const auto b = factor.BuildMeasBlock(n, eth_tmp, layout, ctxNoRematch);
        return b._valid ? b._z(kRow) : std::numeric_limits<double>::quiet_NaN();
    };

    // 1) 旋转列：中心差分
    constexpr double kRotEps = 1.0e-6;
    for (int j = 0; j < 3; ++j) {
        Eigen::Vector3d rv_p = Eigen::Vector3d::Zero();
        Eigen::Vector3d rv_m = Eigen::Vector3d::Zero();
        rv_p(j) = kRotEps;
        rv_m(j) = -kRotEps;

        NominalState n_p = nominal;
        NominalState n_m = nominal;
        n_p._qnb = msf::pose_converter::rv2q(rv_p) * n_p._qnb;
        n_m._qnb = msf::pose_converter::rv2q(rv_m) * n_m._qnb;
        for (auto* n : {&n_p, &n_m}) {
            msf::t_quat::normlize(n->_qnb);
            n->_Cnb = msf::pose_converter::q2mat(n->_qnb);
            n->_Cbn = n->_Cnb.transpose();
            n->_att = msf::pose_converter::m2att(n->_Cnb);
        }
        const double numerical = (residual(n_p) - residual(n_m)) / (2.0 * kRotEps);
        if (!Near(numerical, -base._H(kRow, j), 1.0e-3)) {
            std::cerr << "rot col " << j << ": numerical=" << numerical
                      << " analytic=" << base._H(kRow, j) << std::endl;
            return false;
        }
    }

    // 2) 零块：速度 / 零偏 / 杆臂扰动不应改变残差
    constexpr double kZeroEps = 1.0e-6;
    for (int j = 3; j < 15; ++j) {
        if (j >= 6 && j <= 8) {
            continue;
        }
        NominalState n_p = nominal;
        NominalState n_m = nominal;
        if (j >= 3 && j <= 5) {
            n_p._vn(j - 3) += kZeroEps;
            n_m._vn(j - 3) -= kZeroEps;
        } else if (j >= 9 && j <= 11) {
            n_p._eb(j - 9) += kZeroEps;
            n_m._eb(j - 9) -= kZeroEps;
        } else if (j >= 12 && j <= 14) {
            n_p._db(j - 12) += kZeroEps;
            n_m._db(j - 12) -= kZeroEps;
        }
        const double numerical = (residual(n_p) - residual(n_m)) / (2.0 * kZeroEps);
        if (!Near(numerical, -base._H(kRow, j), 1.0e-4)) {
            std::cerr << "zero col " << j << ": numerical=" << numerical
                      << " analytic=" << base._H(kRow, j) << std::endl;
            return false;
        }
    }

    // 3) 位置列：用 ENU 扰动再转回 LLA，验证残差对 ENU 位移的导数 = -n
    const Eigen::Vector3d origin_rad(30.0 * msf::constants::_D2R,
                                     120.0 * msf::constants::_D2R,
                                     100.0);
    const Eigen::Vector3d r_enu = msf::lidar::PositionEnu(nominal, config);
    const Eigen::Vector3d n_map(0.0, 0.0, -1.0);
    constexpr double kPosEps = 1.0e-5;  // [m]
    Eigen::Vector3d G_enu;
    for (int k = 0; k < 3; ++k) {
        Eigen::Vector3d dr_p = Eigen::Vector3d::Zero();
        Eigen::Vector3d dr_m = Eigen::Vector3d::Zero();
        dr_p(k) = kPosEps;
        dr_m(k) = -kPosEps;
        NominalState n_p = nominal;
        NominalState n_m = nominal;
        n_p._pos = msf::Coordinate::enu2lla(origin_rad, r_enu + dr_p);
        n_m._pos = msf::Coordinate::enu2lla(origin_rad, r_enu + dr_m);
        const double numerical = (residual(n_p) - residual(n_m)) / (2.0 * kPosEps);
        G_enu(k) = numerical;
        if (!Near(numerical, -n_map(k), 1.0e-3)) {
            std::cerr << "enu col " << k << ": numerical=" << numerical
                      << " expected=" << -n_map(k) << std::endl;
            return false;
        }
    }

    // 4) 解析位置雅可比独立验证：有效 H_pos = -nᵀ·J（POST_MSF 符号）
    const int pos = layout.Offset(BlockId::Position);
    Eigen::Matrix3d J;
    J << 0.0, 1.0 / eth._f_cbRNh, 0.0,
         1.0 / eth._f_RMh, 0.0, 0.0,
         0.0, 0.0, 1.0;
    const Eigen::Vector3d expected_h_pos = -J.transpose() * n_map;
    for (int k = 0; k < 3; ++k) {
        if (!Near(base._H(kRow, pos + k), expected_h_pos(k), 1.0e-6)) {
            std::cerr << "pos col " << k << ": analytic=" << base._H(kRow, pos + k)
                      << " expected=" << expected_h_pos(k) << std::endl;
            return false;
        }
    }
    return true;
}

// Commit 使用修正后状态插入地图，不应崩溃
bool TestCommit() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud = MakePlaneZ1();
    map->Build(cloud);

    NominalState nominal;
    SetupNominal(nominal);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    scan._end_time = 100.1;

    LidarPlaneFactor factor(*map, cloud, scan, config);
    factor.Commit(nominal);
    return map->IsBuilt();
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"build_meas_block", TestBuildMeasBlock},
        {"numeric_jacobian", TestNumericJacobian},
        {"numeric_all_columns", TestNumericAllColumns},
        {"commit", TestCommit},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (!test.fn()) {
            std::cerr << "[FAIL] " << test.name << std::endl;
            ++failed;
        } else {
            std::cout << "[PASS] " << test.name << std::endl;
        }
    }
    return failed == 0 ? 0 : 1;
}
