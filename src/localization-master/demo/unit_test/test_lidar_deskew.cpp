#include <cmath>
#include <iostream>
#include <memory>

#include "config/global_config.h"
#include "ins_propagator.h"
#include "lidar_types.h"
#include "math/constants.h"
#include "scan_deskewer.h"
#include "types/nav_state.h"

namespace {

using msf::lidar::DeskewDiag;
using msf::lidar::PointCloudXYZI;
using msf::lidar::PointType;
using msf::lidar::DeskewPoseSample;
using msf::lidar::ScanDeskewer;

bool Near(double actual, double expected, double tol = 1.0e-6) {
    return std::fabs(actual - expected) <= tol;
}

bool NearPoint(const PointType& p, double x, double y, double z, double tol = 1.0e-6) {
    return Near(p.x, x, tol) && Near(p.y, y, tol) && Near(p.z, z, tol);
}

PointType MakePoint(float x, float y, float z, float intensity) {
    PointType p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = intensity;
    return p;
}

void SetupConfig(msf::GlobalConfig& config) {
    config._local_map_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);
    config._calibration._R_bl = Eigen::Matrix3d::Identity();
    config._calibration._T_lb_m = Eigen::Vector3d::Zero();
}

void SetupNominal(msf::NominalState& nominal) {
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

// 无 IMU 递推/静止场景：DeskewScan 应原样返回点云
bool TestStaticUnchanged() {
    msf::GlobalConfig config;
    SetupConfig(config);

    msf::NominalState nominal;
    SetupNominal(nominal);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    scan._end_time = 100.1;

    PointCloudXYZI::Ptr raw(new PointCloudXYZI());
    raw->push_back(MakePoint(1.0f, 0.0f, 0.0f, 0.0f));
    raw->push_back(MakePoint(2.0f, 0.0f, 0.0f, 50.0f));
    raw->push_back(MakePoint(3.0f, 0.0f, 0.0f, 100.0f));

    msf::InsPropagator propagator;
    ScanDeskewer deskewer(config, propagator);
    const std::vector<msf::ImuData> imu_buffer;
    const auto out = deskewer.DeskewScan(raw, imu_buffer, scan, nominal);

    if (!out || out->size() != 3) {
        return false;
    }
    return NearPoint(out->points[0], 1.0, 0.0, 0.0) &&
           NearPoint(out->points[1], 2.0, 0.0, 0.0) &&
           NearPoint(out->points[2], 3.0, 0.0, 0.0);
}

// 有 IMU 递推时，BuildPoseChain 应产生多个样本，PoseAt 能查到中间时刻
bool TestPoseAt() {
    msf::GlobalConfig config;
    SetupConfig(config);

    msf::NominalState nominal;
    SetupNominal(nominal);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    scan._end_time = 100.1;

    std::vector<msf::ImuData> imu_buffer(3);
    for (int i = 0; i < 3; ++i) {
        imu_buffer[i]._timestamp = 100.0 + 0.05 * i;
        imu_buffer[i]._gyro = Eigen::Vector3d::Zero();
        imu_buffer[i]._accel = Eigen::Vector3d::Zero();
    }

    msf::InsPropagator propagator;
    ScanDeskewer deskewer(config, propagator);
    if (!deskewer.BuildPoseChain(nominal, imu_buffer, scan)) {
        return false;
    }

    DeskewPoseSample pose;
    if (!deskewer.PoseAt(100.05, pose)) {
        return false;
    }
    return Near(pose._timestamp, 100.05, 1.0e-6) &&
           pose._position_enu.allFinite() &&
           pose._Cnb.allFinite();
}

// 帧内旋转运动：首点（t=0ms）不动，末点（t=100ms）应被补偿
bool TestMotionCompensates() {
    msf::GlobalConfig config;
    SetupConfig(config);

    msf::NominalState nominal;
    SetupNominal(nominal);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    scan._end_time = 100.1;

    std::vector<msf::ImuData> imu_buffer(3);
    for (int i = 0; i < 3; ++i) {
        imu_buffer[i]._timestamp = 100.0 + 0.05 * i;
        imu_buffer[i]._gyro = Eigen::Vector3d(0.0, 0.0, 0.1);  // 绕 Z 旋转
        imu_buffer[i]._accel = Eigen::Vector3d::Zero();
    }

    PointCloudXYZI::Ptr raw(new PointCloudXYZI());
    raw->push_back(MakePoint(10.0f, 0.0f, 0.0f, 0.0f));
    raw->push_back(MakePoint(10.0f, 0.0f, 0.0f, 100.0f));

    msf::InsPropagator propagator;
    ScanDeskewer deskewer(config, propagator);
    DeskewDiag diag;
    const auto out = deskewer.DeskewScan(raw, imu_buffer, scan, nominal, &diag);

    if (!out || out->size() != 2) {
        return false;
    }
    // 扫描起始时刻的点不应被补偿
    if (!NearPoint(out->points[0], 10.0, 0.0, 0.0, 1.0e-4)) {
        return false;
    }
    // 帧尾的点应被补偿，且与原始点不同
    const double dx = std::fabs(out->points[1].x - 10.0);
    const double dy = std::fabs(out->points[1].y - 0.0);
    if (dx + dy < 1.0e-4) {
        return false;
    }
    return diag._max_compensation_m > 0.0;
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"static_unchanged", TestStaticUnchanged},
        {"pose_at", TestPoseAt},
        {"motion_compensates", TestMotionCompensates},
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
