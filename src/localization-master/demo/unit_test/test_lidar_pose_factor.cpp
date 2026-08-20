#include <cmath>
#include <iostream>
#include <memory>

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "lidar_pose_factor.h"
#include "lidar_pose_meas.h"
#include "lidar_types.h"
#include "local_map.h"
#include "math/constants.h"
#include "math/pose_converter.h"
#include "pcd_preprocess.h"
#include "types/nav_state.h"
#include "types/state_layout.h"

namespace {

using msf::BlockId;
using msf::IterationContext;
using msf::NominalState;
using msf::StateLayout;
using msf::earth;
using msf::lidar::LidarPoseEstimate;
using msf::lidar::LidarPoseFactor;
using msf::lidar::LocalMap;
using msf::lidar::PointCloudXYZI;
using msf::lidar::PointType;

bool MatNear(const Eigen::MatrixXd& actual, const Eigen::MatrixXd& expected, double tol = 1.0e-6) {
    return actual.rows() == expected.rows() && actual.cols() == expected.cols() &&
           (actual - expected).cwiseAbs().maxCoeff() <= tol;
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
    config._process_option._state_dim = 15;
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
    config._lidar._max_iterations = 5;
    config._lidar._pose_meas_attitude_only = true;
    config._lidar._pose_meas_inflate = 50.0;
    config._lidar._pose_meas_pos_std_floor_m = Eigen::Vector3d(0.20, 0.20, 0.20);
    config._lidar._pose_meas_att_std_floor_deg = Eigen::Vector3d(0.30, 0.30, 0.50);
    config._lidar._pose_meas_min_eigenvalue = 20.0;
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
    nominal._Cnb = Eigen::Matrix3d::Identity();
    nominal._Cbn = nominal._Cnb.transpose();
    nominal._qnb = msf::pose_converter::m2qua(nominal._Cnb);
}

bool TestCovarianceSpdAndOrthogonal() {
    msf::GlobalConfig config;
    SetupConfig(config);
    Eigen::Matrix<double, 6, 6> lambda = Eigen::Matrix<double, 6, 6>::Identity() * 100.0;
    lambda(0, 1) = 10.0;
    lambda(1, 0) = 10.0;
    const auto R = msf::lidar::MakeLidarPoseCovariance(lambda, config);
    if (R.rows() != 6 || !R.allFinite()) {
        return false;
    }
    if ((R - R.transpose()).cwiseAbs().maxCoeff() > 1.0e-12) {
        return false;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(R);
    return solver.eigenvalues().minCoeff() > 0.0;
}

bool TestAttitudeOnlyMeasBlock() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud = MakePlaneZ1();
    map->Build(cloud);

    NominalState nominal;
    SetupNominal(nominal);
    earth eth;
    eth.Update(nominal._pos, nominal._vn);

    LidarPoseEstimate estimate;
    estimate._valid = true;
    estimate._C_meas = nominal._Cnb;
    estimate._p_meas_enu = msf::lidar::PositionEnu(nominal, config);
    estimate._R_pose = msf::lidar::MakeLidarPoseCovariance(
        Eigen::Matrix<double, 6, 6>::Identity() * 100.0, config);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    LidarPoseFactor factor(*map, cloud, scan, config, estimate);
    const StateLayout layout = StateLayout::FromStateDim(15);
    IterationContext ctx;
    const auto block = factor.BuildMeasBlock(nominal, eth, layout, ctx);
    if (!block._valid || !block.HasFullR() || block._z.size() != 3 ||
        block._H.rows() != 3 || block._H.cols() != 15) {
        return false;
    }
    const int rot_off = layout.Offset(BlockId::Rotation);
    if (!MatNear(block._H.block(0, rot_off, 3, 3), Eigen::Matrix3d::Identity())) {
        return false;
    }
    return block._z.cwiseAbs().maxCoeff() < 1.0e-9;
}

bool TestFullPoseMeasBlock() {
    msf::GlobalConfig config;
    SetupConfig(config);
    config._lidar._pose_meas_attitude_only = false;

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud = MakePlaneZ1();
    map->Build(cloud);

    NominalState nominal;
    SetupNominal(nominal);
    earth eth;
    eth.Update(nominal._pos, nominal._vn);

    LidarPoseEstimate estimate;
    estimate._valid = true;
    estimate._C_meas = nominal._Cnb;
    estimate._p_meas_enu = msf::lidar::PositionEnu(nominal, config);
    estimate._R_pose = msf::lidar::MakeLidarPoseCovariance(
        Eigen::Matrix<double, 6, 6>::Identity() * 100.0, config);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    LidarPoseFactor factor(*map, cloud, scan, config, estimate);
    const StateLayout layout = StateLayout::FromStateDim(15);
    IterationContext ctx;
    const auto block = factor.BuildMeasBlock(nominal, eth, layout, ctx);
    if (!block._valid || !block.HasFullR() || block._z.size() != 6 ||
        block._R.rows() != 6) {
        return false;
    }
    return block._z.cwiseAbs().maxCoeff() < 1.0e-6;
}

bool TestEstimateOnPlane() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud = MakePlaneZ1();
    map->Build(cloud);

    NominalState nominal;
    SetupNominal(nominal);
    earth eth;
    eth.Update(nominal._pos, nominal._vn);

    msf::LidarScanInfo scan;
    scan._timestamp = 100.0;
    const auto estimate =
        msf::lidar::EstimateLidarPose(*map, cloud, scan, nominal, eth, config);
    if (!estimate._valid) {
        return false;
    }
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    return (estimate._C_meas - I).cwiseAbs().maxCoeff() < 0.05;
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };
    const TestCase tests[] = {
        {"covariance_spd", TestCovarianceSpdAndOrthogonal},
        {"attitude_only_block", TestAttitudeOnlyMeasBlock},
        {"full_pose_block", TestFullPoseMeasBlock},
        {"estimate_on_plane", TestEstimateOnPlane},
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
