#include "lidar_plane_factor.h"

#include <algorithm>
#include <cmath>

#include "math/utility.h"
#include "pcd_preprocess.h"

namespace msf {
namespace lidar {

LidarPlaneFactor::LidarPlaneFactor(LocalMap& map,
                                   const PointCloudXYZI::Ptr& cloud_imu,
                                   const LidarScanInfo& scan,
                                   const GlobalConfig& config)
    : map_(map), cloud_imu_(cloud_imu), scan_(scan), config_(config) {}

MeasFactor::MeasBlock LidarPlaneFactor::BuildMeasBlock(const NominalState& nominal,
                                                       earth& eth,
                                                       const StateLayout& layout,
                                                       const IterationContext& ctx) {
    MeasBlock block;
    block._timestamp = scan_._timestamp;
    ++build_count_;

    if (!cloud_imu_ || cloud_imu_->empty() || !map_.IsBuilt()) {
        return block;
    }

    // 必须刷新 earth：多轮迭代中曲率半径/纬度相关量不能沿用旧位置
    eth.Update(nominal._pos, nominal._vn);

    const PointCloudXYZI::Ptr cloud_world =
        TransformCloudImuToWorld(cloud_imu_, nominal, config_);
    if (!cloud_world || cloud_world->empty()) {
        return block;
    }

    const bool rematch = ctx._iteration == 0 || ctx._last_converged;
    std::vector<LidarPlaneConstraint> matched_planes;
    LidarFeatureDiag feature_diag;
    map_.Match(cloud_world, rematch, matched_planes, feature_diag);
    last_feature_diag_ = feature_diag;

    const int min_effective = std::max(1, config_._lidar._min_effective_features);
    const double noise_var = config_._lidar._iterated_update_noise_var > 0.0
        ? config_._lidar._iterated_update_noise_var : 0.001;

    // 打分门控：与 POST_MSF AssembleLidarMeasurement 一致
    std::vector<LidarPlaneConstraint> effective;
    effective.reserve(matched_planes.size());
    for (const auto& constraint : matched_planes) {
        if (constraint.point_index >= cloud_imu_->size() ||
            constraint.point_index >= cloud_world->size()) {
            continue;
        }
        const PointType& point_world = cloud_world->points[constraint.point_index];
        const Eigen::Vector3d p_imu(cloud_imu_->points[constraint.point_index].x,
                                    cloud_imu_->points[constraint.point_index].y,
                                    cloud_imu_->points[constraint.point_index].z);
        const float pd2 = static_cast<float>(constraint.normal_enu.x()) * point_world.x +
                          static_cast<float>(constraint.normal_enu.y()) * point_world.y +
                          static_cast<float>(constraint.normal_enu.z()) * point_world.z +
                          static_cast<float>(constraint.plane_d);
        const Eigen::Vector3d p_lidar =
            config_._calibration._R_bl.transpose() * (p_imu - config_._calibration._T_lb_m);
        const double point_norm = std::max(p_lidar.norm(), 1.0e-12);
        const double s = 1.0 - config_._lidar._residual_score_scale * std::fabs(pd2) /
                                    std::sqrt(point_norm);
        if (s <= config_._lidar._residual_score_min) {
            continue;
        }

        LidarPlaneConstraint valid = constraint;
        valid.point_imu = p_imu;
        effective.push_back(valid);
    }
    last_feature_diag_.effective_features = static_cast<int>(effective.size());
    if (static_cast<int>(effective.size()) < min_effective) {
        return block;
    }

    const int nx = layout.Dim();
    const int rot_off = layout.Offset(BlockId::Rotation);
    const int pos_off = layout.Offset(BlockId::Position);
    Eigen::Matrix3d J_blh_to_enu;
    J_blh_to_enu << 0.0, 1.0 / eth._f_cbRNh, 0.0,
                    1.0 / eth._f_RMh, 0.0, 0.0,
                    0.0, 0.0, 1.0;
    const Eigen::Vector3d imu_position_enu = PositionEnu(nominal, config_);

    const int nr = static_cast<int>(effective.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nr, nx);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(nr);

    int row = 0;
    for (const auto& constraint : effective) {
        const Eigen::Vector3d p_I = constraint.point_imu;
        Eigen::Vector3d n_map = constraint.normal_enu;
        const double norm_n = n_map.norm();
        if (!p_I.allFinite() || !n_map.allFinite() ||
            !std::isfinite(constraint.plane_d) || norm_n < 1.0e-12) {
            continue;
        }
        n_map /= norm_n;
        const Eigen::Vector3d Cnb_p_I = nominal._Cnb * p_I;
        const Eigen::Vector3d p_w = imu_position_enu + Cnb_p_I;
        const double pd2 = n_map.dot(p_w) + constraint.plane_d;
        if (!std::isfinite(pd2)) {
            continue;
        }

        // 本实现的更新语义是 dx=K·h 且 ApplyErrorToNominal 做 x-=dx，
        // 因此有效 H 是残差对误差状态直接导数的相反数；此处保持 POST_MSF 符号。
        H.block<1, 3>(row, rot_off) = (askew(Cnb_p_I) * n_map).transpose();
        H.block<1, 3>(row, pos_off) = (-J_blh_to_enu.transpose() * n_map).transpose();
        z(row) = -pd2;
        ++row;
    }

    if (row == 0) {
        return block;
    }
    H.conservativeResize(row, Eigen::NoChange);
    z.conservativeResize(row);

    block._H = std::move(H);
    block._z = std::move(z);
    block._R_diag = Eigen::VectorXd::Constant(static_cast<int>(block._z.size()), noise_var);
    block._valid = true;
    last_block_ = block;
    return block;
}

void LidarPlaneFactor::Commit(const NominalState& nominal) {
    if (!cloud_imu_ || cloud_imu_->empty() || !map_.IsBuilt()) {
        return;
    }
    const PointCloudXYZI::Ptr cloud_world =
        TransformCloudImuToWorld(cloud_imu_, nominal, config_);
    map_.Insert(cloud_world);
}

}  // namespace lidar
}  // namespace msf
