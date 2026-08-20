#include "lidar_pose_factor.h"

#include <cmath>

#include "math/pose_converter.h"
#include "math/t_quat.h"
#include "pcd_preprocess.h"
#include "types/state_layout.h"

namespace msf {
namespace lidar {

LidarPoseFactor::LidarPoseFactor(LocalMap& map,
                                 const PointCloudXYZI::Ptr& cloud_imu,
                                 const LidarScanInfo& scan,
                                 const GlobalConfig& config,
                                 const LidarPoseEstimate& estimate)
    : map_(map),
      cloud_imu_(cloud_imu),
      scan_(scan),
      config_(config),
      estimate_(estimate) {}

MeasFactor::MeasBlock LidarPoseFactor::BuildMeasBlock(const NominalState& nominal,
                                                      earth& eth,
                                                      const StateLayout& layout,
                                                      const IterationContext&) {
    MeasBlock block;
    block._timestamp = scan_._timestamp;
    if (!estimate_._valid || layout.Dim() <= 0) {
        return block;
    }

    eth.Update(nominal._pos, nominal._vn);
    t_quat q_meas = pose_converter::m2qua(estimate_._C_meas);
    t_quat::normlize(q_meas);
    const t_quat dq = q_meas * t_quat::conj(nominal._qnb);
    const Eigen::Vector3d z_phi = pose_converter::q2rv(dq);
    if (!z_phi.allFinite()) {
        return block;
    }

    const int nx = layout.Dim();
    const int rot_off = layout.Offset(BlockId::Rotation);
    const int pos_off = layout.Offset(BlockId::Position);
    const bool attitude_only = config_._lidar._pose_meas_attitude_only;

    if (attitude_only) {
        block._H = Eigen::MatrixXd::Zero(3, nx);
        block._H.block(0, rot_off, 3, 3) = Eigen::Matrix3d::Identity();
        block._z = z_phi;
        block._R = estimate_._R_pose.block(0, 0, 3, 3);
        block._R_diag = block._R.diagonal();
        block._valid = true;
        return block;
    }

    if (std::abs(eth._f_RMh) < 1.0e-12 || std::abs(eth._f_cbRNh) < 1.0e-12) {
        return block;
    }
    const Eigen::Vector3d p_pred_enu = PositionEnu(nominal, config_);
    const Eigen::Vector3d z_p = p_pred_enu - estimate_._p_meas_enu;
    if (!z_p.allFinite()) {
        return block;
    }
    Eigen::Matrix3d J_blh_to_enu;
    J_blh_to_enu << 0.0, 1.0 / eth._f_cbRNh, 0.0,
                    1.0 / eth._f_RMh, 0.0, 0.0,
                    0.0, 0.0, 1.0;

    block._H = Eigen::MatrixXd::Zero(6, nx);
    block._H.block(0, rot_off, 3, 3) = Eigen::Matrix3d::Identity();
    block._H.block(3, pos_off, 3, 3) = J_blh_to_enu;
    block._z = Eigen::VectorXd::Zero(6);
    block._z.segment<3>(0) = z_phi;
    block._z.segment<3>(3) = z_p;
    block._R = estimate_._R_pose;
    block._R_diag = block._R.diagonal();
    block._valid = true;
    return block;
}

void LidarPoseFactor::Commit(const NominalState& nominal) {
    if (!cloud_imu_ || cloud_imu_->empty() || !map_.IsBuilt()) {
        return;
    }
    const PointCloudXYZI::Ptr cloud_world =
        TransformCloudImuToWorld(cloud_imu_, nominal, config_);
    map_.Insert(cloud_world);
}

}  // namespace lidar
}  // namespace msf
