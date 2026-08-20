#pragma once

#include <Eigen/Eigen>

#include "config/global_config.h"
#include "lidar_types.h"
#include "local_map.h"
#include "math/earth.h"
#include "types/nav_state.h"
#include "types/sensors/lidar_message.h"

namespace msf {
namespace lidar {

/** 局部位姿 Gauss-Newton 的输出（对应 POST_MSF IteratedLidarPoseUpdate 前半）。 */
struct LidarPoseEstimate {
    bool _valid = false;
    Eigen::Matrix3d _C_meas = Eigen::Matrix3d::Identity();
    Eigen::Vector3d _p_meas_enu = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> _R_pose = Eigen::Matrix<double, 6, 6>::Identity();
    LidarFeatureDiag _feature_diag;
    int _iterations = 0;
    int _converge_iter = -1;
    bool _final_converged = false;
    double _max_dx_last = 0.0;
};

/**
 * 由点面信息矩阵构造 6×6 位姿协方差。
 * 特征向量做极分解正交化后再重建 R，最后对称化。
 */
Eigen::Matrix<double, 6, 6> MakeLidarPoseCovariance(
    const Eigen::Matrix<double, 6, 6>& lambda,
    const GlobalConfig& config);

/**
 * 在名义状态副本上做姿态+位置点面 GN，不改主滤波 P。
 * 失败时 _valid=false，调用方可保持先验不动。
 */
LidarPoseEstimate EstimateLidarPose(LocalMap& map,
                                    const PointCloudXYZI::Ptr& cloud_imu,
                                    const LidarScanInfo& scan,
                                    const NominalState& nominal,
                                    earth eth,
                                    const GlobalConfig& config);

}  // namespace lidar
}  // namespace msf
