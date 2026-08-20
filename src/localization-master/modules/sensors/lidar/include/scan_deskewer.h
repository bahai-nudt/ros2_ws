#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "config/global_config.h"
#include "interfaces/inertial_propagator.h"
#include "types/nav_state.h"
#include "types/sensors/imu_message.h"
#include "types/sensors/lidar_message.h"
#include "lidar_types.h"

namespace msf {
namespace lidar {

/** 帧内去畸变位姿样本（对应 POST_MSF LidarPoseSample）。 */
struct DeskewPoseSample {
    double _timestamp = 0.0;
    Eigen::Vector3d _position_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d _velocity_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d _acceleration_enu = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _Cnb = Eigen::Matrix3d::Identity();
    Eigen::Vector3d _angular_velocity_body = Eigen::Vector3d::Zero();  // 对应 NominalState::_wib
};

/** 去畸变诊断（对应 POST_MSF UndistortDiag）。 */
struct DeskewDiag {
    double _max_rot_diff_rad = 0.0;
    double _max_rot_diff_rel_time_ms = 0.0;
    Eigen::Vector3d _max_rot_diff_vec = Eigen::Vector3d::Zero();
    double _max_compensation_m = 0.0;
    double _mean_compensation_m = 0.0;
};

/**
 * 帧内运动补偿（去畸变 / deskew）。
 * 对应 POST_MSF 的 BuildUndistortPoses + UndistortScan：
 *   - 从扫描起始名义状态出发，用 InertialPropagator 前推位姿链；
 *   - 把每个点补偿回“扫描起始时刻的雷达系”。
 * 参考时刻固定为 scan._timestamp（帧头），不要改成扫描结束。
 */
class ScanDeskewer {
public:
    ScanDeskewer(const GlobalConfig& config, InertialPropagator& propagator);

    /** 构建 scan_begin -> scan._end_time 的位姿链（对应 BuildUndistortPoses）。 */
    bool BuildPoseChain(const NominalState& scan_begin,
                        const std::vector<ImuData>& imu_buffer,
                        const LidarScanInfo& scan);

    /** 查询任意点时刻的位姿；仅用于插值/测试，DeskewScan 内部保持参考算法路径。 */
    bool PoseAt(double point_time, DeskewPoseSample& pose) const;

    /** 把 raw_cloud 补偿到扫描起始时刻雷达系（对应 UndistortScan）。 */
    PointCloudXYZI::Ptr DeskewScan(const PointCloudXYZI::Ptr& raw_cloud,
                                   const std::vector<ImuData>& imu_buffer,
                                   const LidarScanInfo& scan,
                                   const NominalState& scan_begin,
                                   DeskewDiag* diag = nullptr);

    /** 最近一次 BuildPoseChain / DeskewScan 留下的位姿链长度。 */
    int PoseCount() const { return static_cast<int>(poses_.size()); }

private:
    DeskewPoseSample BuildPoseSample(const NominalState& nominal) const;

    const GlobalConfig& config_;
    InertialPropagator& propagator_;
    std::vector<DeskewPoseSample> poses_;
};

}  // namespace lidar
}  // namespace msf
