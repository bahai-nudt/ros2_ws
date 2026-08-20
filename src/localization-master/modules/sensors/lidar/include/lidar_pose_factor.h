#pragma once

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "lidar_pose_meas.h"
#include "lidar_types.h"
#include "local_map.h"
#include "types/sensors/lidar_message.h"

namespace msf {
namespace lidar {

/**
 * LiDAR 位姿量测因子（对应 POST_MSF set_lidar_pose_meas）。
 * 用 EstimateLidarPose 的 C_meas / p_meas / R_pose 在传播先验处线性化；
 * yaml pose_meas_attitude_only 选择 3 维姿态或 6 维姿态+位置。
 * Commit：更新被接受后把当前帧插入地图。
 */
class LidarPoseFactor : public MeasFactor {
public:
    LidarPoseFactor(LocalMap& map,
                    const PointCloudXYZI::Ptr& cloud_imu,
                    const LidarScanInfo& scan,
                    const GlobalConfig& config,
                    const LidarPoseEstimate& estimate);

    MeasBlock BuildMeasBlock(const NominalState& nominal, earth& eth,
                             const StateLayout& layout,
                             const IterationContext& ctx) override;

    void Commit(const NominalState& nominal) override;

private:
    LocalMap& map_;
    PointCloudXYZI::Ptr cloud_imu_;
    LidarScanInfo scan_;
    const GlobalConfig& config_;
    LidarPoseEstimate estimate_;
};

}  // namespace lidar
}  // namespace msf
