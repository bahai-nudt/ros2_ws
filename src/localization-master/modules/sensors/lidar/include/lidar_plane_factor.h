#pragma once

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "lidar_types.h"
#include "local_map.h"
#include "types/sensors/lidar_message.h"

namespace msf {
namespace lidar {

/**
 * LiDAR 点面量测因子。
 * BuildMeasBlock：当前名义状态 -> 世界系点云 -> LocalMap::Match -> 打分门控 -> H/z/_R_diag。
 * Commit：更新被接受后用修正后名义状态重新变换点云并插入地图。
 * 对应 POST_MSF lidar_update.cpp 的 AssembleLidarMeasurement + BuildLidarLinearSystem。
 */
class LidarPlaneFactor : public MeasFactor {
public:
    LidarPlaneFactor(LocalMap& map,
                     const PointCloudXYZI::Ptr& cloud_imu,
                     const LidarScanInfo& scan,
                     const GlobalConfig& config);

    MeasBlock BuildMeasBlock(const NominalState& nominal, earth& eth,
                             const StateLayout& layout,
                             const IterationContext& ctx) override;

    void Commit(const NominalState& nominal) override;

    /** 最近一次 BuildMeasBlock 的匹配诊断（用于日志/回归定位）。 */
    const LidarFeatureDiag& LastFeatureDiag() const { return last_feature_diag_; }

    /** 最近一次 BuildMeasBlock 的量测块（用于残差日志/回归定位）。 */
    const MeasBlock& LastMeasBlock() const { return last_block_; }

    /** 本次因子被线性化调用的次数（用于迭代数对比）。 */
    int BuildCount() const { return build_count_; }

private:
    LocalMap& map_;
    PointCloudXYZI::Ptr cloud_imu_;
    LidarScanInfo scan_;
    const GlobalConfig& config_;
    LidarFeatureDiag last_feature_diag_;
    MeasBlock last_block_;
    int build_count_ = 0;
};

}  // namespace lidar
}  // namespace msf
