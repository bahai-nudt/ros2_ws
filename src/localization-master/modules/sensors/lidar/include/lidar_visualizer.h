#pragma once

#include <memory>

#include "config/global_config.h"
#include "lidar_types.h"
#include "types/nav_state.h"

namespace msf {
namespace lidar {

/**
 * 点云可视化（旁路，不参与融合）。
 *
 * `out.visualization_mode`：
 * - 1 原始雷达系：每帧替换，固定相机看雷达原点
 * - 2 去畸变雷达系：同上，点云为 deskew 后的雷达系
 * - 3 世界系：累积 ENU 点云 + 雷达轨迹 + 跟随相机（原 POST_MSF 行为）
 */
class LidarVisualizer {
public:
    LidarVisualizer();
    ~LidarVisualizer();

    LidarVisualizer(const LidarVisualizer&) = delete;
    LidarVisualizer& operator=(const LidarVisualizer&) = delete;

    void ShowScan(const PointCloudXYZI::Ptr& raw_lidar,
                  const PointCloudXYZI::Ptr& undistorted_lidar,
                  const PointCloudXYZI::Ptr& cloud_imu,
                  const NominalState& nominal,
                  const GlobalConfig& config,
                  int mode);

private:
    struct State;
    std::unique_ptr<State> state_;

    void EnsureWindow(int mode);
    void UpdateDisplayedCloud(const PointCloudXYZI::Ptr& cloud, bool accumulate);
    void UpdateWorldFollowCamera(const NominalState& nominal, const GlobalConfig& config);
};

/**
 * 按可视化模式选出要显示的点云（不做窗口）。
 * 1/2 对雷达系点做与融合相同的抽稀+体素；3 把 IMU 系点投到局部 ENU。
 */
PointCloudXYZI::Ptr VisualizationCloud(int mode,
                                       const PointCloudXYZI::Ptr& raw_lidar,
                                       const PointCloudXYZI::Ptr& undistorted_lidar,
                                       const PointCloudXYZI::Ptr& cloud_imu,
                                       const NominalState& nominal,
                                       const GlobalConfig& config);

}  // namespace lidar
}  // namespace msf
