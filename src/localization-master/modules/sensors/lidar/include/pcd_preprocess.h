#pragma once

#include <string>

#include <Eigen/Eigen>

#include "config/global_config.h"
#include "types/nav_state.h"
#include "types/sensors/lidar_message.h"
#include "lidar_types.h"

namespace msf {
namespace lidar {

/** 名义状态位置 → 局部地图 ENU 坐标（原点取 config._local_map_lla_deg）。 */
Eigen::Vector3d PositionEnu(const NominalState& nominal, const GlobalConfig& config);

/** IMU 系点 → 世界 ENU 点（位置 + 姿态变换）。 */
PointType PointImuToWorld(const PointType& point_imu,
                          const NominalState& nominal,
                          const GlobalConfig& config);

/** 整帧 IMU 系点云 → 世界 ENU 点云。 */
PointCloudXYZI::Ptr TransformCloudImuToWorld(const PointCloudXYZI::Ptr& cloud_imu,
                                             const NominalState& nominal,
                                             const GlobalConfig& config);

/** 整帧 LiDAR 系点云 → IMU 系点云（外参 R_bl / T_lb_m）。 */
PointCloudXYZI::Ptr TransformLidarCloudToImu(const PointCloudXYZI::Ptr& cloud_lidar,
                                             const GlobalConfig& config);

/** 读取 PCD：过滤非有限点/近点，并把相对时间(ms)写入 intensity。 */
bool LoadLidarPcd(const std::string& path,
                  double scan_begin_time,
                  PointCloudXYZI::Ptr cloud,
                  double* scan_end_time = nullptr);

/** 抽稀 + 体素滤波（参数来自 config._lidar）。 */
PointCloudXYZI::Ptr FilterRawScan(const PointCloudXYZI::Ptr& raw_cloud,
                                  const GlobalConfig& config);

/**
 * 世界系 PCD 输出路径：`<output_data_dir>/<lidar_frame_id>/<原文件名>`。
 * 对应 POST_MSF `BuildWorldPcdOutputPath`；目录名用 topic 推导的 `_lidar_frame_id`。
 */
std::string WorldPcdOutputPath(const GlobalConfig& config, const LidarScanInfo& scan);

/**
 * 开关打开时创建输出目录。失败返回 false（demo 应退出）。
 * 对应 POST_MSF 启动时 `create_directories(<output>/<frame_id>)`。
 */
bool EnsureWorldPcdOutputDir(const GlobalConfig& config);

/**
 * 把雷达系点云投到局部地图 ENU 并写 binary PCD（不抽稀、不体素）。
 * 对应 POST_MSF `ExportWorldPcd`：`p_world = ENU + Cnb * (R_bl * p_lidar + T_lb)`。
 * `cloud_lidar` 应为已去畸变（或未开 undistort 时的原始雷达系）点云；位姿用扫描时刻名义状态
 * （demo 在量测更新之后传入）。
 */
bool ExportWorldPcd(const PointCloudXYZI::Ptr& cloud_lidar,
                    const NominalState& nominal,
                    const GlobalConfig& config,
                    const std::string& output_path,
                    std::string* reject_reason = nullptr);

/**
 * `out.output_world_pcd` 打开时导出一帧世界系 PCD；失败只打日志，不参与融合。
 * 首帧只建图时不要调用（与 POST_MSF 一致）。
 */
void MaybeExportWorldPcd(const LidarScanInfo& scan,
                         const PointCloudXYZI::Ptr& cloud_lidar,
                         const NominalState& nominal,
                         const GlobalConfig& config);

}  // namespace lidar
}  // namespace msf
