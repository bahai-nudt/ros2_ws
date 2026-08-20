#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "config/global_config.h"
#include "math/earth.h"
#include "types/nav_state.h"
#include "types/sensors/imu_message.h"

namespace msf {

/**
 * LIO 静基座 IMU 初始化（参考 FAST-LIO2：前若干帧均值）。
 * - 加速度均值 → 调平得 pitch/roll，yaw=0（无寻北）
 * - 陀螺均值 → 陀螺零偏
 * - 位置取 origin_lla_rad，速度为 0
 * 要求载体近似静止；失败返回 false。
 */
bool InitializeLIO(const std::vector<ImuData>& imu_buffer,
                      const Eigen::Vector3d& origin_lla_rad,
                      const GlobalConfig::ImuCalib& imu_calib,
                      int min_samples,
                      NominalState& nominal,
                      earth& eth);

}  // namespace msf
