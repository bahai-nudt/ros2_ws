#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "config/global_config.h"
#include "math/earth.h"
#include "types/nav_state.h"
#include "types/sensors/gnss_message.h"
#include "types/sensors/imu_message.h"

namespace msf {
namespace gnss {

/**
 * 前向搜索第一组可用 GNSS/Heading 观测，初始化名义状态（对应 POST_MSF init_sins_post）。
 * 成功后 nominal 与 eth 均被填充：姿态由 heading 确定，位置/速度由 GNSS 及杆臂补偿得到，
 * 零偏/刻度因子来自 imu_calib。
 */
bool InitializeNominal(const std::vector<ImuData>& imu_buffer,
                       const std::vector<GnssPos>& gnsspos_buffer,
                       const std::vector<GnssVel>& gnssvel_buffer,
                       const std::vector<HeadingData>& heading_buffer,
                       const Eigen::Vector3d& imu_to_antenna_xyz_m,
                       const GlobalConfig::ImuCalib& imu_calib,
                       NominalState& nominal,
                       earth& eth);

}  // namespace gnss
}  // namespace msf
