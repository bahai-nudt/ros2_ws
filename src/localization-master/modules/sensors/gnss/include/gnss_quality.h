#pragma once

#include <string>

#include "types/sensors/gnss_message.h"

namespace msf {
namespace gnss {

/** GNSS 位置质量门控（解状态 / 经纬度范围 / 标准差）。 */
bool AllowedGnssPos(const GnssPos& pos);

/** GNSS 速度质量门控（解状态 / 数值有限 / 航迹角范围）。 */
bool AllowedGnssVel(const GnssVel& vel);

/** 双天线航向质量门控（解状态 / 数值有限 / 标准差与基线长度）。 */
bool AllowedHeading(const HeadingData& heading);

/**
 * GNSS 速度拒绝原因（供日志/诊断）。
 * dt_sins_minus_meas：SINS 时刻减量测时刻，超 0.03s 判定时间同步失败。
 */
std::string GnssVelRejectReason(const GnssVel& vel, double dt_sins_minus_meas);

}  // namespace gnss
}  // namespace msf
