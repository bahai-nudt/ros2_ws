#pragma once

#include <vector>

#include "interfaces/inertial_propagator.h"
#include "interfaces/optimizer.h"
#include "types/sensors/imu_message.h"

namespace msf {
namespace tools {

/**
 * 推进到目标时刻：按 IMU 采样点切段，插值角速度/加速度，
 * 调用 propagator 递推名义状态，并对本步做 optimizer.TimeUpdate（协方差）。
 * advanced_dt 若非空，写入本次成功推进的累计时长（秒）。
 * 对应 POST_MSF EkfTimeUpdate。
 */
bool AdvanceNominal(Optimizer& optimizer, InertialPropagator& propagator,
                    const std::vector<ImuData>& imu_buffer, double target_time,
                    double* advanced_dt = nullptr);

}  // namespace tools
}  // namespace msf
