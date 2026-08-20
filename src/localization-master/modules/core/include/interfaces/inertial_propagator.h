#pragma once

#include <Eigen/Eigen>

#include "math/earth.h"
#include "types/nav_state.h"

namespace msf {

/**
 * 惯性递推抽象：对调用方持有的名义状态与地球模型做机械编排。
 * msf_lidar 等模块只依赖此接口，不依赖 msf_ins。
 * 签名与 InsPropagator::Update 的双子样形式一致。
 */
class InertialPropagator {
public:
    virtual ~InertialPropagator() = default;

    virtual bool Propagate(NominalState& nominal, earth& eth,
                           const Eigen::Vector3d& gyro_prev,
                           const Eigen::Vector3d& accel_prev,
                           const Eigen::Vector3d& gyro_curr,
                           const Eigen::Vector3d& accel_curr,
                           double dt) = 0;
};

}  // namespace msf
