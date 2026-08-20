#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "interfaces/inertial_propagator.h"
#include "math/earth.h"
#include "types/nav_state.h"
#include "imu_processor.h"

namespace msf {

// 无状态惯性导航递推器：对调用方持有的名义状态与地球模型做机械编排。
class InsPropagator : public InertialPropagator {
public:
    InsPropagator() = default;

    Eigen::Vector3d AlignCoarse(const NominalState& nominal,
                                const Eigen::Vector3d& wmm,
                                const Eigen::Vector3d& vmm) const;

    bool Propagate(NominalState& nominal, earth& eth,
                   const Eigen::Vector3d& gyro_prev, const Eigen::Vector3d& accel_prev,
                   const Eigen::Vector3d& gyro_curr, const Eigen::Vector3d& accel_curr,
                   double dt) override;

private:
    ImuProcessor imu_;
};

}  // namespace msf
