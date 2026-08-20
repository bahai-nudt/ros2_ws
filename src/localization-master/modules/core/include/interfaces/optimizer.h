#pragma once

#include "math/earth.h"
#include "types/nav_state.h"

namespace msf {

/**
 * 优化器抽象接缝：向通用融合循环暴露“可被驱动的估计器状态”。
 * tools 的名义推进/融合循环只依赖此接口，不依赖具体优化器（EKF/图优化）。
 * 量测更新入口后续补充（Update / UpdateIterated）。
 */
class Optimizer {
public:
    virtual ~Optimizer() = default;

    /// 协方差时间更新：按 dt 传播误差协方差。
    virtual bool TimeUpdate(double dt, double inflation = 1.0) = 0;

    virtual NominalState& Nominal() = 0;
    virtual earth& Earth() = 0;
};

}  // namespace msf
