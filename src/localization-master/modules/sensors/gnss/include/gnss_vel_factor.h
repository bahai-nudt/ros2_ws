#pragma once

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "types/sensors/gnss_message.h"

namespace msf {
namespace gnss {

/**
 * GNSS 速度量测因子：数学与 POST_MSF set_vel_meas 一致（ZUPT 分支暂缓，
 * 待涉及模块统一设计后再接入），当前为无状态因子，可临时构造。
 */
class GnssVelFactor : public MeasFactor {
public:
    GnssVelFactor(const GnssVel& vel, const GlobalConfig& config);

    MeasBlock BuildMeasBlock(const NominalState& nominal, earth& eth,
                   const StateLayout& layout, const IterationContext& ctx) override;

private:
    GnssVel vel_;
    const GlobalConfig& config_;
};

}  // namespace gnss
}  // namespace msf
