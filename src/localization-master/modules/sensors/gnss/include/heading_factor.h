#pragma once

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "types/sensors/gnss_message.h"

namespace msf {
namespace gnss {

/** 双天线航向量测因子：数学与 POST_MSF set_heading_meas 一致。 */
class HeadingFactor : public MeasFactor {
public:
    HeadingFactor(const HeadingData& heading, const GlobalConfig& config);

    MeasBlock BuildMeasBlock(const NominalState& nominal, earth& eth,
                   const StateLayout& layout, const IterationContext& ctx) override;

private:
    HeadingData heading_;
    const GlobalConfig& config_;
};

}  // namespace gnss
}  // namespace msf
