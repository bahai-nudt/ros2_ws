#pragma once

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "types/sensors/gnss_message.h"

namespace msf {
namespace gnss {

/** GNSS 位置量测因子：数学与 POST_MSF set_pos_meas 一致，只做接口适配。 */
class GnssPosFactor : public MeasFactor {
public:
    GnssPosFactor(const GnssPos& pos, const GlobalConfig& config);

    MeasBlock BuildMeasBlock(const NominalState& nominal, earth& eth,
                   const StateLayout& layout, const IterationContext& ctx) override;

private:
    GnssPos pos_;
    const GlobalConfig& config_;
};

}  // namespace gnss
}  // namespace msf
