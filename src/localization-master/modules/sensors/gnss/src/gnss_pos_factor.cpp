#include "gnss_pos_factor.h"

#include <algorithm>
#include <cmath>

#include "gnss_quality.h"
#include "math/utility.h"

namespace msf {
namespace gnss {

GnssPosFactor::GnssPosFactor(const GnssPos& pos, const GlobalConfig& config)
    : pos_(pos), config_(config) {}

MeasFactor::MeasBlock GnssPosFactor::BuildMeasBlock(const NominalState& nominal, earth& eth,
                                          const StateLayout& layout, const IterationContext& ctx) {
    (void)ctx;
    MeasBlock block;
    block._timestamp = pos_._timestamp;

    const double dt = nominal._t_cur - pos_._timestamp;
    if (std::fabs(dt) > 0.03) {
        return block;  // 时间对齐失败
    }
    if (!AllowedGnssPos(pos_)) {
        return block;  // 质量门控失败
    }

    eth.Update(nominal._pos, nominal._vn);
    Eigen::Matrix3d Mpv;
    Mpv << 0.0, eth._f_RMh, 0.0,
           eth._f_cbRNh, 0.0, 0.0,
           0.0, 0.0, 1.0;

    const Eigen::Vector3d lever_n = nominal._Cnb * nominal._lever;
    const Eigen::Vector3d pos_ins_L = nominal._pos + Mpv * lever_n;
    const Eigen::Vector3d blh_gnss = pos_._blh;

    const bool use_float_downweight = pos_._pos_type == 34;
    const Eigen::Vector3d pos_std_floor_m = use_float_downweight
        ? config_._statistics._meas._gnss_float_pos_std_floor_m
        : config_._statistics._meas._gnss_fixed_pos_std_floor_m;
    const double std_lat_m = std::max(pos_._blh_std(0), pos_std_floor_m(1));  // lat <- N
    const double std_lon_m = std::max(pos_._blh_std(1), pos_std_floor_m(0));  // lon <- E
    const double std_h_m = std::max(pos_._blh_std(2), pos_std_floor_m(2));
    const double std_lat_rad = std_lat_m * eth._f_RMh;
    const double std_lon_rad = std_lon_m * eth._f_cbRNh;
    const Eigen::Vector3d pos_var(std_lat_rad * std_lat_rad,
                                  std_lon_rad * std_lon_rad,
                                  std_h_m * std_h_m);

    const int nx = layout.Dim();
    block._H = Eigen::MatrixXd::Zero(3, nx);
    block._R_diag = pos_var;
    block._z = pos_ins_L - blh_gnss;
    block._H.block(0, layout.Offset(BlockId::Rotation), 3, 3) = Mpv * askew(lever_n);
    block._H.block(0, layout.Offset(BlockId::Position), 3, 3) = Eigen::Matrix3d::Identity();
    if (layout.Has(BlockId::LeverArm)) {
        block._H.block(0, layout.Offset(BlockId::LeverArm), 3, 3) = Mpv * nominal._Cnb;
    }
    block._valid = true;
    return block;
}

}  // namespace gnss
}  // namespace msf
