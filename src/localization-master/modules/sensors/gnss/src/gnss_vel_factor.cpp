#include "gnss_vel_factor.h"

#include <cmath>

#include "gnss_quality.h"
#include "math/constants.h"
#include "math/utility.h"

namespace msf {
namespace gnss {

namespace {

// 对应 POST_MSF GnssVelMeasStdMps（不含 ZUPT）
Eigen::Vector3d GnssVelMeasStdMps(const GnssVel& vel,
                                  const GlobalConfig::Statistics::Meas& meas,
                                  const GlobalConfig::QualityControl& qc) {
    Eigen::Vector3d vel_std = meas._gnss_vel_meas_std_mps;
    const double factor = (vel._hor_speed < 1.0) ? 3.0 : 1.0;
    if (qc._gnss_vel_u_only_hor_speed_mps > 0.0 &&
        vel._hor_speed < qc._gnss_vel_u_only_hor_speed_mps) {
        vel_std(0) = vel_std(1) = 10.0;
        return vel_std;
    }
    return std::sqrt(factor) * vel_std;
}

}  // namespace

GnssVelFactor::GnssVelFactor(const GnssVel& vel, const GlobalConfig& config)
    : vel_(vel), config_(config) {}

MeasFactor::MeasBlock GnssVelFactor::BuildMeasBlock(const NominalState& nominal, earth& eth,
                                          const StateLayout& layout, const IterationContext& ctx) {
    (void)ctx;
    MeasBlock block;
    block._timestamp = vel_._timestamp;

    const double dt = nominal._t_cur - vel_._timestamp;
    if (std::fabs(dt) > 0.03) {
        return block;  // 时间对齐失败
    }
    if (!AllowedGnssVel(vel_)) {
        return block;  // 质量门控失败
    }

    eth.Update(nominal._pos, nominal._vn);
    const Eigen::Vector3d web = nominal._wib - nominal._Cbn * eth._wnie;
    const Eigen::Vector3d lever_n = nominal._Cnb * nominal._lever;
    const Eigen::Vector3d wib_cross_lever_n =
        nominal._Cnb * (nominal._wib.cross(nominal._lever));
    const Eigen::Vector3d vn_ins_L =
        nominal._vn + nominal._Cnb * askew(web) * nominal._lever;

    const double trk_rad = vel_._trk_gnd;
    const Eigen::Vector3d vn_gnss(vel_._hor_speed * std::sin(trk_rad),
                                  vel_._hor_speed * std::cos(trk_rad),
                                  vel_._ver_speed);
    const Eigen::Vector3d vel_std = GnssVelMeasStdMps(vel_, config_._statistics._meas,
                                                      config_._quality_control);

    const int nx = layout.Dim();
    block._H = Eigen::MatrixXd::Zero(3, nx);
    block._R_diag = vel_std.array().square().matrix();
    block._z = vn_ins_L - vn_gnss;
    // POST_MSF set_vel_meas：H_phi = [Cnb(wib×lever)]x - [wnie]x[Cnb lever]x
    block._H.block(0, layout.Offset(BlockId::Rotation), 3, 3) =
        askew(wib_cross_lever_n) - askew(eth._wnie) * askew(lever_n);
    block._H.block(0, layout.Offset(BlockId::Velocity), 3, 3) = Eigen::Matrix3d::Identity();
    block._H.block(0, layout.Offset(BlockId::GyroBias), 3, 3) =
        -nominal._Cnb * askew(nominal._lever);
    if (layout.Has(BlockId::LeverArm)) {
        block._H.block(0, layout.Offset(BlockId::LeverArm), 3, 3) =
            nominal._Cnb * askew(web);
    }
    block._valid = true;
    return block;
}

}  // namespace gnss
}  // namespace msf
