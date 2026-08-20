#include "msf_algorithms/zupt.h"

#include <cmath>

#include "math/constants.h"
#include "math/utility.h"

namespace msf {
namespace algorithms {

ZuptDetector::ZuptDetector(const GlobalConfig::QualityControl::Zupt& zupt) : zupt_(zupt) {}

bool ZuptDetector::Update(double timestamp, double hor_speed, double ver_speed,
                          const NominalState& nominal) {
    const double static_ang_rate_rad = zupt_._static_ang_rate_dps * constants::_deg;
    const bool static_cond =
        hor_speed <= zupt_._static_hor_speed_mps &&
        std::fabs(ver_speed) <= zupt_._static_ver_speed_mps &&
        nominal._vn.norm() <= zupt_._static_ins_speed_mps &&
        nominal._wnb.norm() <= static_ang_rate_rad;

    double dt_zupt = 0.0;
    if (has_prev_ts_) {
        dt_zupt = timestamp - prev_ts_;
    }
    prev_ts_ = timestamp;
    has_prev_ts_ = true;
    last_timestamp_ = timestamp;

    if (!static_cond) {
        static_duration_ = 0.0;
    } else if (dt_zupt > 1.0e-4 && dt_zupt < 1.0) {
        static_duration_ += dt_zupt;
    }

    active_ = static_cond && static_duration_ >= zupt_._min_duration_sec;
    return active_;
}

void ZuptDetector::Reset() {
    static_duration_ = 0.0;
    has_prev_ts_ = false;
    prev_ts_ = 0.0;
    last_timestamp_ = 0.0;
    active_ = false;
}

ZeroVelocityFactor::ZeroVelocityFactor(const GlobalConfig::QualityControl::Zupt& zupt)
    : zupt_(zupt) {}

MeasFactor::MeasBlock ZeroVelocityFactor::BuildMeasBlock(const NominalState& nominal,
                                                         earth& eth,
                                                         const StateLayout& layout, const IterationContext& ctx) {
    // 与速度量测模型一致（z = vn_ins_L - 0）；核心骨架不因 ZUPT 增加专属入口
    (void)ctx;
    MeasFactor::MeasBlock block;
    block._valid = true;
    block._timestamp = nominal._t_cur;

    eth.Update(nominal._pos, nominal._vn);
    const Eigen::Vector3d web = nominal._wib - nominal._Cbn * eth._wnie;
    const Eigen::Vector3d lever_n = nominal._Cnb * nominal._lever;
    const Eigen::Vector3d wib_cross_lever_n =
        nominal._Cnb * (nominal._wib.cross(nominal._lever));
    const Eigen::Vector3d vn_ins_L =
        nominal._vn + nominal._Cnb * askew(web) * nominal._lever;

    const int nx = layout.Dim();
    block._H = Eigen::MatrixXd::Zero(3, nx);
    // 与 POST_MSF set_vel_meas / GNSS 速度因子同一姿态雅可比（ZUPT 共用该量测模型）。
    block._H.block(0, layout.Offset(BlockId::Rotation), 3, 3) =
        askew(wib_cross_lever_n) - askew(eth._wnie) * askew(lever_n);
    block._H.block(0, layout.Offset(BlockId::Velocity), 3, 3) = Eigen::Matrix3d::Identity();
    block._H.block(0, layout.Offset(BlockId::GyroBias), 3, 3) =
        -nominal._Cnb * askew(nominal._lever);
    if (layout.Has(BlockId::LeverArm)) {
        block._H.block(0, layout.Offset(BlockId::LeverArm), 3, 3) =
            nominal._Cnb * askew(web);
    }
    block._R_diag = zupt_._meas_std_mps.array().square().matrix();
    block._z = vn_ins_L;  // z = vn_ins_L - 0
    return block;
}

}  // namespace algorithms
}  // namespace msf
