#include "heading_factor.h"

#include <algorithm>
#include <cmath>

#include "gnss_quality.h"
#include "math/constants.h"
#include "math/pose_converter.h"
#include "math/utility.h"

namespace msf {
namespace gnss {

HeadingFactor::HeadingFactor(const HeadingData& heading, const GlobalConfig& config)
    : heading_(heading), config_(config) {}

MeasFactor::MeasBlock HeadingFactor::BuildMeasBlock(const NominalState& nominal, earth& eth,
                                          const StateLayout& layout, const IterationContext& ctx) {
    (void)ctx;
    MeasBlock block;
    block._timestamp = heading_._timestamp;

    const double dt = nominal._t_cur - heading_._timestamp;
    if (std::fabs(dt) > 0.03) {
        return block;  // 时间对齐失败
    }
    if (!AllowedHeading(heading_)) {
        return block;  // 质量门控失败
    }
    // 航向 std 上限门控（用原始 std 判定）
    const double heading_std_max_rad =
        config_._quality_control._heading_meas_std_max_deg * constants::_deg;
    if (heading_._heading_std > heading_std_max_rad) {
        return block;
    }

    const Eigen::Vector3d baseline_b(heading_._baseline_length, 0.0, 0.0);
    const Eigen::Vector3d baseline_n = nominal._Cnb * baseline_b;
    const double l_e = baseline_n(0);
    const double l_n = baseline_n(1);
    const double rho2 = l_e * l_e + l_n * l_n;
    if (rho2 < 1e-8) {
        return block;
    }

    const double heading_pred = std::atan2(l_e, l_n);
    const double heading_res = pose_converter::WrapAngle(heading_._heading - heading_pred);
    Eigen::Matrix<double, 1, 3> d_heading_d_l;
    d_heading_d_l << l_n / rho2, -l_e / rho2, 0.0;
    const Eigen::Matrix<double, 1, 3> h_heading_phi =
        d_heading_d_l * (-askew(baseline_n));

    const int nx = layout.Dim();
    block._H = Eigen::MatrixXd::Zero(1, nx);
    block._R_diag = Eigen::VectorXd::Zero(1);
    block._z = Eigen::VectorXd::Zero(1);
    block._H.block(0, layout.Offset(BlockId::Rotation), 1, 3) = h_heading_phi;
    block._z(0) = heading_res;
    // 量测噪声地板 + 缩放
    const double heading_std_floor_rad =
        config_._statistics._meas._heading_meas_std_floor_deg * constants::_deg;
    const double heading_std_used =
        std::max(heading_._heading_std * config_._statistics._meas._heading_meas_std_scale,
                 heading_std_floor_rad);
    block._R_diag(0) = heading_std_used * heading_std_used;
    block._valid = true;
    return block;
}

}  // namespace gnss
}  // namespace msf
