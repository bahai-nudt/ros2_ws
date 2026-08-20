#include <cmath>
#include <iostream>

#include "config/global_config.h"
#include "ekf.h"
#include "ekf_state.h"
#include "gnss_pos_factor.h"
#include "gnss_vel_factor.h"
#include "heading_factor.h"
#include "math/constants.h"
#include "math/pose_converter.h"
#include "math/utility.h"
#include "msf_algorithms/zupt.h"

namespace msf {
namespace {

// ===== 参考实现：POST_MSF set_*_meas 数学（不含 ZUPT），仅用于数值对比 =====

bool RefPosMeas(const NominalState& nominal, earth& eth, const StateLayout& layout,
                const GnssPos& pos, const GlobalConfig& config,
                Eigen::MatrixXd& H, Eigen::MatrixXd& R, Eigen::VectorXd& z) {
    const double dt = nominal._t_cur - pos._timestamp;
    if (std::fabs(dt) > 0.03) {
        return false;
    }
    eth.Update(nominal._pos, nominal._vn);
    Eigen::Matrix3d Mpv;
    Mpv << 0.0, eth._f_RMh, 0.0,
           eth._f_cbRNh, 0.0, 0.0,
           0.0, 0.0, 1.0;
    const Eigen::Vector3d lever_n = nominal._Cnb * nominal._lever;
    const Eigen::Vector3d pos_ins_L = nominal._pos + Mpv * lever_n;
    const Eigen::Vector3d blh_gnss = pos._blh;
    const bool use_float_downweight = pos._pos_type == 34;
    const Eigen::Vector3d pos_std_floor_m = use_float_downweight
        ? config._statistics._meas._gnss_float_pos_std_floor_m
        : config._statistics._meas._gnss_fixed_pos_std_floor_m;
    const double std_lat_m = std::max(pos._blh_std(0), pos_std_floor_m(1));
    const double std_lon_m = std::max(pos._blh_std(1), pos_std_floor_m(0));
    const double std_h_m = std::max(pos._blh_std(2), pos_std_floor_m(2));
    const double std_lat_rad = std_lat_m * eth._f_RMh;
    const double std_lon_rad = std_lon_m * eth._f_cbRNh;
    const Eigen::Vector3d pos_var(std_lat_rad * std_lat_rad,
                                  std_lon_rad * std_lon_rad,
                                  std_h_m * std_h_m);
    const int nx = layout.Dim();
    H = Eigen::MatrixXd::Zero(3, nx);
    // POST_MSF set_pos_meas：H_phi = Mpv [Cnb lever]x（此前参考实现遗漏此项）
    H.block(0, layout.Offset(BlockId::Rotation), 3, 3) = Mpv * askew(lever_n);
    H.block(0, layout.Offset(BlockId::Position), 3, 3) = Eigen::Matrix3d::Identity();
    if (layout.Has(BlockId::LeverArm)) {
        H.block(0, layout.Offset(BlockId::LeverArm), 3, 3) = Mpv * nominal._Cnb;
    }
    R = pos_var.asDiagonal();
    z = pos_ins_L - blh_gnss;
    return true;
}

Eigen::Vector3d RefVelStd(const GnssVel& vel, const GlobalConfig& config) {
    Eigen::Vector3d vel_std = config._statistics._meas._gnss_vel_meas_std_mps;
    const double factor = (vel._hor_speed < 1.0) ? 3.0 : 1.0;
    if (config._quality_control._gnss_vel_u_only_hor_speed_mps > 0.0 &&
        vel._hor_speed < config._quality_control._gnss_vel_u_only_hor_speed_mps) {
        vel_std(0) = vel_std(1) = 10.0;
        return vel_std;
    }
    return std::sqrt(factor) * vel_std;
}

bool RefVelMeas(const NominalState& nominal, earth& eth, const StateLayout& layout,
                const GnssVel& vel, const GlobalConfig& config,
                Eigen::MatrixXd& H, Eigen::MatrixXd& R, Eigen::VectorXd& z) {
    const double dt = nominal._t_cur - vel._timestamp;
    if (std::fabs(dt) > 0.03) {
        return false;
    }
    eth.Update(nominal._pos, nominal._vn);
    const Eigen::Vector3d web = nominal._wib - nominal._Cbn * eth._wnie;
    const Eigen::Vector3d lever_n = nominal._Cnb * nominal._lever;
    const Eigen::Vector3d wib_cross_lever_n =
        nominal._Cnb * (nominal._wib.cross(nominal._lever));
    const Eigen::Vector3d vn_ins_L =
        nominal._vn + nominal._Cnb * askew(web) * nominal._lever;
    const Eigen::Vector3d vn_gnss(vel._hor_speed * std::sin(vel._trk_gnd),
                                  vel._hor_speed * std::cos(vel._trk_gnd),
                                  vel._ver_speed);
    const Eigen::Vector3d vel_std = RefVelStd(vel, config);
    const int nx = layout.Dim();
    H = Eigen::MatrixXd::Zero(3, nx);
    H.block(0, layout.Offset(BlockId::Rotation), 3, 3) =
        askew(wib_cross_lever_n) - askew(eth._wnie) * askew(lever_n);
    H.block(0, layout.Offset(BlockId::Velocity), 3, 3) = Eigen::Matrix3d::Identity();
    H.block(0, layout.Offset(BlockId::GyroBias), 3, 3) = -nominal._Cnb * askew(nominal._lever);
    if (layout.Has(BlockId::LeverArm)) {
        H.block(0, layout.Offset(BlockId::LeverArm), 3, 3) = nominal._Cnb * askew(web);
    }
    R = vel_std.array().square().matrix().asDiagonal();
    z = vn_ins_L - vn_gnss;
    return true;
}

bool RefHeadingMeas(const NominalState& nominal, const StateLayout& layout,
                    const HeadingData& heading,
                    const GlobalConfig& config,
                    Eigen::MatrixXd& H, Eigen::MatrixXd& R, Eigen::VectorXd& z) {
    const double dt = nominal._t_cur - heading._timestamp;
    if (std::fabs(dt) > 0.03) {
        return false;
    }
    const double heading_std_max_rad =
        config._quality_control._heading_meas_std_max_deg * constants::_deg;
    if (heading._heading_std > heading_std_max_rad) {
        return false;
    }
    const Eigen::Vector3d baseline_b(heading._baseline_length, 0.0, 0.0);
    const Eigen::Vector3d baseline_n = nominal._Cnb * baseline_b;
    const double l_e = baseline_n(0);
    const double l_n = baseline_n(1);
    const double rho2 = l_e * l_e + l_n * l_n;
    if (rho2 < 1e-8) {
        return false;
    }
    const double heading_pred = std::atan2(l_e, l_n);
    const double heading_res = pose_converter::WrapAngle(heading._heading - heading_pred);
    Eigen::Matrix<double, 1, 3> d_heading_d_l;
    d_heading_d_l << l_n / rho2, -l_e / rho2, 0.0;
    const Eigen::Matrix<double, 1, 3> h_heading_phi =
        d_heading_d_l * (-askew(baseline_n));
    const int nx = layout.Dim();
    H = Eigen::MatrixXd::Zero(1, nx);
    H.block(0, layout.Offset(BlockId::Rotation), 1, 3) = h_heading_phi;
    R = Eigen::MatrixXd::Zero(1, 1);
    const double heading_std_floor_rad =
        config._statistics._meas._heading_meas_std_floor_deg * constants::_deg;
    const double heading_std_used =
        std::max(heading._heading_std * config._statistics._meas._heading_meas_std_scale,
                 heading_std_floor_rad);
    R(0, 0) = heading_std_used * heading_std_used;
    z = Eigen::VectorXd::Zero(1);
    z(0) = heading_res;
    return true;
}

// ===== 测试工具 =====

bool MatNear(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double tol = 1e-12) {
    if (a.rows() != b.rows() || a.cols() != b.cols()) {
        return false;
    }
    return (a - b).cwiseAbs().maxCoeff() <=
           tol * std::max(1.0, a.cwiseAbs().maxCoeff());
}

bool VecNear(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double tol = 1e-12) {
    if (a.size() != b.size()) {
        return false;
    }
    return (a - b).cwiseAbs().maxCoeff() <=
           tol * std::max(1.0, a.cwiseAbs().maxCoeff());
}

EkfState MakeState() {
    EkfState state(15);
    state._nominal._pos = Eigen::Vector3d(0.52, 1.92, 686.0);
    state._nominal._vn = Eigen::Vector3d(1.0, -0.5, 0.1);
    state._nominal._att = Eigen::Vector3d(0.01, -0.02, 1.2);
    state._nominal._Cnb = pose_converter::a2mat(state._nominal._att);
    state._nominal._Cbn = state._nominal._Cnb.transpose();
    state._nominal._qnb = pose_converter::a2qua(state._nominal._att);
    state._nominal._lever = Eigen::Vector3d(0.1, -0.2, 0.3);
    state._nominal._wib = Eigen::Vector3d(0.01, -0.02, 0.03);
    state._nominal._wnb = Eigen::Vector3d(0.001, 0.002, 0.003);
    state._nominal._t_cur = 100.0;
    state._eth.Update(state._nominal._pos, state._nominal._vn);
    return state;
}

bool TestPosFactorMatchesRef() {
    GnssPos pos;
    pos._timestamp = 100.0;
    pos._pos_type = 50;
    pos._blh = Eigen::Vector3d(0.52, 1.92, 686.0);
    pos._blh_std = Eigen::Vector3d(0.05, 0.05, 0.05);
    const GlobalConfig config;

    EkfState a = MakeState();
    EkfState b = MakeState();
    gnss::GnssPosFactor factor(pos, config);
    const auto block = factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{});

    Eigen::MatrixXd H, R;
    Eigen::VectorXd z;
    if (!RefPosMeas(b._nominal, b._eth, b._layout, pos, config, H, R, z)) {
        return false;
    }
    return block._valid && MatNear(block._H, H) && VecNear(block._R_diag, R.diagonal()) &&
           MatNear(block._z, z);
}

bool TestVelFactorMatchesRef() {
    GnssVel vel;
    vel._timestamp = 100.0;
    vel._vel_type = 50;
    vel._hor_speed = 5.0;
    vel._trk_gnd = 1.0;
    vel._ver_speed = 0.1;
    const GlobalConfig config;

    EkfState a = MakeState();
    EkfState b = MakeState();
    gnss::GnssVelFactor factor(vel, config);
    const auto block = factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{});

    Eigen::MatrixXd H, R;
    Eigen::VectorXd z;
    if (!RefVelMeas(b._nominal, b._eth, b._layout, vel, config, H, R, z)) {
        return false;
    }
    return block._valid && MatNear(block._H, H) && VecNear(block._R_diag, R.diagonal()) &&
           MatNear(block._z, z);
}

bool TestHeadingFactorMatchesRef() {
    HeadingData heading;
    heading._timestamp = 100.0;
    heading._pos_type = 50;
    heading._baseline_length = 2.0;
    heading._heading = 1.3;
    heading._heading_std = 0.02;
    const GlobalConfig config;

    EkfState a = MakeState();
    EkfState b = MakeState();
    gnss::HeadingFactor factor(heading, config);
    const auto block = factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{});

    Eigen::MatrixXd H, R;
    Eigen::VectorXd z;
    if (!RefHeadingMeas(b._nominal, b._layout, heading, config, H, R, z)) {
        return false;
    }
    return block._valid && MatNear(block._H, H) && VecNear(block._R_diag, R.diagonal()) &&
           MatNear(block._z, z);
}

bool TestHeadingStdMaxReject() {
    HeadingData heading;
    heading._timestamp = 100.0;
    heading._pos_type = 50;
    heading._baseline_length = 2.0;
    heading._heading = 1.3;
    heading._heading_std = 0.2;  // 11.5 deg > 默认 5 deg 上限
    const GlobalConfig config;

    EkfState a = MakeState();
    gnss::HeadingFactor factor(heading, config);
    return !factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{})._valid;
}

bool TestHeadingNoiseFloorAndScale() {
    GlobalConfig config;
    config._statistics._meas._heading_meas_std_scale = 0.5;
    config._statistics._meas._heading_meas_std_floor_deg = 0.3;

    HeadingData heading;
    heading._timestamp = 100.0;
    heading._pos_type = 50;
    heading._baseline_length = 2.0;
    heading._heading = 1.3;

    EkfState a = MakeState();

    // 缩放后仍高于地板：std=0.02 rad -> 使用 0.01 rad
    heading._heading_std = 0.02;
    {
        gnss::HeadingFactor factor(heading, config);
        const auto block = factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{});
        const double expected_std =
            heading._heading_std * config._statistics._meas._heading_meas_std_scale;
        if (!block._valid ||
            !VecNear(block._R_diag,
                     Eigen::VectorXd::Constant(1, expected_std * expected_std))) {
            return false;
        }
    }

    // 缩放后低于地板：std=0.002 rad -> 使用 0.3 deg
    heading._heading_std = 0.002;
    {
        gnss::HeadingFactor factor(heading, config);
        const auto block = factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{});
        const double expected_std =
            config._statistics._meas._heading_meas_std_floor_deg * constants::_deg;
        if (!block._valid ||
            !VecNear(block._R_diag,
                     Eigen::VectorXd::Constant(1, expected_std * expected_std))) {
            return false;
        }
    }

    return true;
}

bool TestTimeAlignAndQualityReject() {
    const GlobalConfig config;
    GnssPos pos;
    pos._timestamp = 100.5;  // 与 nominal._t_cur 差 0.5s
    pos._pos_type = 50;
    pos._blh = Eigen::Vector3d(0.52, 1.92, 686.0);
    pos._blh_std = Eigen::Vector3d::Ones();

    EkfState a = MakeState();
    gnss::GnssPosFactor factor(pos, config);
    if (factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{})._valid) {
        return false;
    }

    pos._timestamp = 100.0;
    pos._pos_type = 16;  // single point
    gnss::GnssPosFactor factor2(pos, config);
    return !factor2.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{})._valid;
}

bool TestVelLowSpeedStdScale() {
    GnssVel vel;
    vel._timestamp = 100.0;
    vel._vel_type = 50;
    vel._hor_speed = 0.5;  // <1.0 -> factor=3
    vel._trk_gnd = 1.0;
    vel._ver_speed = 0.0;
    const GlobalConfig config;

    EkfState a = MakeState();
    gnss::GnssVelFactor factor(vel, config);
    const auto block = factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{});
    if (!block._valid) {
        return false;
    }
    const Eigen::Vector3d expected_std =
        std::sqrt(3.0) * config._statistics._meas._gnss_vel_meas_std_mps;
    return VecNear(block._R_diag, expected_std.array().square().matrix(), 1e-12);
}

bool TestMeasUpdateBlockEndToEnd() {
    Ekf ekf(15);
    EkfState& s = ekf.State();
    s._nominal = MakeState()._nominal;
    s._eth.Update(s._nominal._pos, s._nominal._vn);
    s._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1e-4;
    s._error._Xk = Eigen::VectorXd::Zero(15);

    GnssPos pos;
    pos._timestamp = 100.0;
    pos._pos_type = 50;
    pos._blh = Eigen::Vector3d(0.52, 1.92, 686.0);
    pos._blh_std = Eigen::Vector3d(0.05, 0.05, 0.05);
    const GlobalConfig config;
    gnss::GnssPosFactor factor(pos, config);
    const auto block = factor.BuildMeasBlock(s._nominal, s._eth, s._layout, IterationContext{});

    const double trace_before = s._error._Pk.trace();
    if (!ekf.Update(factor, 1)) {
        return false;
    }
    return block._valid && s._error._Pk.trace() < trace_before &&
           s._error._Xk.norm() == 0.0;
}

bool TestPosAttitudeJacobianPresent() {
    GnssPos pos;
    pos._timestamp = 100.0;
    pos._pos_type = 50;
    pos._blh = Eigen::Vector3d(0.52, 1.92, 686.0);
    pos._blh_std = Eigen::Vector3d(0.05, 0.05, 0.05);
    const GlobalConfig config;

    EkfState a = MakeState();
    gnss::GnssPosFactor factor(pos, config);
    const auto block = factor.BuildMeasBlock(a._nominal, a._eth, a._layout, IterationContext{});
    if (!block._valid) {
        return false;
    }
    const Eigen::Matrix3d H_phi =
        block._H.block(0, a._layout.Offset(BlockId::Rotation), 3, 3);
    // 非零杆臂时姿态块必须非零，防止再出现“参考实现与因子同时遗漏”的假通过。
    return H_phi.cwiseAbs().maxCoeff() > 0.0;
}

bool TestGnssPosNormalizedResidualGate() {
    Ekf ekf(15);
    EkfState& s = ekf.State();
    s._nominal = MakeState()._nominal;
    s._eth.Update(s._nominal._pos, s._nominal._vn);
    s._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1e-8;
    s._error._Xk = Eigen::VectorXd::Zero(15);

    GnssPos pos;
    pos._timestamp = 100.0;
    pos._pos_type = 50;
    pos._blh = s._nominal._pos + Eigen::Vector3d(1.0e-3, 1.0e-3, 50.0);
    pos._blh_std = Eigen::Vector3d(0.05, 0.05, 0.05);
    const GlobalConfig config;
    gnss::GnssPosFactor factor(pos, config);

    const NominalState before = s._nominal;
    const Eigen::MatrixXd P_before = s._error._Pk;
    if (ekf.Update(factor, 1, constants::_iekf_convergence_threshold,
                   constants::_gnss_norm_res_gate)) {
        return false;
    }
    return s._nominal._pos == before._pos && s._error._Pk.isApprox(P_before, 0.0) &&
           s._error._norm_res.size() == 3 &&
           s._error._norm_res.cwiseAbs().maxCoeff() > constants::_gnss_norm_res_gate;
}

bool TestGnssVelNormalizedResidualGate() {
    const GlobalConfig config;
    {
        Ekf ekf(15);
        EkfState& s = ekf.State();
        s._nominal = MakeState()._nominal;
        s._eth.Update(s._nominal._pos, s._nominal._vn);
        s._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1e-8;
        s._error._Xk = Eigen::VectorXd::Zero(15);

        GnssVel vel;
        vel._timestamp = 100.0;
        vel._vel_type = 50;
        vel._hor_speed = 100.0;  // 异常速度
        vel._trk_gnd = 1.0;
        vel._ver_speed = 0.0;
        gnss::GnssVelFactor factor(vel, config);

        const NominalState before = s._nominal;
        const Eigen::MatrixXd P_before = s._error._Pk;
        if (ekf.Update(factor, 1, constants::_iekf_convergence_threshold,
                       constants::_gnss_norm_res_gate)) {
            return false;
        }
        if (s._nominal._vn != before._vn || !s._error._Pk.isApprox(P_before, 0.0) ||
            s._error._norm_res.size() != 3 ||
            s._error._norm_res.cwiseAbs().maxCoeff() <= constants::_gnss_norm_res_gate) {
            return false;
        }
    }

    {
        Ekf ekf(15);
        EkfState& s = ekf.State();
        s._nominal = MakeState()._nominal;
        s._eth.Update(s._nominal._pos, s._nominal._vn);
        s._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1e-8;
        s._error._Xk = Eigen::VectorXd::Zero(15);

        const Eigen::Vector3d web = s._nominal._wib - s._nominal._Cbn * s._eth._wnie;
        const Eigen::Vector3d vn_ins_L =
            s._nominal._vn + s._nominal._Cnb * askew(web) * s._nominal._lever;
        GnssVel vel;
        vel._timestamp = 100.0;
        vel._vel_type = 50;
        vel._hor_speed = std::sqrt(vn_ins_L(0) * vn_ins_L(0) + vn_ins_L(1) * vn_ins_L(1));
        vel._trk_gnd = std::atan2(vn_ins_L(0), vn_ins_L(1));
        vel._ver_speed = vn_ins_L(2);
        gnss::GnssVelFactor factor(vel, config);

        if (!ekf.Update(factor, 1, constants::_iekf_convergence_threshold,
                        constants::_gnss_norm_res_gate)) {
            return false;
        }
    }
    return true;
}

bool TestZuptNormalizedResidualGate() {
    Ekf ekf(15);
    EkfState& s = ekf.State();
    s._nominal = MakeState()._nominal;
    s._eth.Update(s._nominal._pos, s._nominal._vn);
    s._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1e-8;
    s._error._Xk = Eigen::VectorXd::Zero(15);

    const GlobalConfig config;
    algorithms::ZeroVelocityFactor factor(config._quality_control._zupt);

    const NominalState before = s._nominal;
    const Eigen::MatrixXd P_before = s._error._Pk;
    if (ekf.Update(factor, 1, constants::_iekf_convergence_threshold,
                   constants::_gnss_norm_res_gate)) {
        return false;
    }
    return s._nominal._vn == before._vn && s._error._Pk.isApprox(P_before, 0.0) &&
           s._error._norm_res.size() == 3 &&
           s._error._norm_res.cwiseAbs().maxCoeff() > constants::_gnss_norm_res_gate;
}

}  // namespace
}  // namespace msf

using namespace msf;

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"pos_factor_matches_ref", TestPosFactorMatchesRef},
        {"vel_factor_matches_ref", TestVelFactorMatchesRef},
        {"heading_factor_matches_ref", TestHeadingFactorMatchesRef},
        {"heading_std_max_reject", TestHeadingStdMaxReject},
        {"heading_noise_floor_and_scale", TestHeadingNoiseFloorAndScale},
        {"time_align_and_quality_reject", TestTimeAlignAndQualityReject},
        {"vel_low_speed_std_scale", TestVelLowSpeedStdScale},
        {"meas_update_block_end_to_end", TestMeasUpdateBlockEndToEnd},
        {"pos_attitude_jacobian_present", TestPosAttitudeJacobianPresent},
        {"gnss_pos_normalized_residual_gate", TestGnssPosNormalizedResidualGate},
        {"gnss_vel_normalized_residual_gate", TestGnssVelNormalizedResidualGate},
        {"zupt_normalized_residual_gate", TestZuptNormalizedResidualGate},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (!test.fn()) {
            std::cerr << "[FAIL] " << test.name << std::endl;
            ++failed;
        } else {
            std::cout << "[PASS] " << test.name << std::endl;
        }
    }

    return failed == 0 ? 0 : 1;
}
