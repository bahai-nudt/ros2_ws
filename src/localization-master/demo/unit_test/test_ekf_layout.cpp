#include <cmath>
#include <iostream>

#include "ekf.h"
#include "ekf_state.h"
#include "math/constants.h"
#include "math/pose_converter.h"
#include "math/utility.h"

namespace msf {
namespace {

// ===== 参考实现：旧模板版（仅用于数值对比验证）=====
template <int NX>
void BuildFtFixed(const NominalState& nominal, const earth& eth,
                  Eigen::Matrix<double, NX, NX>& Ft_out) {
    Ft_out.setZero();
    const Eigen::Vector3d& vn = eth._vn;
    const Eigen::Vector3d& wnie = eth._wnie;
    const Eigen::Vector3d& wnen = eth._wnen;
    const Eigen::Vector3d& wnin = eth._wnin;
    const double tl = eth._tb;
    const double secl = 1.0 / eth._cb;
    const double f_RMh = eth._f_RMh, f_RNh = eth._f_RNh, f_clRNh = eth._f_cbRNh;
    const double f_RMh2 = f_RMh * f_RMh, f_RNh2 = f_RNh * f_RNh;
    const double vE_clRNh = vn(0) * f_clRNh;
    const double vE_RNh2 = vn(0) * f_RNh2;
    const double vN_RMh2 = vn(1) * f_RMh2;
    const Eigen::Vector3d wnien = 2.0 * wnie + wnen;

    Eigen::Matrix3d Mp1 = Eigen::Matrix3d::Zero();
    Mp1(1, 0) = -wnie(2);
    Mp1(2, 0) = wnie(1);
    Eigen::Matrix3d Mp2 = Eigen::Matrix3d::Zero();
    Mp2(2, 0) = vE_clRNh * secl;
    Mp2(0, 2) = vN_RMh2;
    Mp2(1, 2) = -vE_RNh2;
    Mp2(2, 2) = -vE_RNh2 * tl;
    Eigen::Matrix3d Map = Mp1 + Mp2;

    Eigen::Matrix3d Mav;
    Mav << 0.0, -f_RMh, 0.0,
           f_RNh, 0.0, 0.0,
           f_RNh * tl, 0.0, 0.0;

    Eigen::Matrix3d Maa = -askew(wnin);
    Eigen::Matrix3d Mva = askew(nominal._fn);
    Eigen::Matrix3d Avn = askew(vn);
    Eigen::Matrix3d Awn = askew(wnien);
    Eigen::Matrix3d Mvv = Avn * Mav - Awn;

    Eigen::Matrix3d Mvp = Avn * (Mp1 + Map);
    const double g0 = constants::_g0;
    const double scl = eth._sb * eth._cb;
    Mvp(2, 0) -= g0 * (5.2790414e-3 * 2.0 + 2.32718e-5 * 4.0 * eth._sb2) * scl;
    Mvp(2, 2) += 3.086e-6;

    Eigen::Matrix3d Mpv;
    Mpv << 0.0, f_RMh, 0.0,
           f_clRNh, 0.0, 0.0,
           0.0, 0.0, 1.0;

    Eigen::Matrix3d Mpp = Eigen::Matrix3d::Zero();
    Mpp(1, 0) = vE_clRNh * tl;
    Mpp(0, 2) = -vN_RMh2;
    Mpp(1, 2) = -vE_RNh2 * secl;

    Eigen::Matrix3d O33 = Eigen::Matrix3d::Zero();
    Ft_out.template block<3, 3>(0, 0) = Maa;
    Ft_out.template block<3, 3>(0, 3) = Mav;
    Ft_out.template block<3, 3>(0, 6) = Map;
    Ft_out.template block<3, 3>(0, 9) = -nominal._Cnb;
    Ft_out.template block<3, 3>(0, 12) = O33;

    Ft_out.template block<3, 3>(3, 0) = Mva;
    Ft_out.template block<3, 3>(3, 3) = Mvv;
    Ft_out.template block<3, 3>(3, 6) = Mvp;
    Ft_out.template block<3, 3>(3, 9) = O33;
    Ft_out.template block<3, 3>(3, 12) = nominal._Cnb;

    Ft_out.template block<3, 3>(6, 0) = O33;
    Ft_out.template block<3, 3>(6, 3) = Mpv;
    Ft_out.template block<3, 3>(6, 6) = Mpp;
    Ft_out.template block<3, 3>(6, 9) = O33;
    Ft_out.template block<3, 3>(6, 12) = O33;

    Ft_out.template block<3, 3>(9, 9) = O33;
    Ft_out.template block<3, 3>(9, 12) = O33;
    Ft_out.template block<3, 3>(12, 9) = O33;
    Ft_out.template block<3, 3>(12, 12) = O33;
    if constexpr (NX > 15) {
        Ft_out.template block<3, 3>(15, 15) = O33;
    }
}

bool PredictReference(ErrorState& error, const Eigen::MatrixXd& Fk,
                      const Eigen::MatrixXd& Qk) {
    error._Xk = Fk * error._Xk;
    error._Pk = Fk * error._Pk * Fk.transpose() + Qk;
    error._Pk = (error._Pk + error._Pk.transpose()) / 2.0;
    return true;
}

template <int NX>
bool TimeUpdateFixed(const NominalState& nominal, const earth& eth, ErrorState& error,
                     double dt, double inflation) {
    constexpr int NQ = NX - 3;

    Eigen::Matrix<double, NX, NX> Ft = Eigen::Matrix<double, NX, NX>::Zero();
    BuildFtFixed<NX>(nominal, eth, Ft);
    const Eigen::Matrix<double, NX, NX> Fk =
        Eigen::Matrix<double, NX, NX>::Identity() + Ft * dt;

    Eigen::Matrix<double, NQ, 1> qc;
    qc.template segment<3>(0) = error._Qt.segment<3>(0);
    qc.template segment<3>(3) = error._Qt.segment<3>(3);
    qc.template segment<3>(6) = error._Qt.segment<3>(9);
    qc.template segment<3>(9) = error._Qt.segment<3>(12);
    if constexpr (NX > 15) {
        qc.template segment<3>(12) = error._Qt.segment<3>(15);
    }

    Eigen::Matrix<double, NX, NQ> G = Eigen::Matrix<double, NX, NQ>::Zero();
    G.template block<3, 3>(0, 0) = -nominal._Cnb;
    G.template block<3, 3>(3, 3) = nominal._Cnb;
    G.template block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    G.template block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();
    if constexpr (NX > 15) {
        G.template block<3, 3>(15, 12) = Eigen::Matrix3d::Identity();
    }

    const Eigen::Matrix<double, NX, NX> GqG = G * qc.asDiagonal() * G.transpose();
    const Eigen::Matrix<double, NX, NX> Qk =
        0.5 * (Fk * GqG * Fk.transpose() + GqG) * dt * inflation;

    // 传播统一走通用代数内核：模型层只负责组装 Fk/Qk
    return PredictReference(error, Fk, Qk);
}


bool MatNear(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double tol = 1e-12) {
    if (a.rows() != b.rows() || a.cols() != b.cols()) {
        return false;
    }
    return (a - b).cwiseAbs().maxCoeff() <=
           tol * std::max(1.0, a.cwiseAbs().maxCoeff());
}

void FillTestState(EkfState& state) {
    const Eigen::Vector3d pos0(0.52, 1.92, 686.0);
    state._nominal._pos = pos0;
    state._nominal._vn = Eigen::Vector3d(1.0, -0.5, 0.1);
    state._nominal._att = Eigen::Vector3d(0.01, -0.02, 1.2);
    state._nominal._Cnb = pose_converter::a2mat(state._nominal._att);
    state._nominal._Cbn = state._nominal._Cnb.transpose();
    state._nominal._qnb = pose_converter::a2qua(state._nominal._att);
    state._eth.Update(state._nominal._pos, state._nominal._vn);

    const int nx = state._layout.Dim();
    state._error._Pk = Eigen::MatrixXd::Identity(nx, nx) * 1e-4;
    state._error._Xk = Eigen::VectorXd::Ones(nx) * 0.01;

    if (state._layout.Has(BlockId::Rotation)) {
        state._error._Qt.segment<3>(state._layout.Offset(BlockId::Rotation)) =
            (Eigen::Vector3d(2.5, 2.5, 2.0) * constants::_dpsh).array().square().matrix();
    }
    if (state._layout.Has(BlockId::Velocity)) {
        state._error._Qt.segment<3>(state._layout.Offset(BlockId::Velocity)) =
            (Eigen::Vector3d(408.0, 408.0, 408.0) * constants::_ugpsHz).array().square().matrix();
    }
    if (state._layout.Has(BlockId::GyroBias)) {
        state._error._Qt.segment<3>(state._layout.Offset(BlockId::GyroBias)) =
            (Eigen::Vector3d(108.0, 108.0, 108.0) * constants::_dphpsh).array().square().matrix();
    }
    if (state._layout.Has(BlockId::AccBias)) {
        state._error._Qt.segment<3>(state._layout.Offset(BlockId::AccBias)) =
            (Eigen::Vector3d(1050.0, 1050.0, 1050.0) * constants::_ugpsh).array().square().matrix();
    }
    if (state._layout.Has(BlockId::LeverArm)) {
        state._error._Qt.segment<3>(state._layout.Offset(BlockId::LeverArm)) =
            Eigen::Vector3d::Constant(1.0e-6);
    }
}

template <int NX>
bool CompareTimeUpdate(double dt) {
    Ekf dyn(NX);
    FillTestState(dyn.State());
    EkfState ref(NX);
    FillTestState(ref);

    if (!dyn.TimeUpdate(dt, 1.0)) {
        return false;
    }
    if (!TimeUpdateFixed<NX>(ref._nominal, ref._eth, ref._error, dt, 1.0)) {
        return false;
    }
    return MatNear(dyn.State()._error._Xk, ref._error._Xk) &&
           MatNear(dyn.State()._error._Pk, ref._error._Pk);
}

bool TestTimeUpdate15() { return CompareTimeUpdate<15>(0.01); }
bool TestTimeUpdate18() { return CompareTimeUpdate<18>(0.01); }

bool TestCustomLayout9() {
    StateLayout layout;
    layout.Enable(BlockId::Rotation, 3);
    layout.Enable(BlockId::Velocity, 3);
    layout.Enable(BlockId::Position, 3);

    Ekf ekf(layout);
    FillTestState(ekf.State());

    const double dt = 0.01;
    return ekf.TimeUpdate(dt, 1.0) &&
           ekf.State()._error._Pk.allFinite() && ekf.State()._error._Xk.allFinite() &&
           ekf.State()._error._Pk.trace() > 0.0;
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
        {"time_update_15_matches_ref", TestTimeUpdate15},
        {"time_update_18_matches_ref", TestTimeUpdate18},
        {"custom_layout_9_sanity", TestCustomLayout9},
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
