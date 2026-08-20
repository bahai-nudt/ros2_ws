#include "ekf.h"

#include <algorithm>
#include <cmath>

#include "config/global_config.h"
#include "math/constants.h"
#include "math/pose_converter.h"
#include "math/t_quat.h"
#include "math/utility.h"

namespace msf {
namespace {

/// R 对角方差下限：只挡数值噪声/零方差；GNSS lat/lon 方差约 1e-17，远大于此。
constexpr double kMinRDiag = 1.0e-150;

void Symmetry(Eigen::MatrixXd& m) { m = (m + m.transpose()) / 2.0; }

// 当前状态相对先验的误差（IEKF 先验约束项；对应 POST_MSF LidarBoxMinus）
Eigen::VectorXd BoxMinus(const NominalState& current, const NominalState& prior,
                         const StateLayout& layout) {
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(layout.Dim());
    const t_quat dq = current._qnb * t_quat::conj(prior._qnb);
    dx.segment<3>(layout.Offset(BlockId::Rotation)) = pose_converter::q2rv(dq);
    dx.segment<3>(layout.Offset(BlockId::Velocity)) = prior._vn - current._vn;
    dx.segment<3>(layout.Offset(BlockId::Position)) = prior._pos - current._pos;
    dx.segment<3>(layout.Offset(BlockId::GyroBias)) = current._eb - prior._eb;
    dx.segment<3>(layout.Offset(BlockId::AccBias)) = current._db - prior._db;
    if (layout.Has(BlockId::LeverArm)) {
        dx.segment<3>(layout.Offset(BlockId::LeverArm)) = prior._lever - current._lever;
    }
    return dx;
}

bool ValidPredictInputs(const ErrorState& error,
                        const Eigen::MatrixXd& Fk,
                        const Eigen::MatrixXd& Qk) {
    if (error._nx <= 0) {
        return false;
    }
    if (error._Xk.size() != error._nx || error._Pk.rows() != error._nx ||
        error._Pk.cols() != error._nx || Fk.rows() != error._nx || Fk.cols() != error._nx ||
        Qk.rows() != error._nx || Qk.cols() != error._nx) {
        return false;
    }
    return error._Xk.allFinite() && error._Pk.allFinite() && Fk.allFinite() && Qk.allFinite();
}

}  // namespace

Ekf::Ekf(const StateLayout& layout) { state_.Init(layout); }
Ekf::Ekf(int state_dim) { state_.Init(state_dim); }

EkfState& Ekf::State() { return state_; }
const EkfState& Ekf::State() const { return state_; }

void Ekf::SetInitialState(const NominalState& nominal, const earth& eth) {
    state_._nominal = nominal;
    state_._eth = eth;
}

void Ekf::InitNoiseFromConfig(const GlobalConfig& config) {
    const StateLayout& layout = state_._layout;
    ErrorState& error = state_._error;
    const int nx = layout.Dim();

    Eigen::VectorXd std_init = Eigen::VectorXd::Zero(nx);
    std_init.segment<3>(layout.Offset(BlockId::Rotation)) = config._statistics._init._att_err_deg * constants::_deg;
    std_init.segment<3>(layout.Offset(BlockId::Velocity)) = config._statistics._init._vel_err_mps;
    std_init.segment<3>(layout.Offset(BlockId::Position)) = config._statistics._init._pos_err_m;
    std_init(layout.Offset(BlockId::Position)) /= constants::_Re;
    std_init(layout.Offset(BlockId::Position) + 1) /= constants::_Re;
    std_init.segment<3>(layout.Offset(BlockId::GyroBias)) = config._statistics._init._gyro_bias_err_dps * constants::_dps;
    std_init.segment<3>(layout.Offset(BlockId::AccBias)) = config._statistics._init._acc_bias_err_mg * constants::_mg;
    if (layout.Has(BlockId::LeverArm)) {
        std_init.segment<3>(layout.Offset(BlockId::LeverArm)) = config._statistics._init._lever_arm_err_m;
    }
    error._Pk = std_init.array().square().matrix().asDiagonal();

    error._Qt = Eigen::VectorXd::Zero(nx);
    error._Qt.segment<3>(layout.Offset(BlockId::Rotation)) = (config._statistics._process._ang_random_walk * constants::_dpsh).array().square().matrix();
    error._Qt.segment<3>(layout.Offset(BlockId::Velocity)) = (config._statistics._process._vel_random_walk * constants::_ugpsHz).array().square().matrix();
    error._Qt.segment<3>(layout.Offset(BlockId::GyroBias)) = (config._statistics._process._gyro_bias_noise * constants::_dphpsh).array().square().matrix();
    error._Qt.segment<3>(layout.Offset(BlockId::AccBias)) = (config._statistics._process._acc_bias_noise * constants::_ugpsh).array().square().matrix();
    if (layout.Has(BlockId::LeverArm)) {
        error._Qt.segment<3>(layout.Offset(BlockId::LeverArm)) = config._statistics._process._lever_random_walk.array().square().matrix();
    }
}

bool Ekf::Predict(const Eigen::MatrixXd& Fk, const Eigen::MatrixXd& Qk) {
    ErrorState& error = state_._error;
    if (!ValidPredictInputs(error, Fk, Qk)) {
        return false;
    }

    // x_k|k-1 = F_k * x_k-1|k-1
    error._Xk = Fk * error._Xk;
    // P_k|k-1 = F_k * P_k-1|k-1 * F_k^T + Q_k
    error._Pk = Fk * error._Pk * Fk.transpose() + Qk;
    Symmetry(error._Pk);
    return true;
}

bool Ekf::Update(MeasFactor& factor, int max_iterations,
                 double convergence_threshold,
                 double normalized_residual_limit) {
    ErrorState& error = state_._error;
    const int nx = state_._layout.Dim();

    // 先验：迭代在“当前状态”上局部修正，失败时恢复（RTKLIB 副本式语义）
    const NominalState prior_nominal = state_._nominal;
    const Eigen::MatrixXd P_prior = error._Pk;
    const int max_iter = std::max(1, max_iterations);
    const double threshold = convergence_threshold > 0.0
        ? convergence_threshold : constants::_iekf_convergence_threshold;

    if (nx <= 0 || !P_prior.allFinite()) {
        return false;
    }

    // P_inv 在循环外只算一次：P_prior 全程不变，避免每轮重复 LDLT。
    Eigen::LDLT<Eigen::MatrixXd> p_ldlt(P_prior);
    if (!p_ldlt.isPositive()) {
        return false;
    }
    const Eigen::MatrixXd P_inv =
        p_ldlt.solve(Eigen::MatrixXd::Identity(nx, nx));
    if (!P_inv.allFinite()) {
        return false;
    }

    bool converge = true;
    bool last_converged = false;
    bool updated = false;
    int converge_count = 0;
    int actual_iter = 0;
    int converge_at = -1;
    double max_dx_last = 0.0;
    Eigen::MatrixXd last_KH = Eigen::MatrixXd::Zero(nx, nx);
    iekf_diag_ = IekfDiag{};

    IterationContext ctx;
    for (int it = 0; it < max_iter; ++it) {
        // 每轮在“被修正过的状态”处重新线性化；迭代上下文交给因子决定数据关联
        ctx._iteration = it;
        ctx._last_converged = last_converged;
        const auto block = factor.BuildMeasBlock(state_._nominal, state_._eth,
                                                 state_._layout, ctx);
        if (!block._valid || block._H.rows() == 0 || block._H.cols() != nx ||
            block._z.size() != block._H.rows()) {
            // 参考语义：首轮无效直接失败；中间轮无效保留上一轮修正继续迭代，
            // 末了用上一轮的 last_KH 提交。
            if (!updated) {
                state_._nominal = prior_nominal;
                return false;
            }
            continue;
        }
        const bool full_r = block.HasFullR();
        if (!full_r && block._R_diag.size() != block._H.rows()) {
            if (!updated) {
                state_._nominal = prior_nominal;
                return false;
            }
            continue;
        }
        const Eigen::MatrixXd& H = block._H;
        const Eigen::VectorXd& h = block._z;

        // 先验约束：当前状态相对先验的误差
        const Eigen::VectorXd dx_new = BoxMinus(state_._nominal, prior_nominal, state_._layout);

        Eigen::LDLT<Eigen::MatrixXd> r_ldlt;
        bool r_ok = true;
        if (full_r) {
            r_ldlt.compute(block._R);
            r_ok = r_ldlt.isPositive() && block._R.allFinite();
        } else {
            for (int i = 0; i < block._R_diag.size(); ++i) {
                if (!(block._R_diag(i) > kMinRDiag)) {
                    r_ok = false;
                    break;
                }
            }
        }
        if (!r_ok) {
            if (!updated) {
                state_._nominal = prior_nominal;
                return false;
            }
            continue;
        }

        // POST_MSF：set_*_meas 后若 |r_i|/sqrt(S_ii) 任一分量 > 3 则拒绝整次 GNSS 更新。
        if (normalized_residual_limit > 0.0 && it == 0) {
            error._nr = static_cast<int>(h.size());
            error._Hk = H;
            error._Rk = full_r ? block._R : Eigen::MatrixXd(block._R_diag.asDiagonal());
            error._Zk = h;
            CalMeasResidual();
            if (error._norm_res.size() > 0 &&
                error._norm_res.cwiseAbs().maxCoeff() > normalized_residual_limit) {
                state_._nominal = prior_nominal;
                return false;
            }
        }

        const double r0 = full_r ? 0.0 : block._R_diag(0);
        bool scalar_r = !full_r && r0 > kMinRDiag;
        for (int i = 1; scalar_r && i < block._R_diag.size(); ++i) {
            scalar_r = block._R_diag(i) == r0;
        }

        Eigen::MatrixXd HtRinv;
        Eigen::MatrixXd HtRinvH;
        if (full_r) {
            const Eigen::MatrixXd RinvH = r_ldlt.solve(H);
            HtRinv = RinvH.transpose();
            HtRinvH = HtRinv * H;
        } else if (scalar_r) {
            HtRinvH = (H.transpose() * H) / r0;
            HtRinv = H.transpose() / r0;
        } else {
            const Eigen::VectorXd r_inv = block._R_diag.cwiseInverse();
            HtRinv = H.transpose() * r_inv.asDiagonal();
            HtRinvH = HtRinv * H;
        }
        if (!HtRinv.allFinite() || !HtRinvH.allFinite()) {
            if (!updated) {
                state_._nominal = prior_nominal;
                return false;
            }
            continue;
        }

        Eigen::LDLT<Eigen::MatrixXd> info_ldlt(HtRinvH + P_inv);
        if (!info_ldlt.isPositive()) {
            return false;
        }
        const Eigen::MatrixXd K_front =
            info_ldlt.solve(Eigen::MatrixXd::Identity(nx, nx));
        if (!K_front.allFinite()) {
            return false;
        }
        Eigen::MatrixXd K;
        if (full_r || !scalar_r) {
            K = K_front * HtRinv;
        } else {
            // 与 POST_MSF 完全一致：K = K_front·Hᵀ / R
            K = (K_front * H.transpose()) / r0;
        }
        const Eigen::MatrixXd KH = K * H;

        const Eigen::VectorXd dx = K * h + (KH - Eigen::MatrixXd::Identity(nx, nx)) * dx_new;

        ApplyErrorToNominal(dx);
        last_KH = KH;
        updated = true;
        actual_iter = it + 1;
        max_dx_last = dx.cwiseAbs().maxCoeff();

        converge = true;
        if (max_dx_last > threshold) {
            converge = false;
        }
        // 倒数第二轮强制置收敛（对应 POST_MSF lidar_update.cpp:296-298）
        if (converge_count == 0 && it == max_iter - 2) {
            converge = true;
        }
        if (converge) {
            ++converge_count;
            if (converge_at < 0) {
                converge_at = actual_iter;
            }
        }
        last_converged = converge;
        // 连续两轮收敛或到达最大迭代即提交（max_iterations=1 时首轮即提交 = 标准 EKF）
        if (converge_count > 1 || it == max_iter - 1) {
            error._Pk = (Eigen::MatrixXd::Identity(nx, nx) - last_KH) * P_prior;
            Symmetry(error._Pk);
            error._Xk.setZero();
            factor.Commit(state_._nominal);
            iekf_diag_.total_iter = actual_iter;
            iekf_diag_.converge_iter = converge_at;
            iekf_diag_.final_converged = converge;
            iekf_diag_.max_dx_last = max_dx_last;
            return true;
        }
    }

    // 循环结束仍存在有效更新（例如末轮因子无效）：用上一轮 last_KH 提交。
    if (updated) {
        error._Pk = (Eigen::MatrixXd::Identity(nx, nx) - last_KH) * P_prior;
        Symmetry(error._Pk);
        error._Xk.setZero();
        factor.Commit(state_._nominal);
        iekf_diag_.total_iter = actual_iter;
        iekf_diag_.converge_iter = converge_at;
        iekf_diag_.final_converged = converge;
        iekf_diag_.max_dx_last = max_dx_last;
        return true;
    }

    // 失败：恢复先验状态，不污染
    state_._nominal = prior_nominal;
    return false;
}

void Ekf::CalMeasResidual() {
    ErrorState& error = state_._error;
    const int nr = error._nr;
    error._res = error._Zk - error._Hk * error._Xk;
    error._norm_res = Eigen::VectorXd::Zero(nr);

    error._Sk = error._Hk * error._Pk * error._Hk.transpose() + error._Rk;
    for (int i = 0; i < nr; ++i) {
        const double sigma = std::sqrt(std::max(error._Sk(i, i), 0.0));
        if (sigma > 1.0e-12) {
            error._norm_res(i) = error._res(i) / sigma;
        }
    }
}

void Ekf::BuildFt(Eigen::MatrixXd& Ft) const {
    const NominalState& nominal = state_._nominal;
    const earth& eth = state_._eth;
    const StateLayout& layout = state_._layout;
    Ft = Eigen::MatrixXd::Zero(layout.Dim(), layout.Dim());

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

    // 按块组装：块间耦合只依赖 layout 是否存在对应块；新块默认无动力学耦合（0）
    if (layout.Has(BlockId::Rotation)) {
        const int r = layout.Offset(BlockId::Rotation);
        Ft.block(r, r, 3, 3) = Maa;
        if (layout.Has(BlockId::Velocity)) {
            Ft.block(r, layout.Offset(BlockId::Velocity), 3, 3) = Mav;
        }
        if (layout.Has(BlockId::Position)) {
            Ft.block(r, layout.Offset(BlockId::Position), 3, 3) = Map;
        }
        if (layout.Has(BlockId::GyroBias)) {
            Ft.block(r, layout.Offset(BlockId::GyroBias), 3, 3) = -nominal._Cnb;
        }
    }
    if (layout.Has(BlockId::Velocity)) {
        const int v = layout.Offset(BlockId::Velocity);
        if (layout.Has(BlockId::Rotation)) {
            Ft.block(v, layout.Offset(BlockId::Rotation), 3, 3) = Mva;
        }
        Ft.block(v, v, 3, 3) = Mvv;
        if (layout.Has(BlockId::Position)) {
            Ft.block(v, layout.Offset(BlockId::Position), 3, 3) = Mvp;
        }
        if (layout.Has(BlockId::AccBias)) {
            Ft.block(v, layout.Offset(BlockId::AccBias), 3, 3) = nominal._Cnb;
        }
    }
    if (layout.Has(BlockId::Position)) {
        const int p = layout.Offset(BlockId::Position);
        if (layout.Has(BlockId::Velocity)) {
            Ft.block(p, layout.Offset(BlockId::Velocity), 3, 3) = Mpv;
        }
        Ft.block(p, p, 3, 3) = Mpp;
    }
}

bool Ekf::TimeUpdate(double dt, double inflation) {
    const NominalState& nominal = state_._nominal;
    const StateLayout& layout = state_._layout;
    ErrorState& error = state_._error;
    const int nx = layout.Dim();
    if (error._Qt.size() != nx) {
        return false;
    }

    Eigen::MatrixXd Ft;
    BuildFt(Ft);
    const Eigen::MatrixXd Fk = Eigen::MatrixXd::Identity(nx, nx) + Ft * dt;

    // 噪声输入矩阵 G：行 = 状态块偏移，列 = 有过程噪声的块（按块计数）
    int nq = 0;
    if (layout.Has(BlockId::Rotation)) {
        ++nq;
    }
    if (layout.Has(BlockId::Velocity)) {
        ++nq;
    }
    if (layout.Has(BlockId::GyroBias)) {
        ++nq;
    }
    if (layout.Has(BlockId::AccBias)) {
        ++nq;
    }
    if (layout.Has(BlockId::LeverArm)) {
        ++nq;
    }

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(nx, 3 * nq);
    Eigen::VectorXd qc = Eigen::VectorXd::Zero(3 * nq);
    int q = 0;
    if (layout.Has(BlockId::Rotation)) {
        const int off = layout.Offset(BlockId::Rotation);
        G.block(off, 3 * q, 3, 3) = -nominal._Cnb;
        qc.segment<3>(3 * q) = error._Qt.segment<3>(off);
        ++q;
    }
    if (layout.Has(BlockId::Velocity)) {
        const int off = layout.Offset(BlockId::Velocity);
        G.block(off, 3 * q, 3, 3) = nominal._Cnb;
        qc.segment<3>(3 * q) = error._Qt.segment<3>(off);
        ++q;
    }
    if (layout.Has(BlockId::GyroBias)) {
        const int off = layout.Offset(BlockId::GyroBias);
        G.block(off, 3 * q, 3, 3) = Eigen::Matrix3d::Identity();
        qc.segment<3>(3 * q) = error._Qt.segment<3>(off);
        ++q;
    }
    if (layout.Has(BlockId::AccBias)) {
        const int off = layout.Offset(BlockId::AccBias);
        G.block(off, 3 * q, 3, 3) = Eigen::Matrix3d::Identity();
        qc.segment<3>(3 * q) = error._Qt.segment<3>(off);
        ++q;
    }
    if (layout.Has(BlockId::LeverArm)) {
        const int off = layout.Offset(BlockId::LeverArm);
        G.block(off, 3 * q, 3, 3) = Eigen::Matrix3d::Identity();
        qc.segment<3>(3 * q) = error._Qt.segment<3>(off);
        ++q;
    }

    const Eigen::MatrixXd GqG = G * qc.asDiagonal() * G.transpose();
    const Eigen::MatrixXd Qk = 0.5 * (Fk * GqG * Fk.transpose() + GqG) * dt * inflation;

    // 传播统一走通用代数内核
    return Predict(Fk, Qk);
}

void Ekf::ApplyErrorToNominal(const Eigen::VectorXd& dx) {
    NominalState& nominal = state_._nominal;
    const StateLayout& layout = state_._layout;
    if (dx.size() != layout.Dim()) {
        return;
    }

    const Eigen::Vector3d phi = dx.segment<3>(layout.Offset(BlockId::Rotation));
    const Eigen::Vector3d dvn = dx.segment<3>(layout.Offset(BlockId::Velocity));
    const Eigen::Vector3d dpos = dx.segment<3>(layout.Offset(BlockId::Position));
    const Eigen::Vector3d deb = dx.segment<3>(layout.Offset(BlockId::GyroBias));
    const Eigen::Vector3d ddb = dx.segment<3>(layout.Offset(BlockId::AccBias));

    nominal._vn -= dvn;
    nominal._pos -= dpos;
    nominal._eb += deb;
    nominal._db += ddb;
    if (layout.Has(BlockId::LeverArm)) {
        const Eigen::Vector3d dlever = dx.segment<3>(layout.Offset(BlockId::LeverArm));
        nominal._lever -= dlever;
        nominal._lever = nominal._lever.array().max(-3.0).min(3.0).matrix();
    }

    nominal._qnb = pose_converter::rv2q(phi) * nominal._qnb;
    t_quat::normlize(nominal._qnb);
    nominal._Cnb = pose_converter::q2mat(nominal._qnb);
    nominal._Cbn = nominal._Cnb.transpose();
    nominal._att = pose_converter::m2att(nominal._Cnb);

    // 反馈后同步地球参数，避免下一轮/下一次时间推进使用旧位置的曲率半径。
    state_._eth.Update(nominal._pos, nominal._vn);
}

void Ekf::StateFeedback() {
    ApplyErrorToNominal(state_._error._Xk);
    state_._error._Xk.setZero();
}

}  // namespace msf
