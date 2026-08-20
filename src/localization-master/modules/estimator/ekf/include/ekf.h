#pragma once

#include <Eigen/Eigen>

#include "ekf_state.h"
#include "interfaces/meas_factor.h"
#include "interfaces/optimizer.h"
#include "math/constants.h"

namespace msf {

struct GlobalConfig;

/** IEKF 诊断（对齐 POST_MSF IekfDiag，供 log.txt）。 */
struct IekfDiag {
    int total_iter = 0;
    int converge_iter = -1;
    bool final_converged = false;
    double max_dx_last = 0.0;
};

/**
 * @brief EKF：持有唯一状态源（EkfState），并直接实现状态更新流程。
 *
 * 时间更新 / 量测更新 / 误差反馈均作为成员函数操作内部状态，
 * 不再通过自由函数转发；底层矩阵运算保留为私有成员。
 */
class Ekf : public Optimizer {
public:
    explicit Ekf(const StateLayout& layout);
    explicit Ekf(int state_dim);

    /// 注入滤波起点：名义状态与地球模型。
    void SetInitialState(const NominalState& nominal, const earth& eth);

    /// 从 GlobalConfig 初始化 P0 / Qt（对应 POST_MSF init_ekf）。
    void InitNoiseFromConfig(const GlobalConfig& config);

    /// 时间更新：INS 误差动力学传播 _Xk/_Pk（dt 为积分步长）。
    bool TimeUpdate(double dt, double inflation = 1.0) override;

    /**
     * 统一量测更新入口：EKF / IEKF 共用一套代码。
     * max_iterations 只是迭代次数（默认 1 = 标准 EKF）：每轮在“被修正过的状态”处
     * 重新线性化（factor 依据 iteration 决定数据关联），先验约束 + 局部修正，
     * 收敛或到达最大迭代后更新协方差、清零误差状态并调用 factor.Commit()。
     * convergence_threshold <= 0 时回退到 constants::_iekf_convergence_threshold。
     * normalized_residual_limit > 0 时，在首轮线性化后按
     * |r_i|/sqrt(S_ii) 任一分量超过 limit 即拒绝整次更新（复现 POST_MSF 3σ 门控）。
     */
    bool Update(MeasFactor& factor, int max_iterations = 1,
                double convergence_threshold = constants::_iekf_convergence_threshold,
                double normalized_residual_limit = 0.0);

    /// 量测残差预检（门控/抗差），不修改滤波状态。
    void CalMeasResidual();

    /// 误差反馈：将 _Xk 修正到名义状态并清零 _Xk。
    void StateFeedback();

    EkfState& State();
    const EkfState& State() const;

    NominalState& Nominal() override { return state_._nominal; }
    earth& Earth() override { return state_._eth; }

    const IekfDiag& LastIekfDiag() const { return iekf_diag_; }

private:
    bool Predict(const Eigen::MatrixXd& Fk, const Eigen::MatrixXd& Qk);
    void BuildFt(Eigen::MatrixXd& Ft) const;
    void ApplyErrorToNominal(const Eigen::VectorXd& dx);

    EkfState state_;
    IekfDiag iekf_diag_;
};

}  // namespace msf
