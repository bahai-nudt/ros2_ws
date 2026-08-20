#pragma once

#include <Eigen/Eigen>

#include "math/earth.h"
#include "types/nav_state.h"
#include "types/state_layout.h"

namespace msf {

/**
 * =============================================================================
 * 名义状态 vs 误差状态
 * =============================================================================
 *
 *   NominalState  ：机械编排积分得到的当前导航解（定义见 core/nav_state.h）。
 *   ErrorState    ：误差状态 EKF 工作区；核心为 _Xk 与 _Pk。
 *   StateLayout   ：误差向量分块下标表。
 *   EkfState      ：名义状态 + 地球模型 + 误差状态 + 分块布局（唯一状态源）。
 *
 *   典型流程（误差状态反馈型）：
 *     1) IMU 驱动，在 NominalState 上机械编排；
 *     2) TimeUpdate 主要传播 _Pk（及过程噪声）；反馈后 _Xk 保持为 0；
 *     3) 量测更新（Ekf::Update）在名义状态处线性化，直接计算修正量并反馈，
 *        随后将 _Xk 置零；EKF 即 1 次迭代，IEKF 为多次迭代。
 *
 *   _Xk 分块（StateLayout / BlockId，15 或 18 维）：
 *     [ φ(3) | δv(3) | δp(3) | eb(3) | db(3) | lever?(3) ]
 */

/**
 * @brief 误差状态 EKF 工作区
 */
struct ErrorState {
    double _timestamp = 0.0;
    int _nx = 0;
    int _nr = 0;

    Eigen::MatrixXd _Ft;
    Eigen::MatrixXd _Pk;
    Eigen::VectorXd _Qt;
    Eigen::VectorXd _Xk;

    Eigen::MatrixXd _Hk;
    Eigen::MatrixXd _Rk;
    Eigen::MatrixXd _Sk;
    Eigen::MatrixXd _Kk;
    Eigen::VectorXd _Zk;
    Eigen::VectorXd _res;
    Eigen::VectorXd _norm_res;
};

/**
 * @brief EKF 状态整体：名义状态 + 地球模型 + 误差状态 + 分块布局
 */
class EkfState {
public:
    EkfState() = default;
    explicit EkfState(int state_dim) { Init(state_dim); }

    void Init(int state_dim);
    void Init(const StateLayout& layout);

    int Offset(BlockId id) const { return _layout.Offset(id); }
    bool Has(BlockId id) const { return _layout.Has(id); }
    int Dim() const { return _layout.Dim(); }

    NominalState _nominal;
    earth _eth;
    ErrorState _error;
    StateLayout _layout;
};

}  // namespace msf
