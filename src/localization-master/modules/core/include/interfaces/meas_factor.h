#pragma once

#include <Eigen/Eigen>

#include "math/earth.h"
#include "types/nav_state.h"
#include "types/state_layout.h"

namespace msf {

/** IEKF 迭代上下文：由优化器在每轮线性化前填充，因子只读。 */
struct IterationContext {
    int _iteration = 0;            ///< 第几次线性化，0 = 首轮
    bool _last_converged = false;  ///< 上一轮 dx 是否收敛；首轮固定为 false
};

/**
 * 传感器量测因子：在给定名义状态处线性化，产出 MeasBlock(H, R, z)。
 * EKF 与 IEKF 共用：迭代次数由 Ekf::Update 的 max_iterations 控制，因子本身不区分。
 */
class MeasFactor {
public:
    /** 传感器无关的量测包：任何因子只吐出 (H, R, z)，不触碰具体滤波器状态。
     *  EKF 消费为 (雅可比, 量测噪声, 新息/残差)；因子图优化器消费为 (雅可比, 信息, 残差)。 */
    struct MeasBlock {
        bool _valid = false;       ///< 量测是否可用（门控/时间对齐通过）
        double _timestamp = 0.0;   ///< 量测时间 [s]
        Eigen::MatrixXd _H;        ///< 量测雅可比 H；尺寸 nr×nx
        Eigen::VectorXd _R_diag;   ///< 量测噪声方差对角向量；尺寸 nr×1
        Eigen::MatrixXd _R;        ///< 可选满矩阵 R；nr×nr 且有限时优先于 _R_diag
        Eigen::VectorXd _z;        ///< 新息 / 量测残差 z；尺寸 nr

        int Rows() const { return static_cast<int>(_z.size()); }

        bool HasFullR() const {
            return _R.rows() == Rows() && _R.cols() == Rows() && Rows() > 0 && _R.allFinite();
        }
    };

    virtual ~MeasFactor() = default;

    /**
     * 在给定名义状态处线性化，产出 MeasBlock(H, R, z)。
     * 数据关联 / rematch 策略由因子自行决定：
     * GNSS 因子忽略；LiDAR 因子可依据 ctx._iteration 与 ctx._last_converged
     * 决定是否重新匹配（例如首轮或上一轮收敛时重做 kNN）。
     */
    virtual MeasBlock BuildMeasBlock(const NominalState& nominal, earth& eth,
                                     const StateLayout& layout,
                                     const IterationContext& ctx) = 0;

    /** 更新被接受后提交（默认空；LiDAR 覆写为地图插点）。 */
    virtual void Commit(const NominalState& nominal) {}
};

}  // namespace msf
