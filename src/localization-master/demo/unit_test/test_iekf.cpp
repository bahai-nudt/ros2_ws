#include <cmath>
#include <iostream>
#include <vector>

#include "ekf.h"
#include "interfaces/meas_factor.h"

namespace msf {
namespace {

bool Near(double actual, double expected, double tol = 1e-9) {
    return std::abs(actual - expected) <= tol;
}

// 一维线性因子：残差 h = vn(0) - x_true，H = 1，R = 1。
// 用速度块(offset 3)作为被估计的一维状态；记录收到的 iteration 序列。
class LinearFactor : public MeasFactor {
public:
    explicit LinearFactor(double x_true) : x_true_(x_true) {}

    MeasBlock BuildMeasBlock(const NominalState& nominal, earth&,
                             const StateLayout& layout,
                             const IterationContext& ctx) override {
        iterations_.push_back(ctx._iteration);
        MeasBlock block;
        block._valid = !force_invalid_;
        block._timestamp = nominal._t_cur;
        const int nx = layout.Dim();
        block._H = Eigen::MatrixXd::Zero(1, nx);
        block._H(0, layout.Offset(BlockId::Velocity)) = 1.0;
        block._R_diag = Eigen::VectorXd::Zero(1);
        block._R_diag(0) = 1.0;
        block._z = Eigen::VectorXd::Zero(1);
        block._z(0) = nominal._vn(0) - x_true_;
        return block;
    }

    void Commit(const NominalState&) override { committed_ = true; }

    std::vector<int> iterations_;
    bool committed_ = false;
    bool force_invalid_ = false;

private:
    double x_true_;
};

Ekf MakeEkf() {
    Ekf ekf(15);
    EkfState& s = ekf.State();
    s._nominal._vn = Eigen::Vector3d(5.0, 0.0, 0.0);  // 先验 x=5
    s._error._Pk = Eigen::MatrixXd::Zero(15, 15);
    s._error._Pk(3, 3) = 4.0;                          // P=4
    s._error._Xk.setZero();
    return ekf;
}

// 首轮有效、次轮无效：验证中间轮失败保留上一轮修正，循环结束后用 last_KH 提交。
class InvalidMiddleFactor : public MeasFactor {
public:
    explicit InvalidMiddleFactor(double x_true) : x_true_(x_true) {}

    MeasBlock BuildMeasBlock(const NominalState& nominal, earth&,
                             const StateLayout& layout,
                             const IterationContext& ctx) override {
        MeasBlock block;
        if (ctx._iteration == 1) {
            invalid_seen_ = true;
            return block;
        }
        block._valid = true;
        block._timestamp = nominal._t_cur;
        const int nx = layout.Dim();
        block._H = Eigen::MatrixXd::Zero(1, nx);
        block._H(0, layout.Offset(BlockId::Velocity)) = 1.0;
        block._R_diag = Eigen::VectorXd::Zero(1);
        block._R_diag(0) = 1.0;
        block._z = Eigen::VectorXd::Zero(1);
        block._z(0) = nominal._vn(0) - x_true_;
        return block;
    }

    void Commit(const NominalState&) override { committed_ = true; }

    bool invalid_seen_ = false;
    bool committed_ = false;

private:
    double x_true_;
};

// 中间轮因子无效：不应回滚，而应沿用上一轮 last_KH 提交。
bool TestMiddleInvalidUsesLastUpdate() {
    Ekf ekf = MakeEkf();
    InvalidMiddleFactor factor(0.0);

    if (!ekf.Update(factor, 2)) {
        return false;
    }
    const EkfState& s = ekf.State();
    return factor.invalid_seen_ && factor.committed_ &&
           Near(s._nominal._vn(0), 1.0) &&
           Near(s._error._Pk(3, 3), 0.8) &&
           s._error._Xk.norm() == 0.0;
}

// max_iterations=1（标准 EKF）：x 5->1，P 4->0.8，误差清零，提交
bool TestSingleIterationEqualsEkf() {
    Ekf ekf = MakeEkf();
    LinearFactor factor(0.0);

    if (!ekf.Update(factor, 1)) {
        return false;
    }
    const EkfState& s = ekf.State();
    return Near(s._nominal._vn(0), 1.0) &&
           Near(s._error._Pk(3, 3), 0.8) &&
           s._error._Xk.norm() == 0.0 &&
           factor.committed_ &&
           factor.iterations_.size() == 1 && factor.iterations_[0] == 0;
}

// max_iterations=5（IEKF）：收敛到同一后验均值，P 相同，提交
bool TestIteratedConverges() {
    Ekf ekf = MakeEkf();
    LinearFactor factor(0.0);

    if (!ekf.Update(factor, 5)) {
        return false;
    }
    const EkfState& s = ekf.State();
    return Near(s._nominal._vn(0), 1.0) &&
           Near(s._error._Pk(3, 3), 0.8) &&
           s._error._Xk.norm() == 0.0 &&
           factor.committed_ &&
           !factor.iterations_.empty();
}

// 无效量测块：返回 false，且失败后恢复先验状态（不污染）
bool TestInvalidBlockRestoresPrior() {
    Ekf ekf = MakeEkf();
    LinearFactor factor(0.0);
    factor.force_invalid_ = true;

    if (ekf.Update(factor, 3)) {
        return false;
    }
    const EkfState& s = ekf.State();
    return Near(s._nominal._vn(0), 5.0) && !factor.committed_;
}

// 迭代序号按 0,1,2,... 传入（因子据此决定数据关联）
bool TestIterationSequence() {
    Ekf ekf = MakeEkf();
    LinearFactor factor(0.0);

    if (!ekf.Update(factor, 3)) {
        return false;
    }
    if (factor.iterations_.size() != 3) {
        return false;
    }
    return factor.iterations_[0] == 0 && factor.iterations_[1] == 1 && factor.iterations_[2] == 2;
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
        {"single_iteration_equals_ekf", TestSingleIterationEqualsEkf},
        {"iterated_converges", TestIteratedConverges},
        {"invalid_block_restores_prior", TestInvalidBlockRestoresPrior},
        {"iteration_sequence", TestIterationSequence},
        {"middle_invalid_uses_last_update", TestMiddleInvalidUsesLastUpdate},
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
