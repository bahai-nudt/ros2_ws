#include <cmath>
#include <iostream>

#include "ekf.h"

namespace msf {
namespace {

bool Near(double actual, double expected, double tolerance = 1e-12) {
    return std::abs(actual - expected) <= tolerance;
}


class BlockFactor : public MeasFactor {
public:
    explicit BlockFactor(MeasBlock block) : block_(std::move(block)) {}
    MeasBlock BuildMeasBlock(const NominalState&, earth&, const StateLayout&, const IterationContext&) override {
        return block_;
    }
private:
    MeasBlock block_;
};

Ekf MakeEkf() {
    Ekf ekf(15);
    return ekf;
}

bool TestMeasUpdate() {
    Ekf ekf = MakeEkf();
    EkfState& s = ekf.State();
    s._error._Xk = Eigen::VectorXd::Zero(15);
    s._error._Pk = Eigen::MatrixXd::Zero(15, 15);
    s._error._Pk(0, 0) = 4.0;

    Eigen::MatrixXd H(1, 15);
    H.setZero();
    H(0, 0) = 1.0;
    Eigen::MatrixXd R(1, 1);
    R << 1.0;
    Eigen::VectorXd Z(1);
    Z << 7.0;

    MeasFactor::MeasBlock block;
    block._valid = true;
    block._timestamp = 0.0;
    block._H = H;
    block._R_diag = R.diagonal();
    block._z = Z;

    BlockFactor factor(std::move(block));
    if (!ekf.Update(factor, 1)) {
        return false;
    }

    const ErrorState& err = s._error;
    return Near(err._Pk(0, 0), 0.8) && err._Xk.norm() == 0.0;
}

bool TestMeasUpdateMatrixCase() {
    Ekf ekf = MakeEkf();
    EkfState& s = ekf.State();
    s._error._Xk = Eigen::VectorXd::Zero(15);
    s._error._Pk = Eigen::MatrixXd::Zero(15, 15);
    s._error._Pk.block<2, 2>(0, 0) << 2.0, 1.0,
                                      1.0, 2.0;

    Eigen::MatrixXd H(1, 15);
    H.setZero();
    H(0, 0) = 1.0;
    H(0, 1) = 1.0;
    Eigen::MatrixXd R(1, 1);
    R << 1.0;
    Eigen::VectorXd Z(1);
    Z << 6.0;

    MeasFactor::MeasBlock block;
    block._valid = true;
    block._timestamp = 0.0;
    block._H = H;
    block._R_diag = R.diagonal();
    block._z = Z;

    BlockFactor factor(std::move(block));
    if (!ekf.Update(factor, 1)) {
        return false;
    }

    Eigen::Matrix2d expected_p;
    expected_p << 5.0 / 7.0, -2.0 / 7.0,
                  -2.0 / 7.0, 5.0 / 7.0;

    const ErrorState& err = s._error;
    return err._Pk.block<2, 2>(0, 0).isApprox(expected_p, 1e-12) &&
           err._Xk.norm() == 0.0;
}

bool TestRejectsInvalidMeasDimensions() {
    Ekf ekf = MakeEkf();
    EkfState& s = ekf.State();
    s._error._Xk = Eigen::VectorXd::Zero(15);
    s._error._Pk = Eigen::MatrixXd::Identity(15, 15);

    Eigen::MatrixXd H(2, 15);
    H.setZero();
    Eigen::MatrixXd R(1, 1);
    R << 1.0;
    Eigen::VectorXd Z(1);
    Z << 0.0;

    MeasFactor::MeasBlock block;
    block._valid = true;
    block._timestamp = 0.0;
    block._H = H;
    block._R_diag = R.diagonal();
    block._z = Z;

    BlockFactor factor(std::move(block));
    return !ekf.Update(factor, 1);
}

bool TestNormalizedResidualGateRejectsAndAccepts() {
    auto make_factor = [](double z_value) {
        Eigen::MatrixXd H(1, 15);
        H.setZero();
        H(0, 0) = 1.0;
        MeasFactor::MeasBlock block;
        block._valid = true;
        block._timestamp = 0.0;
        block._H = H;
        block._R_diag = Eigen::VectorXd::Constant(1, 1.0e-4);
        block._z = Eigen::VectorXd::Constant(1, z_value);
        return BlockFactor(std::move(block));
    };

    {
        Ekf ekf = MakeEkf();
        EkfState& s = ekf.State();
        s._error._Xk = Eigen::VectorXd::Zero(15);
        s._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1.0e-4;
        s._nominal._att = Eigen::Vector3d(0.1, 0.2, 0.3);
        const NominalState before = s._nominal;
        const Eigen::MatrixXd P_before = s._error._Pk;
        BlockFactor factor = make_factor(1.0);
        // S = 1e-4+1e-4，|r|/σ ≈ 70.7 > 3，应拒绝且不改状态
        if (ekf.Update(factor, 1, constants::_iekf_convergence_threshold, 3.0)) {
            return false;
        }
        if (s._nominal._att != before._att || !s._error._Pk.isApprox(P_before, 0.0) ||
            s._error._norm_res.size() != 1 ||
            s._error._norm_res.cwiseAbs().maxCoeff() <= 3.0) {
            return false;
        }
    }

    {
        Ekf ekf = MakeEkf();
        EkfState& s = ekf.State();
        s._error._Xk = Eigen::VectorXd::Zero(15);
        s._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1.0e-4;
        BlockFactor factor = make_factor(0.01);
        // |r|/σ ≈ 0.707 < 3，应接受
        if (!ekf.Update(factor, 1, constants::_iekf_convergence_threshold, 3.0)) {
            return false;
        }
        if (s._error._Xk.norm() != 0.0 || s._error._Pk(0, 0) >= 1.0e-4) {
            return false;
        }
    }

    {
        Ekf ekf = MakeEkf();
        EkfState& s = ekf.State();
        s._error._Xk = Eigen::VectorXd::Zero(15);
        s._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1.0e-4;
        BlockFactor factor = make_factor(1.0);
        // 无门控的 Update 仍应融合大新息（LiDAR / heading 路径）
        if (!ekf.Update(factor, 1)) {
            return false;
        }
    }
    return true;
}

bool TestRejectsInvalidTimeUpdate() {
    Ekf ekf(15);
    ekf.State()._error._Qt.conservativeResize(0);
    return !ekf.TimeUpdate(0.01);
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
        {"meas_update_scalar", TestMeasUpdate},
        {"meas_update_matrix", TestMeasUpdateMatrixCase},
        {"rejects_invalid_meas_dims", TestRejectsInvalidMeasDimensions},
        {"normalized_residual_gate", TestNormalizedResidualGateRejectsAndAccepts},
        {"rejects_invalid_time_update", TestRejectsInvalidTimeUpdate},
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
