#include <cmath>
#include <iostream>

#include "ekf.h"
#include "interfaces/meas_factor.h"

namespace msf {

bool Near(double actual, double expected, double tol = 1.0e-8) {
    return std::fabs(actual - expected) <= tol;
}

class BlockFactor : public MeasFactor {
public:
    explicit BlockFactor(MeasFactor::MeasBlock block) : block_(std::move(block)) {}
    MeasFactor::MeasBlock BuildMeasBlock(const NominalState&, earth&, const StateLayout&, const IterationContext&) override {
        return block_;
    }
private:
    MeasFactor::MeasBlock block_;
};

// nr=100 的对角 R 会触发信息形式路径；这里与标准 S 路径公式对比。
bool TestInfoFormMatchesStandard() {
    Ekf ekf(15);
    EkfState& s = ekf.State();
    s._error._Xk = Eigen::VectorXd::Zero(15);
    s._error._Pk = Eigen::MatrixXd::Identity(15, 15);

    const int nr = 100;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nr, 15);
    for (int i = 0; i < nr; ++i) {
        H(i, 3) = std::sin(static_cast<double>(i) * 0.1);  // 只观测速度 E
    }
    Eigen::VectorXd z(nr);
    for (int i = 0; i < nr; ++i) {
        z(i) = std::cos(static_cast<double>(i) * 0.07);
    }
    const Eigen::MatrixXd R = 0.25 * Eigen::MatrixXd::Identity(nr, nr);

    // 标准 S 路径的期望结果
    const Eigen::MatrixXd P0 = s._error._Pk;
    const Eigen::MatrixXd S = H * P0 * H.transpose() + R;
    const Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (!ldlt.isPositive()) {
        return false;
    }
    const Eigen::MatrixXd K = (ldlt.solve(H * P0)).transpose();
    const Eigen::VectorXd dx_expected = K * z;
    Eigen::MatrixXd P_expected =
        (Eigen::MatrixXd::Identity(15, 15) - K * H) * P0;
    P_expected = (P_expected + P_expected.transpose()) / 2.0;

    MeasFactor::MeasBlock block;
    block._valid = true;
    block._timestamp = 0.0;
    block._H = H;
    block._R_diag = R.diagonal();
    block._z = z;

    BlockFactor factor(std::move(block));
    if (!ekf.Update(factor, 1)) {
        return false;
    }

    const ErrorState& err = s._error;
    if (err._Xk.norm() != 0.0) {
        return false;
    }
    if (!Near(s._nominal._vn(0), -dx_expected(3))) {
        return false;
    }
    if (!err._Pk.isApprox(P_expected, 1.0e-8)) {
        return false;
    }
    return true;
}

// 真实量纲失衡的 P：姿态 1e-8、速度 1e-4、位置 1e-17、零偏 1e-16。
bool TestIllConditionedMatchesStandard() {
    Ekf ekf(15);
    EkfState& s = ekf.State();
    s._error._Xk = Eigen::VectorXd::Zero(15);

    Eigen::VectorXd p_diag(15);
    p_diag.segment<3>(0) = Eigen::Vector3d::Constant(1.0e-8);   // 姿态
    p_diag.segment<3>(3) = Eigen::Vector3d::Constant(1.0e-4);   // 速度
    p_diag.segment<3>(6) = Eigen::Vector3d::Constant(1.0e-17);  // 位置
    p_diag.segment<3>(9) = Eigen::Vector3d::Constant(1.0e-16);  // 陀螺零偏
    p_diag.segment<3>(12) = Eigen::Vector3d::Constant(1.0e-16); // 加计零偏
    s._error._Pk = p_diag.asDiagonal();

    const int nr = 100;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nr, 15);
    for (int i = 0; i < nr; ++i) {
        H(i, 0) = std::sin(static_cast<double>(i) * 0.1);
        H(i, 1) = std::cos(static_cast<double>(i) * 0.13);
        H(i, 6) = std::cos(static_cast<double>(i) * 0.07);
        H(i, 7) = std::sin(static_cast<double>(i) * 0.11);
        H(i, 8) = 1.0;
    }
    Eigen::VectorXd z(nr);
    for (int i = 0; i < nr; ++i) {
        z(i) = 0.01 * std::sin(static_cast<double>(i) * 0.03);
    }
    const double noise_var = 0.001;
    const Eigen::MatrixXd R = noise_var * Eigen::MatrixXd::Identity(nr, nr);

    const Eigen::MatrixXd P0 = s._error._Pk;
    const Eigen::MatrixXd S = H * P0 * H.transpose() + R;
    const Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (!ldlt.isPositive()) {
        return false;
    }
    const Eigen::MatrixXd K = (ldlt.solve(H * P0)).transpose();
    Eigen::MatrixXd P_expected =
        (Eigen::MatrixXd::Identity(15, 15) - K * H) * P0;
    P_expected = (P_expected + P_expected.transpose()) / 2.0;

    MeasFactor::MeasBlock block;
    block._valid = true;
    block._timestamp = 0.0;
    block._H = H;
    block._R_diag = Eigen::VectorXd::Constant(nr, noise_var);
    block._z = z;

    BlockFactor factor(std::move(block));
    if (!ekf.Update(factor, 1)) {
        return false;
    }

    const ErrorState& err = s._error;
    if (err._Xk.norm() != 0.0) {
        return false;
    }
    if (!err._Pk.isApprox(P_expected, 1.0e-6)) {
        return false;
    }
    return true;
}

}  // namespace msf

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"info_form_matches_standard", msf::TestInfoFormMatchesStandard},
        {"ill_conditioned_matches_standard", msf::TestIllConditionedMatchesStandard},
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
