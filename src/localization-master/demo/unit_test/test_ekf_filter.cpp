#include <cmath>
#include <iostream>

#include "ekf.h"
#include "math/pose_converter.h"

namespace msf {
namespace {

bool Near(double a, double b, double tol = 1e-12) {
    return std::abs(a - b) <= tol;
}

void FillState(Ekf& ekf) {
    EkfState& s = ekf.State();
    s._nominal._pos = Eigen::Vector3d(0.52, 1.92, 686.0);
    s._nominal._vn = Eigen::Vector3d(1.0, -0.5, 0.1);
    s._nominal._att = Eigen::Vector3d(0.01, -0.02, 1.2);
    s._nominal._Cnb = pose_converter::a2mat(s._nominal._att);
    s._nominal._Cbn = s._nominal._Cnb.transpose();
    s._nominal._qnb = pose_converter::a2qua(s._nominal._att);
    s._eth.Update(s._nominal._pos, s._nominal._vn);

    const int nx = s.Dim();
    s._error._Pk = Eigen::MatrixXd::Identity(nx, nx) * 1e-4;
    s._error._Xk = Eigen::VectorXd::Ones(nx) * 0.01;
    s._error._Qt.setConstant(1e-6);
}

bool TestInit() {
    Ekf ekf(15);
    const EkfState& s = ekf.State();
    return s.Dim() == 15 && s._error._nx == 15 &&
           s._error._Xk.size() == 15 && s._error._Pk.rows() == 15 &&
           s._error._Pk.cols() == 15 && s.Has(BlockId::Rotation) &&
           s.Has(BlockId::GyroBias) && !s.Has(BlockId::LeverArm);
}

bool TestTimeUpdate() {
    Ekf ekf(15);
    FillState(ekf);
    const double trace_before = ekf.State()._error._Pk.trace();
    if (!ekf.TimeUpdate(0.01)) {
        return false;
    }
    const ErrorState& err = ekf.State()._error;
    return err._Pk.allFinite() && err._Xk.allFinite() &&
           err._Pk.trace() > trace_before;
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

bool TestMeasUpdate() {
    Ekf ekf(15);
    EkfState& s = ekf.State();
    s._error._Xk.setZero();
    s._error._Pk = 4.0 * Eigen::MatrixXd::Identity(15, 15);
    s._error._nr = 1;

    Eigen::MatrixXd H(1, 15);
    H.setZero();
    H(0, 0) = 1.0;
    Eigen::MatrixXd R(1, 1);
    R << 1.0;
    Eigen::VectorXd Z(1);
    Z << 7.0;

    s._error._Hk = H;
    s._error._Rk = R;
    s._error._Zk = Z;
    ekf.CalMeasResidual();
    if (s._error._norm_res.size() != 1 || !Near(s._error._res(0), 7.0)) {
        return false;
    }

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

bool TestStateFeedback() {
    Ekf ekf(15);
    EkfState& s = ekf.State();
    s._error._Xk = Eigen::VectorXd::Zero(15);
    s._error._Xk(3) = 1.0;  // δvE
    ekf.StateFeedback();
    return s._error._Xk.isZero() && Near(s._nominal._vn(0), -1.0);
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
        {"ins_ekf_init", TestInit},
        {"ins_ekf_time_update", TestTimeUpdate},
        {"ins_ekf_meas_update", TestMeasUpdate},
        {"ins_ekf_state_feedback", TestStateFeedback},
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
