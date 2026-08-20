#include <cmath>
#include <iostream>

#include "config/global_config.h"
#include "math/pose_converter.h"
#include "math/utility.h"
#include "msf_algorithms/zupt.h"

namespace msf {
namespace {

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

NominalState MakeNominal() {
    NominalState nominal;
    nominal._pos = Eigen::Vector3d(0.52, 1.92, 686.0);
    nominal._vn = Eigen::Vector3d(0.0, 0.0, 0.0);
    nominal._att = Eigen::Vector3d(0.01, -0.02, 1.2);
    nominal._Cnb = pose_converter::a2mat(nominal._att);
    nominal._Cbn = nominal._Cnb.transpose();
    nominal._qnb = pose_converter::a2qua(nominal._att);
    nominal._lever = Eigen::Vector3d(0.1, -0.2, 0.3);
    nominal._wib = Eigen::Vector3d(0.001, 0.002, 0.003);
    nominal._wnb = Eigen::Vector3d(0.0, 0.0, 0.0);  // 静止判定用
    return nominal;
}

bool TestZuptStaticDuration() {
    const GlobalConfig config;
    algorithms::ZuptDetector detector(config._quality_control._zupt);

    NominalState nominal = MakeNominal();
    bool seen_active = false;
    for (int i = 0; i < 60; ++i) {
        const double ts = 100.0 + i * 0.01;
        const bool active = detector.Update(ts, 0.0, 0.0, nominal);
        if (i < 50 && active) {
            return false;  // 0.5s（50 帧间隔）之前不应判静止
        }
        if (i >= 50 && !active) {
            return false;
        }
        if (active) {
            seen_active = true;
        }
    }
    return seen_active && detector.Active();
}

bool TestZuptInterruptResets() {
    const GlobalConfig config;
    algorithms::ZuptDetector detector(config._quality_control._zupt);

    NominalState nominal = MakeNominal();
    // 先累计到静止
    for (int i = 0; i < 60; ++i) {
        detector.Update(100.0 + i * 0.01, 0.0, 0.0, nominal);
    }
    if (!detector.Active()) {
        return false;
    }
    // 一帧“运动”量测打断
    if (detector.Update(100.6, 1.0, 0.0, nominal)) {
        return false;
    }
    if (detector.StaticDuration() != 0.0) {
        return false;
    }
    // 打断后重新累计，短时间内不应判静止
    for (int i = 0; i < 10; ++i) {
        if (detector.Update(100.61 + i * 0.01, 0.0, 0.0, nominal)) {
            return false;
        }
    }
    return true;
}

bool TestZuptBlock() {
    const GlobalConfig config;
    algorithms::ZuptDetector detector(config._quality_control._zupt);

    NominalState nominal = MakeNominal();
    for (int i = 0; i < 60; ++i) {
        detector.Update(100.0 + i * 0.01, 0.0, 0.0, nominal);
    }
    if (!detector.Active()) {
        return false;
    }

    earth eth;
    StateLayout layout = StateLayout::FromStateDim(15);
    algorithms::ZeroVelocityFactor factor(config._quality_control._zupt);
    const auto block = factor.BuildMeasBlock(nominal, eth, layout, IterationContext{});
    if (!block._valid || block.Rows() != 3) {
        return false;
    }

    // 参考计算
    const Eigen::Vector3d web = nominal._wib - nominal._Cbn * eth._wnie;
    const Eigen::Vector3d lever_n = nominal._Cnb * nominal._lever;
    const Eigen::Vector3d wib_cross_lever_n =
        nominal._Cnb * (nominal._wib.cross(nominal._lever));
    const Eigen::Vector3d vn_ins_L =
        nominal._vn + nominal._Cnb * askew(web) * nominal._lever;

    Eigen::MatrixXd H_ref = Eigen::MatrixXd::Zero(3, 15);
    H_ref.block(0, layout.Offset(BlockId::Rotation), 3, 3) =
        askew(wib_cross_lever_n) - askew(eth._wnie) * askew(lever_n);
    H_ref.block(0, layout.Offset(BlockId::Velocity), 3, 3) = Eigen::Matrix3d::Identity();
    H_ref.block(0, layout.Offset(BlockId::GyroBias), 3, 3) =
        -nominal._Cnb * askew(nominal._lever);
    const Eigen::Vector3d R_diag =
        config._quality_control._zupt._meas_std_mps.array().square().matrix();

    return MatNear(block._H, H_ref) &&
           VecNear(block._R_diag, R_diag) &&
           MatNear(block._z, vn_ins_L) &&
           detector.StaticDuration() >= config._quality_control._zupt._min_duration_sec;
}

bool TestZuptReset() {
    const GlobalConfig config;
    algorithms::ZuptDetector detector(config._quality_control._zupt);
    NominalState nominal = MakeNominal();
    for (int i = 0; i < 60; ++i) {
        detector.Update(100.0 + i * 0.01, 0.0, 0.0, nominal);
    }
    if (!detector.Active()) {
        return false;
    }
    detector.Reset();
    return !detector.Active() && detector.StaticDuration() == 0.0;
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
        {"zupt_static_duration", TestZuptStaticDuration},
        {"zupt_interrupt_resets", TestZuptInterruptResets},
        {"zupt_block", TestZuptBlock},
        {"zupt_reset", TestZuptReset},
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
