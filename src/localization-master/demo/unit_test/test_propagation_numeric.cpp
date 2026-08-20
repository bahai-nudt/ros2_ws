#include <cmath>
#include <iostream>

#include "ekf_state.h"
#include "ekf.h"
#include "ins_propagator.h"
#include "math/pose_converter.h"
#include "math/constants.h"

namespace {

bool Near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

}  // namespace

int main() {
    msf::Ekf ekf(15);
    msf::EkfState& state = ekf.State();
    state._error._Qt.segment<3>(0) =
        (Eigen::Vector3d(2.5, 2.5, 2.0) * msf::constants::_dpsh).array().square().matrix();
    state._error._Qt.segment<3>(3) =
        (Eigen::Vector3d(408.0, 408.0, 408.0) * msf::constants::_ugpsHz).array().square().matrix();
    state._error._Qt.segment<3>(9) =
        (Eigen::Vector3d(108.0, 108.0, 108.0) * msf::constants::_dphpsh).array().square().matrix();
    state._error._Qt.segment<3>(12) =
        (Eigen::Vector3d(1050.0, 1050.0, 1050.0) * msf::constants::_ugpsh).array().square().matrix();
    state._error._Pk = Eigen::MatrixXd::Identity(15, 15) * 1e-4;

    const Eigen::Vector3d pos0(0.52, 1.92, 686.0);
    state._nominal._pos = pos0;
    state._nominal._vn = Eigen::Vector3d(1.0, -0.5, 0.1);
    state._nominal._att = Eigen::Vector3d(0.01, -0.02, 1.2);
    state._nominal._Cnb = msf::pose_converter::a2mat(state._nominal._att);
    state._nominal._Cbn = state._nominal._Cnb.transpose();
    state._nominal._qnb = msf::pose_converter::a2qua(state._nominal._att);
    state._eth.Update(state._nominal._pos, state._nominal._vn);

    msf::InsPropagator propagator;
    const double dt = 0.01;
    const Eigen::Vector3d gyro(0.01, -0.02, 0.05);
    const Eigen::Vector3d accel(0.1, 0.2, 9.8);

    for (int i = 0; i < 100; ++i) {
        propagator.Propagate(state._nominal, state._eth, gyro, accel, gyro, accel, dt);
        if (!ekf.TimeUpdate(dt, 1.0)) {
            std::cerr << "TimeUpdate failed at step " << i << "\n";
            return 1;
        }
    }

    const double pk_trace = state._error._Pk.trace();
    std::cout << "propagation preflight: t=" << state._nominal._t_cur
              << " pk_trace=" << pk_trace << "\n";

    if (!Near(state._nominal._t_cur, 1.0, 1e-9) || !std::isfinite(pk_trace) || pk_trace <= 0.0) {
        std::cerr << "numeric preflight failed\n";
        return 1;
    }

    std::cout << "numeric preflight passed\n";
    return 0;
}
