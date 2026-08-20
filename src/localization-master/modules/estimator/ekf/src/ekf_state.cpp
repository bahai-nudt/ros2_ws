#include "ekf_state.h"

namespace msf {

void EkfState::Init(const StateLayout& layout) {
    _layout = layout;

    const int nx = _layout.Dim();
    _error._timestamp = 0.0;
    _error._nx = nx;
    _error._nr = 0;
    _error._Ft = Eigen::MatrixXd::Zero(nx, nx);
    _error._Pk = Eigen::MatrixXd::Zero(nx, nx);
    _error._Qt = Eigen::VectorXd::Zero(nx);
    _error._Xk = Eigen::VectorXd::Zero(nx);
    _error._Hk.resize(0, nx);
    _error._Rk.resize(0, 0);
    _error._Sk.resize(0, 0);
    _error._Kk.resize(nx, 0);
    _error._Zk.resize(0);
    _error._res.resize(0);
    _error._norm_res.resize(0);
}

void EkfState::Init(int state_dim) {
    Init(StateLayout::FromStateDim(state_dim));
}

}  // namespace msf
