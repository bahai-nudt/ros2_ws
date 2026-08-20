/**
 * @file         earth.cpp
 * @brief        Earth parameter updating class
 */

#include "math/earth.h"

#include <cmath>

#include "math/constants.h"

namespace msf {

earth::earth() : earth(constants::_Aell, 1.0 / constants::_Finv, constants::_G_EQUA) {}

earth::earth(double a0, double f0, double g0) {
    _a = a0;
    _f = f0;
    _b = (1 - _f) * _a;
    _wie = constants::_wie;
    _e = sqrt(_a * _a - _b * _b) / _a;
    _e2 = _e * _e;
    _sb = 0.0;
    _sb2 = 0.0;
    _sb4 = 0.0;
    _cb = 1.0;
    _tb = 0.0;
    _sl = 0.0;
    _cl = 1.0;
    _RMh = 0.0;
    _RNh = 0.0;
    _cbRNh = 0.0;
    _f_RMh = 0.0;
    _f_RNh = 0.0;
    _f_cbRNh = 0.0;
    _pos = Eigen::Vector3d::Zero();
    _vn = Eigen::Vector3d::Zero();
    _wnie = Eigen::Vector3d::Zero();
    _wnen = Eigen::Vector3d::Zero();
    _wnin = Eigen::Vector3d::Zero();
    _weie = Eigen::Vector3d::Zero();
    _gn = Eigen::Vector3d(0.0, 0.0, -g0);
    _gcc = _gn;
    _Cen = Eigen::Matrix3d::Identity();
    _Cne = Eigen::Matrix3d::Identity();

    Update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void earth::Update(const Eigen::Vector3d& pos, const Eigen::Vector3d& vn) {
    _pos = pos;
    _vn = vn;
    _sb = sin(pos(0)), _cb = cos(pos(0));
    if (fabs(_cb) < 1e-7) {
        _cb = (_cb >= 0.0 ? 1e-7 : -1e-7);
    }
    _tb = _sb / _cb;
    _sb2 = _sb * _sb, _sb4 = _sb2 * _sb2;
    _sl = sin(pos(1)), _cl = cos(pos(1));
    _Cen << -_sl, -_sb * _cl, _cb * _cl, _cl, -_sb * _sl, _cb * _sl, 0.0, _cb, _sb;
    _Cne = _Cen.transpose();
    double sq = 1.0 - _e2 * _sb2, sq2 = sqrt(sq);
    _RMh = _a * (1.0 - _e2) / sq / sq2 + pos(2);
    _f_RMh = 1.0 / _RMh;
    _RNh = _a / sq2 + pos(2);
    _cbRNh = _cb * _RNh;
    _f_RNh = 1.0 / _RNh;
    _f_cbRNh = 1.0 / _cbRNh;
    _wnie << 0.0, _wie * _cb, _wie * _sb;
    _weie = _Cen * _wnie;
    _wnen << -vn(1) * _f_RMh, vn(0) * _f_RNh, vn(0) * _f_RNh * _tb;
    _wnin = _wnie + _wnen;
    const double gL =
        constants::_g0 *
        (1.0 + constants::_beta * _sb2 -
         constants::_beta1 * (2.0 * _sb * _cb) * (2.0 * _sb * _cb));
    const double hR = pos(2) / (constants::_Re * (1 - constants::_f * _sb2));
    _gn(2) = -(gL * (1.0 - 2.0 * hR - 5.0 * hR * hR));
    _gcc = _gn - (_wnie + _wnin).cross(vn);
}

Eigen::Vector3d earth::v2dp(const Eigen::Vector3d& vn, double ts) {
    return Eigen::Vector3d(vn(1) * _f_RMh, vn(0) * _f_cbRNh, vn(2)) * ts;
}

}  // namespace msf
