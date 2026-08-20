/**
 * @file         earth.h
 * @brief        Earth parameter updating class
 */

#pragma once

#include <Eigen/Eigen>


namespace msf {

class earth {
public:
    earth();
    earth(double a0, double f0, double g0);

    void Update(const Eigen::Vector3d& pos, const Eigen::Vector3d& vn);
    Eigen::Vector3d v2dp(const Eigen::Vector3d& vn, double ts);

public:
    double _a, _b;
    double _f, _e, _e2, _wie;
    double _sb, _sb2, _sb4, _cb, _tb, _sl, _cl;
    double _RMh, _RNh, _cbRNh, _f_RMh, _f_RNh, _f_cbRNh;
    Eigen::Vector3d _pos, _vn;
    Eigen::Vector3d _wnie, _wnen, _wnin, _weie, _gn, _gcc;
    Eigen::Matrix3d _Cen, _Cne;
};

}  // namespace msf