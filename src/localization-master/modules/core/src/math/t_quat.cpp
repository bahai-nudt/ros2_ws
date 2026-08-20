/**
 * @file         t_quat.cpp
 * @brief        quaternion helper class
 */

#include "math/t_quat.h"

#include <cmath>
#include "math/utility.h"

#include "math/pose_converter.h"

namespace msf {

t_quat::t_quat() {
    _q0 = 1.0;
    _q1 = 0.0;
    _q2 = 0.0;
    _q3 = 0.0;
}

t_quat::t_quat(double q0, double q1, double q2, double q3) {
    _q0 = q0;
    _q1 = q1;
    _q2 = q2;
    _q3 = q3;
}

t_quat::t_quat(const Eigen::Vector4d& m) {
    _q0 = m(0);
    _q1 = m(1);
    _q2 = m(2);
    _q3 = m(3);
}

t_quat t_quat::operator+(const t_quat& q) const {
    return t_quat(_q0 + q._q0, _q1 + q._q1, _q2 + q._q2, _q3 + q._q3);
}

t_quat t_quat::operator+(const Eigen::Vector3d& phi) const {
    t_quat qtmp = pose_converter::rv2q(-phi);
    return qtmp * (*this);
}

t_quat t_quat::operator-(const Eigen::Vector3d& phi) const {
    t_quat qtmp = pose_converter::rv2q(phi);
    return qtmp * (*this);
}

t_quat t_quat::operator*(const t_quat& q) const {
    t_quat qtmp;
    qtmp._q0 = _q0 * q._q0 - _q1 * q._q1 - _q2 * q._q2 - _q3 * q._q3;
    qtmp._q1 = _q0 * q._q1 + _q1 * q._q0 + _q2 * q._q3 - _q3 * q._q2;
    qtmp._q2 = _q0 * q._q2 + _q2 * q._q0 + _q3 * q._q1 - _q1 * q._q3;
    qtmp._q3 = _q0 * q._q3 + _q3 * q._q0 + _q1 * q._q2 - _q2 * q._q1;
    return qtmp;
}

Eigen::Vector3d t_quat::operator-(const t_quat& quat) const {
    t_quat dq = quat * (t_quat::conj(*this));
    if (dq._q0 < 0) {
        dq._q0 = -dq._q0;
        dq._q1 = -dq._q1;
        dq._q2 = -dq._q2;
        dq._q3 = -dq._q3;
    }
    double n2 = std::acos(dq._q0);
    double f = 2.0;
    if (msf::sign(n2) != 0) {
        f = 2.0 / (std::sin(n2) / n2);
    }
    return Eigen::Vector3d(dq._q1, dq._q2, dq._q3) * f;
}

Eigen::Vector3d t_quat::operator*(const Eigen::Vector3d& v) const {
    t_quat qtmp;
    Eigen::Vector3d vtmp;
    qtmp._q0 = -_q1 * v(0) - _q2 * v(1) - _q3 * v(2);
    qtmp._q1 = _q0 * v(0) + _q2 * v(2) - _q3 * v(1);
    qtmp._q2 = _q0 * v(1) + _q3 * v(0) - _q1 * v(2);
    qtmp._q3 = _q0 * v(2) + _q1 * v(1) - _q2 * v(0);
    vtmp(0) = -qtmp._q0 * _q1 + qtmp._q1 * _q0 - qtmp._q2 * _q3 + qtmp._q3 * _q2;
    vtmp(1) = -qtmp._q0 * _q2 + qtmp._q2 * _q0 - qtmp._q3 * _q1 + qtmp._q1 * _q3;
    vtmp(2) = -qtmp._q0 * _q3 + qtmp._q3 * _q0 - qtmp._q1 * _q2 + qtmp._q2 * _q1;
    return vtmp;
}

t_quat& t_quat::operator*=(const t_quat& q) { return (*this = *this * q); }

void t_quat::normlize(t_quat& q) {
    double nq = std::sqrt(q._q0 * q._q0 + q._q1 * q._q1 + q._q2 * q._q2 + q._q3 * q._q3);
    if (nq < 1e-12) {
        return;
    }
    q._q0 /= nq;
    q._q1 /= nq;
    q._q2 /= nq;
    q._q3 /= nq;
}

t_quat t_quat::conj(const t_quat& q) { return t_quat(q._q0, -q._q1, -q._q2, -q._q3); }

Eigen::Matrix4d t_quat::left() {
    Eigen::Matrix4d mat;
    mat << _q0, -_q1, -_q2, -_q3,
        _q1, _q0, -_q3, _q2,
        _q2, _q3, _q0, -_q1,
        _q3, -_q2, _q1, _q0;
    return mat;
}

Eigen::Matrix4d t_quat::right() {
    Eigen::Matrix4d mat;
    mat << _q0, -_q1, -_q2, -_q3,
        _q1, _q0, _q3, -_q2,
        _q2, -_q3, _q0, _q1,
        _q3, _q2, -_q1, _q0;
    return mat;
}

Eigen::Vector3d t_quat::vec() { return Eigen::Vector3d(_q1, _q2, _q3); }

}  // namespace msf