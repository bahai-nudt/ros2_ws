/**
 * @file         t_quat.h
 * @brief        quaternion helper class
 */

#pragma once

#include <Eigen/Eigen>


namespace msf {

class t_quat {
public:
    t_quat();
    t_quat(double q0, double q1, double q2, double q3);
    explicit t_quat(const Eigen::Vector4d& m);

    t_quat operator+(const t_quat& q) const;
    t_quat operator+(const Eigen::Vector3d& phi) const;
    t_quat operator-(const Eigen::Vector3d& phi) const;
    t_quat operator*(const t_quat& q) const;
    Eigen::Vector3d operator-(const t_quat& quat) const;
    Eigen::Vector3d operator*(const Eigen::Vector3d& v) const;
    t_quat& operator*=(const t_quat& q);

    static void normlize(t_quat& q);
    static t_quat conj(const t_quat& q);

    Eigen::Matrix4d left();
    Eigen::Matrix4d right();
    Eigen::Vector3d vec();

public:
    double _q0;
    double _q1;
    double _q2;
    double _q3;
};

}  // namespace msf