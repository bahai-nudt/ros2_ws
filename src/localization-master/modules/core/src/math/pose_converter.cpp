/**
 * @file         pose_converter.cpp
 * @brief        base function
 */

#include "math/pose_converter.h"
#include "math/utility.h"

#include <cmath>

namespace msf {

Eigen::Matrix3d pose_converter::a2mat(const Eigen::Vector3d& att) {
    double sp = sin(att(0)), cp = cos(att(0));
    double sr = sin(att(1)), cr = cos(att(1));
    double sy = sin(att(2)), cy = cos(att(2));

    Eigen::Matrix3d m;
    m << cy * cr - sy * sp * sr, -sy * cp, cy * sr + sy * sp * cr,
        sy * cr + cy * sp * sr, cy * cp, sy * sr - cy * sp * cr,
        -cp * sr, sp, cp * cr;
    return m;
}

Eigen::Vector3d pose_converter::m2att(const Eigen::Matrix3d& m) {
    Eigen::Vector3d att;
    att(0) = asin(m(2, 1));
    att(1) = atan2(-m(2, 0), m(2, 2));
    att(2) = atan2(-m(0, 1), m(1, 1));
    return att;
}

t_quat pose_converter::a2qua(const Eigen::Vector3d& att) {
    double pitch = att(0) / 2.0, roll = att(1) / 2.0, yaw = att(2) / 2.0;
    double sp = sin(pitch), sr = sin(roll), sy = sin(yaw), cp = cos(pitch), cr = cos(roll),
           cy = cos(yaw);
    t_quat qnb;
    qnb._q0 = cp * cr * cy - sp * sr * sy;
    qnb._q1 = sp * cr * cy - cp * sr * sy;
    qnb._q2 = cp * sr * cy + sp * cr * sy;
    qnb._q3 = cp * cr * sy + sp * sr * cy;
    return qnb;
}

Eigen::Vector3d pose_converter::q2att(const t_quat& qnb) {
    double q11 = qnb._q0 * qnb._q0, q12 = qnb._q0 * qnb._q1, q13 = qnb._q0 * qnb._q2,
           q14 = qnb._q0 * qnb._q3, q22 = qnb._q1 * qnb._q1, q23 = qnb._q1 * qnb._q2,
           q24 = qnb._q1 * qnb._q3, q33 = qnb._q2 * qnb._q2, q34 = qnb._q2 * qnb._q3,
           q44 = qnb._q3 * qnb._q3;
    Eigen::Vector3d att;
    att(0) = asin(2 * (q34 + q12));
    att(1) = atan2(-2 * (q24 - q13), q11 - q22 - q33 + q44);
    att(2) = atan2(-2 * (q23 - q14), q11 - q22 + q33 - q44);
    return att;
}

t_quat pose_converter::rv2q(const Eigen::Vector3d& rv) {
#define F1 (2 * 1)
#define F2 (F1 * 2 * 2)
#define F3 (F2 * 2 * 3)
#define F4 (F3 * 2 * 4)
#define F5 (F4 * 2 * 5)
    double c, f, n2 = rv.norm() * rv.norm();
    if (n2 < (M_PI / 180.0 * M_PI / 180.0)) {
        double n4 = n2 * n2;
        c = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);
        f = 0.5 - n2 * (1.0 / F3) + n4 * (1.0 / F5);
    } else {
        double n_2 = sqrt(n2) / 2.0;
        c = cos(n_2);
        f = sin(n_2) / n_2 * 0.5;
    }
    return t_quat(c, f * rv(0), f * rv(1), f * rv(2));
}

Eigen::Vector3d pose_converter::q2rv(const t_quat& q) {
    t_quat dq = q;
    if (dq._q0 < 0) {
        dq._q0 = -dq._q0;
        dq._q1 = -dq._q1;
        dq._q2 = -dq._q2;
        dq._q3 = -dq._q3;
    }
    if (dq._q0 > 1.0) {
        dq._q0 = 1.0;
    }
    double n2 = acos(dq._q0), f;
    if (n2 > 1.0e-20) {
        f = 2.0 / (sin(n2) / n2);
    } else {
        f = 2.0;
    }
    return Eigen::Vector3d(dq._q1, dq._q2, dq._q3) * f;
}

double pose_converter::WrapAngle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

t_quat pose_converter::m2qua(const Eigen::Matrix3d& Cnb) {
    double q0, q1, q2, q3, qq4;
    if (Cnb(0, 0) >= Cnb(1, 1) + Cnb(2, 2)) {
        q1 = 0.5 * sqrt(1 + Cnb(0, 0) - Cnb(1, 1) - Cnb(2, 2));
        qq4 = 4 * q1;
        q0 = (Cnb(2, 1) - Cnb(1, 2)) / qq4;
        q2 = (Cnb(0, 1) + Cnb(1, 0)) / qq4;
        q3 = (Cnb(0, 2) + Cnb(2, 0)) / qq4;
    } else if (Cnb(1, 1) >= Cnb(0, 0) + Cnb(2, 2)) {
        q2 = 0.5 * sqrt(1 - Cnb(0, 0) + Cnb(1, 1) - Cnb(2, 2));
        qq4 = 4 * q2;
        q0 = (Cnb(0, 2) - Cnb(2, 0)) / qq4;
        q1 = (Cnb(0, 1) + Cnb(1, 0)) / qq4;
        q3 = (Cnb(1, 2) + Cnb(2, 1)) / qq4;
    } else if (Cnb(2, 2) >= Cnb(0, 0) + Cnb(1, 1)) {
        q3 = 0.5 * sqrt(1 - Cnb(0, 0) - Cnb(1, 1) + Cnb(2, 2));
        qq4 = 4 * q3;
        q0 = (Cnb(1, 0) - Cnb(0, 1)) / qq4;
        q1 = (Cnb(0, 2) + Cnb(2, 0)) / qq4;
        q2 = (Cnb(1, 2) + Cnb(2, 1)) / qq4;
    } else {
        q0 = 0.5 * sqrt(1 + Cnb(0, 0) + Cnb(1, 1) + Cnb(2, 2));
        qq4 = 4 * q0;
        q1 = (Cnb(2, 1) - Cnb(1, 2)) / qq4;
        q2 = (Cnb(0, 2) - Cnb(2, 0)) / qq4;
        q3 = (Cnb(1, 0) - Cnb(0, 1)) / qq4;
    }
    double nq = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= nq;
    q1 /= nq;
    q2 /= nq;
    q3 /= nq;
    return t_quat(q0, q1, q2, q3);
}

Eigen::Matrix3d pose_converter::q2mat(const t_quat& qnb) {
    double q11 = qnb._q0 * qnb._q0, q12 = qnb._q0 * qnb._q1, q13 = qnb._q0 * qnb._q2,
           q14 = qnb._q0 * qnb._q3, q22 = qnb._q1 * qnb._q1, q23 = qnb._q1 * qnb._q2,
           q24 = qnb._q1 * qnb._q3, q33 = qnb._q2 * qnb._q2, q34 = qnb._q2 * qnb._q3,
           q44 = qnb._q3 * qnb._q3;
    Eigen::Matrix3d Cnb;
    Cnb(0, 0) = q11 + q22 - q33 - q44;
    Cnb(0, 1) = 2 * (q23 - q14);
    Cnb(0, 2) = 2 * (q24 + q13);
    Cnb(1, 0) = 2 * (q23 + q14);
    Cnb(1, 1) = q11 - q22 + q33 - q44;
    Cnb(1, 2) = 2 * (q34 - q12);
    Cnb(2, 0) = 2 * (q24 - q13);
    Cnb(2, 1) = 2 * (q34 + q12);
    Cnb(2, 2) = q11 - q22 - q33 + q44;
    return Cnb;
}

Eigen::Matrix3d pose_converter::rv2m(const Eigen::Vector3d& rv) {
    double theta = rv.norm(), a, b;
    const double th_small = (M_PI / 180.0) * (M_PI / 180.0);
    if (theta * theta < th_small) {
        double th2 = theta * theta;
        a = 1.0 - th2 / 6.0 + th2 * th2 / 120.0;
        b = 0.5 - th2 / 24.0 + th2 * th2 / 720.0;
    } else {
        a = sin(theta) / theta;
        b = (1.0 - cos(theta)) / (theta * theta);
    }
    Eigen::Matrix3d rx = askew(rv);
    Eigen::Matrix3d DCM = Eigen::Matrix3d::Identity() + a * rx + b * rx * rx;
    return DCM;
}

Eigen::Matrix3d pose_converter::dv2mat(const Eigen::Vector3d& vb1,
                                       const Eigen::Vector3d& vb2,
                                       const Eigen::Vector3d& vn1,
                                       const Eigen::Vector3d& vn2) {
    Eigen::Vector3d vb = vb1.cross(vb2), vn = vn1.cross(vn2);
    Eigen::Vector3d vbb = vb.cross(vb1), vnn = vn.cross(vn1);
    Eigen::Matrix3d Mb, Mn;
    Mb.row(0) = (vb1 / vb1.norm()).transpose();
    Mn.row(0) = (vn1 / vn1.norm()).transpose();
    Mb.row(1) = (vb / vb.norm()).transpose();
    Mn.row(1) = (vn / vn.norm()).transpose();
    Mb.row(2) = (vbb / vbb.norm()).transpose();
    Mn.row(2) = (vnn / vnn.norm()).transpose();
    return Mb.transpose() * Mn;
}

Eigen::Matrix4d pose_converter::m2m4(const Eigen::Vector3d& v) {
    Eigen::Matrix4d M;
    M << 0.0, -v(0), -v(1), -v(2),
        v(0), 0.0, -v(2), v(1),
        v(1), v(2), 0.0, -v(0),
        v(2), -v(1), v(0), 0.0;
    return M;
}

Eigen::Matrix4d pose_converter::m2m4_(const Eigen::Vector3d& v) {
    Eigen::Matrix4d M;
    M << 0.0, -v(0), -v(1), -v(2),
        v(0), 0.0, v(2), -v(1),
        v(1), -v(2), 0.0, v(0),
        v(2), v(1), -v(0), 0.0;
    return M;
}



}  // namespace msf