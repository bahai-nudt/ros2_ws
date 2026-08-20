/**
 * @file         pose_converter.h
 * @brief        pose / attitude conversion helpers (rotation only)
 */

#pragma once

#include <Eigen/Eigen>

#include "math/t_quat.h"

namespace msf {

/**
 * 姿态数学统一入口：欧拉角 / 旋转矩阵 / 四元数 / 等效旋转矢量互转。
 * 通用矩阵工具（askew / symmetry / delrow* 等）见 utility.h；
 * 坐标转换（lla/xyz/enu/utm）请使用 Coordinate。
 */
class pose_converter {
public:
    static Eigen::Matrix3d a2mat(const Eigen::Vector3d& att);
    static Eigen::Vector3d m2att(const Eigen::Matrix3d& m);
    static t_quat a2qua(const Eigen::Vector3d& att);
    static Eigen::Vector3d q2att(const t_quat& qnb);
    static t_quat rv2q(const Eigen::Vector3d& rv);
    static Eigen::Vector3d q2rv(const t_quat& q);
    static double WrapAngle(double angle_rad);
    static t_quat m2qua(const Eigen::Matrix3d& Cnb);
    static Eigen::Matrix3d q2mat(const t_quat& qnb);
    static Eigen::Matrix3d rv2m(const Eigen::Vector3d& rv);
    static Eigen::Matrix3d dv2mat(const Eigen::Vector3d& vb1,
                                  const Eigen::Vector3d& vb2,
                                  const Eigen::Vector3d& vn1,
                                  const Eigen::Vector3d& vn2);
    static Eigen::Matrix4d m2m4(const Eigen::Vector3d& v);
    static Eigen::Matrix4d m2m4_(const Eigen::Vector3d& v);
};

}  // namespace msf
