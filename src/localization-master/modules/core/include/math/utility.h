#pragma once

#include <Eigen/Eigen>

#include "math/constants.h"

namespace msf {

/** 符号函数：正 / 负 / 零（判零阈值为 constants::_EPS） */
inline int sign(double d) noexcept {
    if (d > constants::_EPS) {
        return 1;
    }
    if (d < -constants::_EPS) {
        return -1;
    }
    return 0;
}

// —— 通用矩阵 / 线性代数小工具（与具体领域无关）——

/** 车体 FLU → IMU RFU：V_rfu = FLU2RFU() * V_flu */
inline Eigen::Matrix3d FLU2RFU() {
    Eigen::Matrix3d R;
    R << 0.0, -1.0, 0.0,
         1.0,  0.0, 0.0,
         0.0,  0.0, 1.0;
    return R;
}

/** 叉积反对称矩阵：v× = askew(v) */
inline Eigen::Matrix3d askew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d vnx;
    vnx << 0, -v(2), v(1),
           v(2), 0, -v(0),
           -v(1), v(0), 0;
    return vnx;
}

/** 矩阵对称化：m = (m + mᵀ) / 2 */
inline void symmetry(Eigen::MatrixXd& m) {
    m = (m + m.transpose()) / 2.0;
}

/** vecᵀ · mat：逐列点积 */
inline Eigen::Vector3d product(const Eigen::Vector3d& vec, const Eigen::Matrix3d& mat) {
    Eigen::Vector3d res;
    for (int i = 0; i < 3; ++i) {
        res(i) = vec(0) * mat(0, i) + vec(1) * mat(1, i) + vec(2) * mat(2, i);
    }
    return res;
}

/** 删除矩阵第 i 行第 i 列 */
inline void delrowcol(Eigen::MatrixXd& M, int i) {
    Eigen::MatrixXd TMP = M;
    const int m = TMP.rows();
    const int n = TMP.cols();
    M = Eigen::MatrixXd::Zero(m - 1, n - 1);
    M.block(0, 0, i, i) = TMP.block(0, 0, i, i);
    M.block(0, i, i, n - i - 1) = TMP.block(0, i + 1, i, n - i - 1);
    M.block(i, 0, m - i - 1, i) = TMP.block(i + 1, 0, m - i - 1, i);
    M.block(i, i, m - i - 1, n - i - 1) = TMP.block(i + 1, i + 1, m - i - 1, n - i - 1);
}

/** 删除矩阵第 i 行 */
inline void delrow(Eigen::MatrixXd& M, int i) {
    Eigen::MatrixXd TMP = M;
    const int m = TMP.rows();
    const int n = TMP.cols();
    M = Eigen::MatrixXd::Zero(m - 1, n);
    M.block(0, 0, i, n) = TMP.block(0, 0, i, n);
    M.block(i, 0, m - i - 1, n) = TMP.block(i + 1, 0, m - i - 1, n);
}

/** 删除矩阵第 i 列 */
inline void delcol(Eigen::MatrixXd& M, int i) {
    Eigen::MatrixXd TMP = M;
    const int m = TMP.rows();
    const int n = TMP.cols();
    M = Eigen::MatrixXd::Zero(m, n - 1);
    M.block(0, 0, m, i) = TMP.block(0, 0, m, i);
    M.block(0, i, m, n - i - 1) = TMP.block(0, i + 1, m, n - i - 1);
}

/** 删除向量第 i 个元素 */
inline void delrow(Eigen::VectorXd& V, int i) {
    Eigen::VectorXd TMP = V;
    const int m = TMP.rows();
    V = Eigen::VectorXd::Zero(m - 1);
    V.block(0, 0, i, 1) = TMP.block(0, 0, i, 1);
    V.block(i, 0, m - i - 1, 1) = TMP.block(i + 1, 0, m - i - 1, 1);
}

}  // namespace msf
