#include "ins_initializer.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "math/constants.h"
#include "math/pose_converter.h"
#include "math/t_quat.h"

namespace msf {

bool InitializeLIO(const std::vector<ImuData>& imu_buffer,
                      const Eigen::Vector3d& origin_lla_rad,
                      const GlobalConfig::ImuCalib& imu_calib,
                      int min_samples,
                      NominalState& nominal,
                      earth& eth) {
    const int need = std::max(min_samples, 10);
    if (static_cast<int>(imu_buffer.size()) < need) {
        std::cerr << "[INIT LIO] 数据量不足: imu=" << imu_buffer.size() << " (>=" << need << ")"
                  << std::endl;
        return false;
    }

    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr = Eigen::Vector3d::Zero();
    for (int i = 0; i < need; ++i) {
        mean_acc += imu_buffer[static_cast<size_t>(i)]._accel;
        mean_gyr += imu_buffer[static_cast<size_t>(i)]._gyro;
    }
    mean_acc /= static_cast<double>(need);
    mean_gyr /= static_cast<double>(need);

    const double acc_norm = mean_acc.norm();
    if (acc_norm < 1.0) {
        std::cerr << "[INIT LIO] 加速度均值过小，无法调平: |mean_acc|=" << acc_norm << std::endl;
        return false;
    }

    // 与 AlignCoarse 一致：将比力单位矢量作为 Cnb 第三行（静止且水平时 ≈ [0,0,1]）
    const Eigen::Vector3d fb = mean_acc / acc_norm;
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, fb(1))));
    const double roll = std::atan2(-fb(0), fb(2));
    const double yaw = 0.0;

    const ImuData& last = imu_buffer[static_cast<size_t>(need - 1)];
    nominal = NominalState();
    nominal._t_cur = last._timestamp;
    nominal._nts = last._dt;
    nominal._pos = origin_lla_rad;
    nominal._vn = Eigen::Vector3d::Zero();
    nominal._att = Eigen::Vector3d(pitch, roll, yaw);
    nominal._Cnb = pose_converter::a2mat(nominal._att);
    nominal._Cbn = nominal._Cnb.transpose();
    nominal._qnb = pose_converter::a2qua(nominal._att);

    nominal._Kg = imu_calib._gyro_scale_factor;
    nominal._Ka = imu_calib._acc_scale_factor;
    nominal._eb = mean_gyr;  // 静止段陀螺均值 ≈ 零偏
    nominal._db = imu_calib._acc_bias_mg * constants::_mg;
    nominal._lever = Eigen::Vector3d::Zero();

    // 中间量与 e 系输出量显式清零（与 gnss::InitializeNominal 一致）
    nominal._wib = nominal._wnb = nominal._web = Eigen::Vector3d::Zero();
    nominal._fn = Eigen::Vector3d::Zero();
    nominal._an = Eigen::Vector3d::Zero();
    nominal._pos_ecef = Eigen::Vector3d::Zero();
    nominal._ve = Eigen::Vector3d::Zero();
    nominal._ae = Eigen::Vector3d::Zero();
    eth.Update(nominal._pos, nominal._vn);
    nominal._Ceb = Eigen::Matrix3d::Identity();
    nominal._Cbe = Eigen::Matrix3d::Identity();
    nominal._qeb = t_quat();

    std::cerr << "[INIT LIO] static init done: samples=" << need
              << " |mean_acc|=" << acc_norm
              << " att(deg)=" << (nominal._att * constants::_R2D).transpose()
              << " eb=" << nominal._eb.transpose() << std::endl;
    return true;
}

}  // namespace msf
