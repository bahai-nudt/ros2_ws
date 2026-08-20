#pragma once

#include <Eigen/Eigen>

namespace msf {

/** IMU 原始/预处理数据（b 系，RFU） */
struct ImuData {
    double _timestamp = 0.0;  // UTC 时间 [s]
    double _dt = 0.0;         // 与上一帧的时间间隔 [s]
    Eigen::Vector3d _gyro = Eigen::Vector3d::Zero();   // 角速度 [rad/s]
    Eigen::Vector3d _accel = Eigen::Vector3d::Zero();  // 加速度 [m/s^2]
    Eigen::Vector3d _wm = Eigen::Vector3d::Zero();     // 角增量 [rad]
    Eigen::Vector3d _vm = Eigen::Vector3d::Zero();     // 速度增量 [m/s]
};

}  // namespace msf
