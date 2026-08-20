#pragma once

#include <Eigen/Eigen>

#include "math/t_quat.h"

namespace msf {

/** 车体导航输出（跨模块输出契约：tools 写、app / 在线发布读） */
struct VehiclePose {
    double _timestamp = 0.0;  // 系统时间 [s]（后处理中为 0）
    double _obs_time = 0.0;   // 观测时间（Unix 秒）
    int _state = 0;           // 0=异常, 1=警告, 2=正常

    Eigen::Vector3d _position_enu = Eigen::Vector3d::Zero();        // 车体 ENU 位置 [m]
    t_quat _qnb;                                                   // n←b 姿态四元数（默认单位）
    Eigen::Vector3d _euler = Eigen::Vector3d::Zero();              // [roll, pitch, yaw] [rad]
    Eigen::Vector3d _velocity_enu = Eigen::Vector3d::Zero();       // ENU 速度 [m/s]
    Eigen::Vector3d _accel_enu = Eigen::Vector3d::Zero();          // ENU 加速度 [m/s^2]
    Eigen::Vector3d _angular_velocity_body = Eigen::Vector3d::Zero();  // 机体系角速度 [rad/s]

    Eigen::Vector3d _lla_deg = Eigen::Vector3d::Zero();  // [lat, lon, h]（度, 米）
    double _speed = 0.0;                                 // 沿航向速度 [m/s]（含符号）
};

}  // namespace msf
