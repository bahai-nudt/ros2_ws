#pragma once

#include <Eigen/Eigen>

namespace msf {

/** GNSS 位置观测（bestgnsspos） */
struct GnssPos {
    double _timestamp = 0.0;  // UTC 时间 [s]
    int _sol_status = 0;      // 解算状态
    int _pos_type = 0;        // 50=fixed, 34=float
    Eigen::Vector3d _blh = Eigen::Vector3d::Zero();      // [lat(rad), lon(rad), h(m)]
    Eigen::Vector3d _blh_std = Eigen::Vector3d::Zero();  // 标准差 [lat(m), lon(m), h(m)]
};

/** GNSS 速度观测（bestvel，地速/航迹角/垂向速度） */
struct GnssVel {
    double _timestamp = 0.0;  // UTC 时间 [s]
    int _vel_type = 0;        // 50=fixed, 34=float, 8=differential
    double _hor_speed = 0.0;  // 水平地速 [m/s]
    double _trk_gnd = 0.0;    // 航迹角（相对真北）[rad]
    double _ver_speed = 0.0;  // 垂向速度 [m/s]（向上为正）
};

/** 双天线航向观测（heading2） */
struct HeadingData {
    double _timestamp = 0.0;  // UTC 时间 [s]
    int _sol_status = 0;      // 解算状态
    int _pos_type = 0;        // 50=fixed, 34=float, 16=single
    double _baseline_length = 0.0;  // 基线长度 [m]
    double _heading = 0.0;          // 航向 [rad]
    double _heading_std = 0.0;      // 航向标准差 [rad]
};

}  // namespace msf
