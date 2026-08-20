#pragma once

#include <Eigen/Eigen>

#include "math/t_quat.h"

namespace msf {

/**
 * =============================================================================
 * 坐标系约定（多传感器融合）
 * =============================================================================
 *
 *   i 系（地心惯性系 Inertial）
 *     - 原点：地心；轴相对惯性空间近似固定。
 *     - 一般不显式存 i 系坐标，仅在地球自转、科氏项中出现。
 *
 *   e 系（地心地固系 ECEF）
 *     - 原点：地心；随地球旋转；WGS-84 (X,Y,Z) [m]。
 *     - 本结构体当前聚焦 n/b；e 系量可在 INS 模块中扩展。
 *
 *   n 系（导航系，东北天 ENU）
 *     - 轴：东(E)、北(N)、天(U)。
 *     - _vn = [vE, vN, vU]^T [m/s]。
 *     - 姿态、比力投影相对 n 系描述；局部结果也可投影到地理 ENU。
 *
 *   b 系（IMU 系 Body，右前上 RFU）
 *     - 轴：Xb 右、Yb 前、Zb 上（Right-Forward-Up）。
 *     - 陀螺 / 加计、零偏 _eb/_db、杆臂 _lever 均在 b 系表达。
 *     - 若传感器原始轴向不是 RFU，须在进入滤波器前变换到本约定。
 *
 *   v 系（车体系 Vehicle，前左上 FLU，非滤波名义状态）
 *     - 轴：前–左–上；常用于车体/外参标定。
 *     - 与 b 系之间由安装旋转联系，例如 V_b = R_bv * V_v（v=FLU → b=RFU）。
 *     - 对外发布若使用车体航向，转换放在 IO/应用层，不改变本结构内 _att 定义。
 *
 *   方向余弦 / 四元数
 *     - Cnb, qnb：n ← b，v_n = Cnb * v_b。
 *     - Cbn = Cnb^T：b ← n。
 *
 *   姿态欧拉角
 *     - _att = [pitch, roll, yaw]^T [rad]，与姿态转换工具（a2mat / m2att）一致。
 *
 *   位置
 *     - _pos = [lat, lon, h]^T，lat/lon [rad]，h [m]（大地高）。
 */

/**
 * @brief 名义导航状态（SINS 解）
 *
 * 机械编排的输出；滤波通过误差反馈修正这些量。跨模块共享契约：INS 写入，EKF 读取并反馈。
 */
struct NominalState {
    double _t_cur = 0.0;  ///< 当前时刻 [s]（与传感器时间轴一致）
    double _nts = 0.0;    ///< 本步积分间隔 Δt [s]

    Eigen::Vector3d _att = Eigen::Vector3d::Zero();              ///< 欧拉角 [pitch, roll, yaw]^T [rad]
    Eigen::Matrix3d _Cnb = Eigen::Matrix3d::Identity();          ///< Cnb：n ← b（RFU→ENU），v_n = Cnb * v_b
    Eigen::Matrix3d _Cbn = Eigen::Matrix3d::Identity();          ///< Cbn = Cnb^T：b ← n
    t_quat _qnb;                                                 ///< qnb：n ← b（与 Cnb 等价），默认单位四元数

    Eigen::Vector3d _vn = Eigen::Vector3d::Zero();   ///< ENU 速度 [vE, vN, vU]^T [m/s]
    Eigen::Vector3d _pos = Eigen::Vector3d::Zero();  ///< 大地坐标 [lat, lon, h]^T，lat/lon [rad]，h [m]

    Eigen::Vector3d _eb = Eigen::Vector3d::Zero();   ///< 陀螺零偏 [rad/s]
    Eigen::Vector3d _db = Eigen::Vector3d::Zero();   ///< 加计零偏 [m/s^2]
    Eigen::Vector3d _Kg = Eigen::Vector3d::Ones();   ///< 陀螺三轴刻度因子（无量纲，默认 1）
    Eigen::Vector3d _Ka = Eigen::Vector3d::Ones();   ///< 加计三轴刻度因子（无量纲，默认 1）

    Eigen::Vector3d _lever = Eigen::Vector3d::Zero();  ///< IMU 中心 → 量测传感器相位中心 [m]

    Eigen::Vector3d _wib = Eigen::Vector3d::Zero();  ///< 机体系相对惯性相关角速度中间量 [rad/s]
    Eigen::Vector3d _wnb = Eigen::Vector3d::Zero();  ///< 机体系相对导航系角速度中间量 [rad/s]
    Eigen::Vector3d _web = Eigen::Vector3d::Zero();  ///< 机体系相对地固系角速度中间量 [rad/s]
    Eigen::Vector3d _an = Eigen::Vector3d::Zero();   ///< 导航系加速度中间量 [m/s^2]（ENU）
    Eigen::Vector3d _fn = Eigen::Vector3d::Zero();   ///< 导航系比力 [m/s^2]（ENU）

    // e 系（ECEF）输出量：由 n 系解投影得到，供 ECEF 域使用
    Eigen::Vector3d _pos_ecef = Eigen::Vector3d::Zero();       ///< ECEF 位置 [m]
    Eigen::Vector3d _ve = Eigen::Vector3d::Zero();             ///< ECEF 速度 [m/s]
    Eigen::Vector3d _ae = Eigen::Vector3d::Zero();             ///< ECEF 加速度 [m/s^2]
    Eigen::Matrix3d _Ceb = Eigen::Matrix3d::Identity();        ///< Ceb：e ← b
    Eigen::Matrix3d _Cbe = Eigen::Matrix3d::Identity();        ///< Cbe = Ceb^T
    t_quat _qeb;                                               ///< qeb：e ← b（与 Ceb 等价），默认单位四元数
};

}  // namespace msf
