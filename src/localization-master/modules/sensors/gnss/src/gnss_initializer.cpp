#include "gnss_initializer.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "math/constants.h"
#include "math/pose_converter.h"
#include "math/utility.h"

namespace msf {
namespace gnss {

namespace {

// 用一组同步观测初始化名义状态（对应 POST_MSF init_sins_state）
bool InitState(const ImuData& imu_data,
               const GnssPos& gnss_pos,
               const GnssPos& second_gnss_pos,
               const GnssVel& gnss_vel,
               const HeadingData& heading_data,
               const Eigen::Vector3d& imu_to_antenna_xyz_m,
               const GlobalConfig::ImuCalib& imu_calib,
               NominalState& nominal,
               earth& eth) {
    nominal._t_cur = imu_data._timestamp;
    nominal._nts = imu_data._dt;
    nominal._lever = imu_to_antenna_xyz_m.array().max(-3.0).min(3.0).matrix();

    // GNSS 速度：低速时用位置差分，否则用地速/航迹角
    const Eigen::Vector3d& blh_curr_rad = gnss_pos._blh;
    Eigen::Vector3d vgnss_n;
    if (gnss_vel._hor_speed < 1.5) {
        const double dt = gnss_pos._timestamp - second_gnss_pos._timestamp;
        if (dt <= 1e-6) {
            return false;
        }

        earth earth_curr;
        earth_curr.Update(blh_curr_rad, Eigen::Vector3d::Zero());
        const Eigen::Vector3d delta_blh = gnss_pos._blh - second_gnss_pos._blh;
        vgnss_n = Eigen::Vector3d(delta_blh(1) / (earth_curr._f_cbRNh * dt),
                                  delta_blh(0) / (earth_curr._f_RMh * dt),
                                  delta_blh(2) / dt);
    } else {
        vgnss_n = Eigen::Vector3d(gnss_vel._hor_speed * std::sin(gnss_vel._trk_gnd),
                                  gnss_vel._hor_speed * std::cos(gnss_vel._trk_gnd),
                                  gnss_vel._ver_speed);
    }

    // 用 heading 初始化姿态
    eth.Update(blh_curr_rad, vgnss_n);
    nominal._att = Eigen::Vector3d(0.0, 0.0, constants::_PI / 2.0 - heading_data._heading);
    nominal._Cnb = pose_converter::a2mat(nominal._att);
    nominal._Cbn = nominal._Cnb.transpose();
    nominal._qnb = pose_converter::a2qua(nominal._att);

    // GNSS POS - lever = IMU 位置
    const Eigen::Vector3d lever_n = nominal._Cnb * nominal._lever;
    const Eigen::Vector3d delta_blh = eth.v2dp(lever_n, 1.0);
    nominal._pos = blh_curr_rad - delta_blh;

    // IMU 速度 = GNSS 速度 - 杆臂旋转速度
    const Eigen::Vector3d web = imu_data._gyro - nominal._Cbn * eth._wnie;
    nominal._vn = vgnss_n - nominal._Cnb * askew(web) * nominal._lever;

    // 初始化时刻导航系地球参数
    eth.Update(nominal._pos, nominal._vn);

    // IMU 零偏与刻度因子
    nominal._Kg = imu_calib._gyro_scale_factor;
    nominal._Ka = imu_calib._acc_scale_factor;
    nominal._eb = imu_calib._gyro_bias_dps * constants::_dps;
    nominal._db = imu_calib._acc_bias_mg * constants::_mg;

    // 中间量与 e 系输出量显式清零（对应 POST_MSF 全新 sins 的初始状态）
    nominal._wib = nominal._wnb = nominal._web = Eigen::Vector3d::Zero();
    nominal._fn = Eigen::Vector3d::Zero();
    nominal._an = Eigen::Vector3d::Zero();
    nominal._pos_ecef = Eigen::Vector3d::Zero();
    nominal._ve = Eigen::Vector3d::Zero();
    nominal._ae = Eigen::Vector3d::Zero();
    nominal._Ceb = Eigen::Matrix3d::Identity();
    nominal._Cbe = Eigen::Matrix3d::Identity();
    nominal._qeb = t_quat();
    return true;
}

}  // namespace

bool InitializeNominal(const std::vector<ImuData>& imu_buffer,
                       const std::vector<GnssPos>& gnsspos_buffer,
                       const std::vector<GnssVel>& gnssvel_buffer,
                       const std::vector<HeadingData>& heading_buffer,
                       const Eigen::Vector3d& imu_to_antenna_xyz_m,
                       const GlobalConfig::ImuCalib& imu_calib,
                       NominalState& nominal,
                       earth& eth) {
    if (imu_buffer.size() < 80 || gnsspos_buffer.size() < 10 ||
        gnssvel_buffer.empty() || heading_buffer.size() < 3) {
        std::cerr << "[INIT GNSS] 数据量不足: imu=" << imu_buffer.size() << " (>=80), "
                  << "gnsspos=" << gnsspos_buffer.size() << " (>=10), "
                  << "gnssvel=" << gnssvel_buffer.size() << " (>=1), "
                  << "heading=" << heading_buffer.size() << " (>2)" << std::endl;
        return false;
    }

    // imu/pos/vel 的 [0, *_end) 表示截至当前 heading 时刻的前缀。
    size_t imu_end = 0;
    size_t pos_end = 0;
    size_t vel_end = 0;

    for (size_t heading_idx = 0; heading_idx < heading_buffer.size(); ++heading_idx) {
        const HeadingData& heading = heading_buffer[heading_idx];
        const double t = heading._timestamp;

        while (imu_end < imu_buffer.size() && imu_buffer[imu_end]._timestamp <= t + 1.0e-6) {
            ++imu_end;
        }
        while (pos_end < gnsspos_buffer.size() && gnsspos_buffer[pos_end]._timestamp <= t + 1.0e-6) {
            ++pos_end;
        }
        while (vel_end < gnssvel_buffer.size() && gnssvel_buffer[vel_end]._timestamp <= t + 1.0e-6) {
            ++vel_end;
        }

        if (imu_end < 80 || pos_end < 10 || vel_end == 0 || heading_idx + 1 < 3) {
            continue;
        }

        bool recent_fixed = true;
        for (size_t i = pos_end - 5; i < pos_end; ++i) {
            if (gnsspos_buffer[i]._pos_type != 50) {
                recent_fixed = false;
                break;
            }
        }
        if (!recent_fixed) {
            continue;
        }

        // 在最近若干 POS 中找与 heading 同步的一条，再取其更老的一条作差分速度。
        size_t pos_idx = pos_end;
        const size_t pos_try = std::min(static_cast<size_t>(4), pos_end);
        for (size_t i = 0; i < pos_try; ++i) {
            const size_t idx = pos_end - 1 - i;
            if (std::fabs(t - gnsspos_buffer[idx]._timestamp) < 0.01) {
                pos_idx = idx;
                break;
            }
        }
        if (pos_idx == 0 || pos_idx == pos_end) {
            continue;
        }
        const size_t pos_prev_idx = pos_idx - 1;

        size_t vel_idx = vel_end;
        for (size_t i = 0; i < vel_end; ++i) {
            const size_t idx = vel_end - 1 - i;
            if (std::fabs(t - gnssvel_buffer[idx]._timestamp) < 0.01) {
                vel_idx = idx;
                break;
            }
        }
        if (vel_idx == vel_end) {
            continue;
        }

        size_t imu_idx = imu_end;
        for (size_t i = 0; i < imu_end; ++i) {
            const size_t idx = imu_end - 1 - i;
            if (std::fabs(t - imu_buffer[idx]._timestamp) < 0.01) {
                imu_idx = idx;
                break;
            }
        }
        if (imu_idx == imu_end) {
            continue;
        }

        if (InitState(imu_buffer[imu_idx],
                      gnsspos_buffer[pos_idx],
                      gnsspos_buffer[pos_prev_idx],
                      gnssvel_buffer[vel_idx],
                      heading,
                      imu_to_antenna_xyz_m,
                      imu_calib,
                      nominal,
                      eth)) {
            return true;
        }
    }

    std::cerr << "[INIT GNSS] 未找到有效初始化点: 已按时间前向尝试 heading="
              << heading_buffer.size() << " 条, imu=" << imu_buffer.size()
              << ", gnsspos=" << gnsspos_buffer.size()
              << ", gnssvel=" << gnssvel_buffer.size() << std::endl;
    return false;
}

}  // namespace gnss
}  // namespace msf
