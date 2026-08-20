#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Eigen>

#include "types/sensors/imu_message.h"

namespace msf {

/**
 * 在 IMU 缓冲区内按时间采样角速度/加速度（最近点或线性插值）。
 * 对应 POST_MSF SampleImuRateAtTimeFromBuffer。
 */
inline bool SampleImuRateAtTimeFromBuffer(const std::vector<ImuData>& imu_buffer,
                                          double sample_time,
                                          Eigen::Vector3d& gyro,
                                          Eigen::Vector3d& accel) {
    if (imu_buffer.empty()) {
        return false;
    }
    const double tol = 1.0e-6;
    const double front_time = imu_buffer.front()._timestamp;
    const double back_time = imu_buffer.back()._timestamp;
    if (sample_time <= front_time - tol || sample_time >= back_time + tol) {
        return false;
    }

    size_t right_idx = 0;
    size_t right_bound = imu_buffer.size();
    while (right_idx < right_bound) {
        const size_t mid = right_idx + (right_bound - right_idx) / 2;
        if (imu_buffer[mid]._timestamp < sample_time) {
            right_idx = mid + 1;
        } else {
            right_bound = mid;
        }
    }

    const ImuData* nearest = nullptr;
    if (right_idx < imu_buffer.size()) {
        nearest = &imu_buffer[right_idx];
    }
    if (right_idx > 0) {
        const ImuData& left = imu_buffer[right_idx - 1];
        if (nearest == nullptr ||
            std::fabs(sample_time - left._timestamp) <
                std::fabs(sample_time - nearest->_timestamp)) {
            nearest = &left;
        }
    }
    if (nearest != nullptr && std::fabs(sample_time - nearest->_timestamp) <= tol) {
        gyro = nearest->_gyro;
        accel = nearest->_accel;
        return true;
    }

    if (right_idx == 0 || right_idx >= imu_buffer.size()) {
        return false;
    }
    const ImuData& left = imu_buffer[right_idx - 1];
    const ImuData& right = imu_buffer[right_idx];
    const double dt = right._timestamp - left._timestamp;
    if (dt <= 1.0e-9) {
        return false;
    }
    const double alpha = (sample_time - left._timestamp) / dt;
    gyro = left._gyro + alpha * (right._gyro - left._gyro);
    accel = left._accel + alpha * (right._accel - left._accel);
    return true;
}

}  // namespace msf
