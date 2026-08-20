#include "nominal_advance.h"

#include <algorithm>
#include <cmath>

#include "math/imu_sampler.h"

namespace msf {
namespace tools {

bool AdvanceNominal(Optimizer& optimizer, InertialPropagator& propagator,
                    const std::vector<ImuData>& imu_buffer, double target_time,
                    double* advanced_dt) {
    if (advanced_dt != nullptr) {
        *advanced_dt = 0.0;
    }
    if (imu_buffer.empty()) {
        return false;
    }

    NominalState& nominal = optimizer.Nominal();
    earth& eth = optimizer.Earth();
    const double tol = 1.0e-6;
    const double t_cur = nominal._t_cur;
    if (target_time + tol < t_cur) {
        return false;
    }
    if (imu_buffer.back()._timestamp + tol < t_cur ||
        imu_buffer.front()._timestamp - tol > target_time) {
        return false;
    }

    double advanced = 0.0;
    while (nominal._t_cur + 1.0e-6 < target_time) {
        double segment_end_time = target_time;

        size_t seg_idx = 0;
        size_t seg_bound = imu_buffer.size();
        while (seg_idx < seg_bound) {
            const size_t mid = seg_idx + (seg_bound - seg_idx) / 2;
            if (imu_buffer[mid]._timestamp <= nominal._t_cur) {
                seg_idx = mid + 1;
            } else {
                seg_bound = mid;
            }
        }
        if (seg_idx < imu_buffer.size()) {
            segment_end_time = std::min(segment_end_time, imu_buffer[seg_idx]._timestamp);
        }

        const double segment_dt = segment_end_time - nominal._t_cur;
        if (segment_dt <= tol) {
            if (segment_end_time > nominal._t_cur) {
                nominal._t_cur = segment_end_time;
                continue;
            }
            break;
        }

        Eigen::Vector3d gyro_start, accel_start, gyro_end, accel_end;
        if (!SampleImuRateAtTimeFromBuffer(imu_buffer, nominal._t_cur, gyro_start, accel_start) ||
            !SampleImuRateAtTimeFromBuffer(imu_buffer, segment_end_time, gyro_end, accel_end)) {
            return false;
        }

        if (!propagator.Propagate(nominal, eth, gyro_start, accel_start,
                                  gyro_end, accel_end, segment_dt)) {
            return false;
        }
        if (!optimizer.TimeUpdate(segment_dt, 1.0)) {
            return false;
        }
        advanced += std::fabs(segment_dt);
    }

    if (advanced_dt != nullptr) {
        *advanced_dt = advanced;
    }

    if (std::abs(nominal._t_cur - target_time) <= tol) {
        if (target_time > nominal._t_cur) {
            nominal._t_cur = target_time;
        }
        return true;
    }
    return false;
}

}  // namespace tools
}  // namespace msf
