#include "scan_deskewer.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "math/constants.h"
#include "math/coordinate.h"
#include "math/imu_sampler.h"
#include "pcd_preprocess.h"

namespace msf {
namespace lidar {
namespace {

Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& rotation_vector) {
    const double angle = rotation_vector.norm();
    if (angle < 1.0e-12) {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d& rotation) {
    Eigen::AngleAxisd angle_axis(rotation);
    return angle_axis.angle() * angle_axis.axis();
}

Eigen::Vector3d InterpolatePositionHermite(const DeskewPoseSample& head,
                                           const DeskewPoseSample& tail,
                                           double alpha) {
    const double segment_dt = tail._timestamp - head._timestamp;
    if (segment_dt <= 1.0e-9) {
        return head._position_enu;
    }

    alpha = std::max(0.0, std::min(1.0, alpha));
    const double alpha2 = alpha * alpha;
    const double alpha3 = alpha2 * alpha;
    const double h00 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0;
    const double h10 = alpha3 - 2.0 * alpha2 + alpha;
    const double h01 = -2.0 * alpha3 + 3.0 * alpha2;
    const double h11 = alpha3 - alpha2;
    return h00 * head._position_enu +
           h10 * segment_dt * head._velocity_enu +
           h01 * tail._position_enu +
           h11 * segment_dt * tail._velocity_enu;
}

}  // namespace

ScanDeskewer::ScanDeskewer(const GlobalConfig& config, InertialPropagator& propagator)
    : config_(config), propagator_(propagator) {}

DeskewPoseSample ScanDeskewer::BuildPoseSample(const NominalState& nominal) const {
    DeskewPoseSample sample;
    sample._timestamp = nominal._t_cur;
    sample._position_enu = PositionEnu(nominal, config_);
    sample._velocity_enu = nominal._vn;
    sample._acceleration_enu = nominal._an;
    sample._angular_velocity_body = nominal._wib;
    sample._Cnb = nominal._Cnb;
    return sample;
}

bool ScanDeskewer::BuildPoseChain(const NominalState& scan_begin,
                                  const std::vector<ImuData>& imu_buffer,
                                  const LidarScanInfo& scan) {
    poses_.clear();
    poses_.reserve(16);

    NominalState temp = scan_begin;
    earth eth;
    eth.Update(temp._pos, temp._vn);
    poses_.push_back(BuildPoseSample(temp));

    if (imu_buffer.empty() || scan._end_time <= scan._timestamp + 1.0e-9) {
        return !poses_.empty();
    }

    while (temp._t_cur + 1.0e-6 < scan._end_time) {
        double segment_end_time = scan._end_time;

        size_t seg_idx = 0;
        size_t seg_bound = imu_buffer.size();
        while (seg_idx < seg_bound) {
            const size_t mid = seg_idx + (seg_bound - seg_idx) / 2;
            if (imu_buffer[mid]._timestamp <= temp._t_cur) {
                seg_idx = mid + 1;
            } else {
                seg_bound = mid;
            }
        }
        if (seg_idx < imu_buffer.size()) {
            segment_end_time = std::min(segment_end_time, imu_buffer[seg_idx]._timestamp);
        }

        const double segment_dt = segment_end_time - temp._t_cur;
        if (segment_dt <= 1.0e-6) {
            break;
        }

        Eigen::Vector3d gyro_start, accel_start, gyro_end, accel_end;
        if (!SampleImuRateAtTimeFromBuffer(imu_buffer, temp._t_cur, gyro_start, accel_start) ||
            !SampleImuRateAtTimeFromBuffer(imu_buffer, segment_end_time, gyro_end, accel_end)) {
            break;
        }

        if (!propagator_.Propagate(temp, eth, gyro_start, accel_start,
                                   gyro_end, accel_end, segment_dt)) {
            break;
        }
        poses_.push_back(BuildPoseSample(temp));
    }

    return !poses_.empty();
}

bool ScanDeskewer::PoseAt(double point_time, DeskewPoseSample& pose) const {
    if (poses_.empty()) {
        return false;
    }
    if (poses_.size() == 1 || point_time <= poses_.front()._timestamp + 1.0e-9) {
        pose = poses_.front();
        return true;
    }
    if (point_time >= poses_.back()._timestamp - 1.0e-9) {
        pose = poses_.back();
        return true;
    }

    for (size_t i = 1; i < poses_.size(); ++i) {
        const DeskewPoseSample& head = poses_[i - 1];
        const DeskewPoseSample& tail = poses_[i];
        if (point_time > tail._timestamp + 1.0e-9) {
            continue;
        }
        const double segment_dt = tail._timestamp - head._timestamp;
        const double alpha = segment_dt > 1.0e-9
            ? std::max(0.0, std::min(1.0, (point_time - head._timestamp) / segment_dt))
            : 0.0;
        const double dt_point = point_time - head._timestamp;

        pose._timestamp = point_time;
        pose._position_enu = InterpolatePositionHermite(head, tail, alpha);
        pose._velocity_enu = head._velocity_enu + alpha * (tail._velocity_enu - head._velocity_enu);
        pose._acceleration_enu =
            head._acceleration_enu + alpha * (tail._acceleration_enu - head._acceleration_enu);
        pose._Cnb = head._Cnb * ExpSO3(head._angular_velocity_body * dt_point);
        pose._angular_velocity_body = head._angular_velocity_body;
        return true;
    }
    return false;
}

PointCloudXYZI::Ptr ScanDeskewer::DeskewScan(const PointCloudXYZI::Ptr& raw_cloud,
                                             const std::vector<ImuData>& imu_buffer,
                                             const LidarScanInfo& scan,
                                             const NominalState& scan_begin,
                                             DeskewDiag* diag) {
    PointCloudXYZI::Ptr undistorted(new PointCloudXYZI());
    if (!raw_cloud) {
        return undistorted;
    }

    BuildPoseChain(scan_begin, imu_buffer, scan);
    if (poses_.empty()) {
        *undistorted = *raw_cloud;
        return undistorted;
    }

    std::vector<DeskewPoseSample> poses = poses_;
    std::sort(poses.begin(), poses.end(), [](const auto& lhs, const auto& rhs) {
        return lhs._timestamp < rhs._timestamp;
    });
    poses.erase(std::unique(poses.begin(), poses.end(), [](const auto& lhs, const auto& rhs) {
        return std::fabs(lhs._timestamp - rhs._timestamp) < 1.0e-9;
    }), poses.end());
    if (poses.size() < 2) {
        *undistorted = *raw_cloud;
        return undistorted;
    }

    DeskewPoseSample reference_pose = poses.front();
    if (std::fabs(reference_pose._timestamp - scan._timestamp) > 1.0e-6) {
        reference_pose._timestamp = scan._timestamp;
        reference_pose._position_enu = PositionEnu(scan_begin, config_);
        reference_pose._velocity_enu = scan_begin._vn;
        reference_pose._acceleration_enu = scan_begin._an;
        reference_pose._angular_velocity_body = scan_begin._wib;
        reference_pose._Cnb = scan_begin._Cnb;
        poses.insert(poses.begin(), reference_pose);
    }

    const Eigen::Matrix3d R_lb = config_._calibration._R_bl.transpose();

    PointCloudXYZI sorted_cloud = *raw_cloud;
    std::sort(sorted_cloud.points.begin(), sorted_cloud.points.end(), [](const PointType& lhs, const PointType& rhs) {
        return lhs.intensity < rhs.intensity;
    });

    double max_compensation_m = 0.0;
    double sum_compensation_m = 0.0;
    int compensated_count = 0;

    double max_rot_diff_rad = 0.0;
    double max_rot_diff_rel_time_ms = 0.0;
    Eigen::Vector3d max_rot_diff_vec = Eigen::Vector3d::Zero();

    undistorted->resize(sorted_cloud.size());
    int point_index = static_cast<int>(sorted_cloud.size()) - 1;
    for (int pose_index = static_cast<int>(poses.size()) - 1; pose_index > 0 && point_index >= 0; --pose_index) {
        const DeskewPoseSample& head = poses[static_cast<size_t>(pose_index - 1)];
        const DeskewPoseSample& tail = poses[static_cast<size_t>(pose_index)];

        for (; point_index >= 0; --point_index) {
            const PointType& point_lidar = sorted_cloud.points[static_cast<size_t>(point_index)];
            const double point_time = scan._timestamp + static_cast<double>(point_lidar.intensity) * 0.001;
            if (point_time <= head._timestamp && pose_index > 1) {
                break;
            }

            const double segment_dt = tail._timestamp - head._timestamp;
            const double dt_point = point_time - head._timestamp;
            const double alpha = segment_dt > 1.0e-9
                ? std::max(0.0, std::min(1.0, dt_point / segment_dt))
                : 0.0;

            const Eigen::Matrix3d R_i = head._Cnb * ExpSO3(head._angular_velocity_body * dt_point);

            const Eigen::Matrix3d R_delta = head._Cnb.transpose() * tail._Cnb;
            const Eigen::Matrix3d R_i_old = head._Cnb * ExpSO3(LogSO3(R_delta) * alpha);
            const Eigen::Vector3d rot_diff_vec = LogSO3(R_i.transpose() * R_i_old);
            const double rot_diff_rad = rot_diff_vec.norm();
            if (rot_diff_rad > max_rot_diff_rad) {
                max_rot_diff_rad = rot_diff_rad;
                max_rot_diff_vec = rot_diff_vec;
                max_rot_diff_rel_time_ms = static_cast<double>(point_lidar.intensity);
            }

            const Eigen::Vector3d p_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
            const Eigen::Vector3d T_ei = InterpolatePositionHermite(head, tail, alpha) -
                                         reference_pose._position_enu;
            const Eigen::Vector3d p_compensated =
                R_lb * (reference_pose._Cnb.transpose() *
                        (R_i * (config_._calibration._R_bl * p_lidar + config_._calibration._T_lb_m) + T_ei) -
                        config_._calibration._T_lb_m);

            PointType out_point;
            out_point.x = static_cast<float>(p_compensated.x());
            out_point.y = static_cast<float>(p_compensated.y());
            out_point.z = static_cast<float>(p_compensated.z());
            out_point.intensity = point_lidar.intensity;
            (*undistorted)[static_cast<size_t>(point_index)] = out_point;

            const Eigen::Vector3d compensation_vec = p_compensated - p_lidar;
            const double compensation_m = compensation_vec.norm();
            if (compensation_m > max_compensation_m) {
                max_compensation_m = compensation_m;
            }
            sum_compensation_m += compensation_m;
            ++compensated_count;
        }
    }

    for (; point_index >= 0; --point_index) {
        (*undistorted)[static_cast<size_t>(point_index)] = sorted_cloud.points[static_cast<size_t>(point_index)];
    }

    const double mean_compensation_m = compensated_count > 0
        ? sum_compensation_m / static_cast<double>(compensated_count)
        : 0.0;

    if (diag != nullptr) {
        diag->_max_rot_diff_rad = max_rot_diff_rad;
        diag->_max_rot_diff_rel_time_ms = max_rot_diff_rel_time_ms;
        diag->_max_rot_diff_vec = max_rot_diff_vec;
        diag->_max_compensation_m = max_compensation_m;
        diag->_mean_compensation_m = mean_compensation_m;
    }

    return undistorted;
}

}  // namespace lidar
}  // namespace msf
