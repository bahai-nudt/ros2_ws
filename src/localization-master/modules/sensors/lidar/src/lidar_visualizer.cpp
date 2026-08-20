#include "lidar_visualizer.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include "pcd_preprocess.h"

namespace msf {
namespace lidar {

namespace {

const char* WindowTitle(int mode) {
    switch (mode) {
        case 1:
            return "LiDAR raw";
        case 2:
            return "LiDAR undistort";
        default:
            return "LiDAR world";
    }
}

bool HasDisplay() {
    const char* display = std::getenv("DISPLAY");
    const char* wayland = std::getenv("WAYLAND_DISPLAY");
    return (display != nullptr && display[0] != '\0') ||
           (wayland != nullptr && wayland[0] != '\0');
}

}  // namespace

struct LidarVisualizer::State {
    PointCloudXYZI::Ptr accum_cloud{new PointCloudXYZI()};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr traj_cloud{new pcl::PointCloud<pcl::PointXYZRGB>()};
    pcl::visualization::PCLVisualizer::Ptr visualizer;
    bool accum_cloud_added = false;
    bool traj_cloud_added = false;
    bool camera_initialized = false;
    Eigen::Vector3d smoothed_forward = Eigen::Vector3d::UnitY();
    Eigen::Vector3d smoothed_camera_pos = Eigen::Vector3d::Zero();
};

LidarVisualizer::LidarVisualizer() : state_(std::make_unique<State>()) {}
LidarVisualizer::~LidarVisualizer() = default;

PointCloudXYZI::Ptr VisualizationCloud(int mode,
                                       const PointCloudXYZI::Ptr& raw_lidar,
                                       const PointCloudXYZI::Ptr& undistorted_lidar,
                                       const PointCloudXYZI::Ptr& cloud_imu,
                                       const NominalState& nominal,
                                       const GlobalConfig& config) {
    if (mode == 1) {
        return FilterRawScan(raw_lidar ? raw_lidar : undistorted_lidar, config);
    }
    if (mode == 2) {
        return FilterRawScan(undistorted_lidar ? undistorted_lidar : raw_lidar, config);
    }
    if (mode >= 3) {
        if (cloud_imu && !cloud_imu->empty()) {
            return TransformCloudImuToWorld(cloud_imu, nominal, config);
        }
        const PointCloudXYZI::Ptr src = undistorted_lidar ? undistorted_lidar : raw_lidar;
        return TransformCloudImuToWorld(TransformLidarCloudToImu(FilterRawScan(src, config), config),
                                        nominal, config);
    }
    return PointCloudXYZI::Ptr(new PointCloudXYZI());
}

void LidarVisualizer::EnsureWindow(int mode) {
    if (state_->visualizer) {
        return;
    }
    state_->visualizer.reset(new pcl::visualization::PCLVisualizer(WindowTitle(mode)));
    state_->visualizer->setBackgroundColor(0.0, 0.0, 0.0);
    state_->visualizer->initCameraParameters();
    state_->visualizer->setCameraPosition(0.0, -20.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
}

void LidarVisualizer::UpdateDisplayedCloud(const PointCloudXYZI::Ptr& cloud, bool accumulate) {
    constexpr std::size_t kMaxAccumPoints = 400000;
    if (accumulate) {
        *state_->accum_cloud += *cloud;
        if (state_->accum_cloud->size() > kMaxAccumPoints) {
            const std::size_t overflow = state_->accum_cloud->size() - kMaxAccumPoints;
            state_->accum_cloud->points.erase(state_->accum_cloud->points.begin(),
                                              state_->accum_cloud->points.begin() +
                                                  static_cast<std::ptrdiff_t>(overflow));
        }
    } else {
        *state_->accum_cloud = *cloud;
    }
    state_->accum_cloud->width = static_cast<std::uint32_t>(state_->accum_cloud->size());
    state_->accum_cloud->height = 1;
    state_->accum_cloud->is_dense = true;

    pcl::visualization::PointCloudColorHandlerGenericField<PointType> color_handler(
        state_->accum_cloud, "z");
    if (!state_->accum_cloud_added) {
        if (color_handler.isCapable()) {
            state_->visualizer->addPointCloud<PointType>(state_->accum_cloud, color_handler,
                                                         "accum_cloud");
        } else {
            pcl::visualization::PointCloudColorHandlerCustom<PointType> fallback_color(
                state_->accum_cloud, 0, 255, 0);
            state_->visualizer->addPointCloud<PointType>(state_->accum_cloud, fallback_color,
                                                         "accum_cloud");
        }
        state_->visualizer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "accum_cloud");
        state_->accum_cloud_added = true;
    } else if (color_handler.isCapable()) {
        state_->visualizer->updatePointCloud<PointType>(state_->accum_cloud, color_handler,
                                                        "accum_cloud");
    } else {
        state_->visualizer->updatePointCloud<PointType>(state_->accum_cloud, "accum_cloud");
    }
}

void LidarVisualizer::UpdateWorldFollowCamera(const NominalState& nominal,
                                              const GlobalConfig& config) {
    const Eigen::Vector3d pos_lidar =
        PositionEnu(nominal, config) + nominal._Cnb * config._calibration._T_lb_m;
    pcl::PointXYZRGB traj_point;
    traj_point.x = static_cast<float>(pos_lidar.x());
    traj_point.y = static_cast<float>(pos_lidar.y());
    traj_point.z = static_cast<float>(pos_lidar.z());
    traj_point.r = 0;
    traj_point.g = 255;
    traj_point.b = 255;
    state_->traj_cloud->points.push_back(traj_point);

    if (!state_->traj_cloud_added) {
        state_->visualizer->addPointCloud<pcl::PointXYZRGB>(state_->traj_cloud, "traj_cloud");
        state_->visualizer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "traj_cloud");
        state_->traj_cloud_added = true;
    } else {
        state_->visualizer->updatePointCloud<pcl::PointXYZRGB>(state_->traj_cloud, "traj_cloud");
    }

    constexpr double kFollowBack = 20.0;
    constexpr double kFollowUp = 5.0;
    constexpr double kLookAhead = 4.0;
    constexpr double kDirSmoothing = 0.15;
    constexpr double kPosSmoothing = 0.12;
    Eigen::Vector3d forward = nominal._Cnb * Eigen::Vector3d::UnitY();
    if (forward.norm() < 1.0e-9) {
        forward = Eigen::Vector3d::UnitY();
    } else {
        forward.normalize();
    }

    const Eigen::Vector3d up(0.0, 0.0, 1.0);
    const Eigen::Vector3d target = pos_lidar;

    if (!state_->camera_initialized) {
        state_->smoothed_forward = forward;
        state_->smoothed_camera_pos = target - kFollowBack * forward + kFollowUp * up;
        state_->camera_initialized = true;
    } else {
        state_->smoothed_forward =
            (1.0 - kDirSmoothing) * state_->smoothed_forward + kDirSmoothing * forward;
        if (state_->smoothed_forward.norm() < 1.0e-9) {
            state_->smoothed_forward = forward;
        } else {
            state_->smoothed_forward.normalize();
        }
        const Eigen::Vector3d desired_camera_pos =
            target - kFollowBack * state_->smoothed_forward + kFollowUp * up;
        state_->smoothed_camera_pos =
            (1.0 - kPosSmoothing) * state_->smoothed_camera_pos + kPosSmoothing * desired_camera_pos;
    }

    const Eigen::Vector3d camera_pos = state_->smoothed_camera_pos;
    const Eigen::Vector3d look_at = target + kLookAhead * state_->smoothed_forward;
    state_->visualizer->setCameraPosition(camera_pos.x(), camera_pos.y(), camera_pos.z(),
                                          look_at.x(), look_at.y(), look_at.z(),
                                          up.x(), up.y(), up.z());
}

void LidarVisualizer::ShowScan(const PointCloudXYZI::Ptr& raw_lidar,
                               const PointCloudXYZI::Ptr& undistorted_lidar,
                               const PointCloudXYZI::Ptr& cloud_imu,
                               const NominalState& nominal,
                               const GlobalConfig& config,
                               int mode) {
    if (mode <= 0 || !HasDisplay()) {
        return;
    }

    const PointCloudXYZI::Ptr cloud =
        VisualizationCloud(mode, raw_lidar, undistorted_lidar, cloud_imu, nominal, config);
    if (!cloud || cloud->empty()) {
        return;
    }

    EnsureWindow(mode);
    const bool world_mode = (mode >= 3);
    UpdateDisplayedCloud(cloud, world_mode);
    if (world_mode) {
        UpdateWorldFollowCamera(nominal, config);
    } else if (!state_->camera_initialized) {
        state_->visualizer->setCameraPosition(0.0, -20.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        state_->camera_initialized = true;
    }
    state_->visualizer->spinOnce(1);
}

}  // namespace lidar
}  // namespace msf
