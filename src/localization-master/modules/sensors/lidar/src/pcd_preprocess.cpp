#include "pcd_preprocess.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "math/constants.h"
#include "math/coordinate.h"

namespace msf {
namespace lidar {

Eigen::Vector3d PositionEnu(const NominalState& nominal, const GlobalConfig& config) {
    const Eigen::Vector3d origin_lla_rad(config._local_map_lla_deg(0) * constants::_D2R,
                                         config._local_map_lla_deg(1) * constants::_D2R,
                                         config._local_map_lla_deg(2));
    return Coordinate::lla2enu(origin_lla_rad, nominal._pos);
}

PointType PointImuToWorld(const PointType& point_imu,
                          const NominalState& nominal,
                          const GlobalConfig& config) {
    const Eigen::Vector3d p_imu(point_imu.x, point_imu.y, point_imu.z);
    const Eigen::Vector3d p_world = PositionEnu(nominal, config) + nominal._Cnb * p_imu;

    PointType point_world;
    point_world.x = static_cast<float>(p_world.x());
    point_world.y = static_cast<float>(p_world.y());
    point_world.z = static_cast<float>(p_world.z());
    point_world.intensity = point_imu.intensity;
    return point_world;
}

PointCloudXYZI::Ptr TransformCloudImuToWorld(const PointCloudXYZI::Ptr& cloud_imu,
                                             const NominalState& nominal,
                                             const GlobalConfig& config) {
    PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI());
    if (!cloud_imu) {
        return cloud_world;
    }
    cloud_world->reserve(cloud_imu->size());
    for (const auto& point_imu : cloud_imu->points) {
        cloud_world->push_back(PointImuToWorld(point_imu, nominal, config));
    }
    return cloud_world;
}

PointCloudXYZI::Ptr TransformLidarCloudToImu(const PointCloudXYZI::Ptr& cloud_lidar,
                                             const GlobalConfig& config) {
    PointCloudXYZI::Ptr cloud_imu(new PointCloudXYZI());
    if (!cloud_lidar) {
        return cloud_imu;
    }

    cloud_imu->reserve(cloud_lidar->size());
    for (const auto& point_lidar : cloud_lidar->points) {
        const Eigen::Vector3d p_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
        const Eigen::Vector3d p_imu = config._calibration._R_bl * p_lidar + config._calibration._T_lb_m;
        PointType point_imu;
        point_imu.x = static_cast<float>(p_imu.x());
        point_imu.y = static_cast<float>(p_imu.y());
        point_imu.z = static_cast<float>(p_imu.z());
        point_imu.intensity = point_lidar.intensity;
        cloud_imu->push_back(point_imu);
    }
    return cloud_imu;
}

bool LoadLidarPcd(const std::string& path,
                  double scan_begin_time,
                  PointCloudXYZI::Ptr cloud,
                  double* scan_end_time) {
    if (!cloud) {
        return false;
    }
    cloud->clear();

    if (path.empty() || !std::filesystem::exists(path)) {
        std::cerr << "LiDAR PCD not found: " << path << std::endl;
        return false;
    }

    if (std::filesystem::path(path).extension() != ".pcd") {
        std::cerr << "Unsupported LiDAR scan format: " << path << std::endl;
        return false;
    }

    pcl::PointCloud<PointXYZIRT> raw_cloud;
    if (pcl::io::loadPCDFile<PointXYZIRT>(path, raw_cloud) != 0) {
        std::cerr << "Failed to load LiDAR PCD: " << path << std::endl;
        cloud->clear();
        return false;
    }

    if (raw_cloud.empty()) {
        return false;
    }

    double min_point_time = std::numeric_limits<double>::max();
    double max_point_time = std::numeric_limits<double>::lowest();
    for (const auto& point : raw_cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.time)) {
            continue;
        }
        if (point.x * point.x + point.y * point.y + point.z * point.z <= 25.0f) {
            continue;
        }
        min_point_time = std::min(min_point_time, static_cast<double>(point.time));
        max_point_time = std::max(max_point_time, static_cast<double>(point.time));
    }

    if (min_point_time > max_point_time) {
        return false;
    }

    cloud->reserve(raw_cloud.size());
    for (const auto& point : raw_cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.time)) {
            continue;
        }
        if (point.x * point.x + point.y * point.y + point.z * point.z <= 25.0f) {
            continue;
        }

        PointType out_point;
        out_point.x = point.x;
        out_point.y = point.y;
        out_point.z = point.z;
        out_point.intensity = static_cast<float>((point.time - min_point_time) * 1000.0);
        cloud->push_back(out_point);
    }

    if (scan_end_time != nullptr) {
        const double scan_duration_sec = std::max(0.0, max_point_time - min_point_time);
        *scan_end_time = scan_begin_time + scan_duration_sec;
    }

    return !cloud->empty();
}

PointCloudXYZI::Ptr FilterRawScan(const PointCloudXYZI::Ptr& raw_cloud,
                                  const GlobalConfig& config) {
    PointCloudXYZI::Ptr skipped(new PointCloudXYZI());
    if (!raw_cloud) {
        return skipped;
    }

    const int point_filter_num = config._lidar._point_filter_num > 0
        ? config._lidar._point_filter_num : 1;
    skipped->reserve(raw_cloud->size() / static_cast<size_t>(point_filter_num) + 1);
    for (size_t i = 0; i < raw_cloud->size(); ++i) {
        if (i % static_cast<size_t>(point_filter_num) == 0) {
            skipped->push_back(raw_cloud->points[i]);
        }
    }

    pcl::VoxelGrid<PointType> downsample_frame;
    const double frame_voxel_size = config._lidar._frame_voxel_size > 0.0
        ? config._lidar._frame_voxel_size : 0.5;
    downsample_frame.setLeafSize(static_cast<float>(frame_voxel_size),
                                 static_cast<float>(frame_voxel_size),
                                 static_cast<float>(frame_voxel_size));
    PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
    downsample_frame.setInputCloud(skipped);
    downsample_frame.filter(*filtered);
    return filtered;
}

namespace {

std::string WorldPcdFrameId(const GlobalConfig& config) {
    return config._lidar_frame_id.empty() ? config._lidar._frame_id : config._lidar_frame_id;
}

std::string WorldPcdFrameDir(const GlobalConfig& config) {
    return (std::filesystem::path(config._file._output_data_dir) / WorldPcdFrameId(config)).string();
}

}  // namespace

std::string WorldPcdOutputPath(const GlobalConfig& config, const LidarScanInfo& scan) {
    return (std::filesystem::path(WorldPcdFrameDir(config)) /
            std::filesystem::path(scan._path).filename()).string();
}

bool EnsureWorldPcdOutputDir(const GlobalConfig& config) {
    if (!config._out._output_world_pcd) {
        return true;
    }
    const std::filesystem::path dir(WorldPcdFrameDir(config));
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        std::cerr << "无法创建世界系 PCD 输出目录: " << dir << ", " << ec.message() << std::endl;
        return false;
    }
    std::cout << "[LiDAR] 世界系 PCD 输出目录: " << dir << std::endl;
    return true;
}

bool ExportWorldPcd(const PointCloudXYZI::Ptr& cloud_lidar,
                    const NominalState& nominal,
                    const GlobalConfig& config,
                    const std::string& output_path,
                    std::string* reject_reason) {
    if (!cloud_lidar || cloud_lidar->empty()) {
        if (reject_reason != nullptr) {
            *reject_reason = "pcd_load_failed";
        }
        return false;
    }

    const PointCloudXYZI::Ptr cloud_imu = TransformLidarCloudToImu(cloud_lidar, config);
    if (!cloud_imu || cloud_imu->empty()) {
        if (reject_reason != nullptr) {
            *reject_reason = "no_points_after_undistort";
        }
        return false;
    }

    PointCloudXYZI::Ptr cloud_world = TransformCloudImuToWorld(cloud_imu, nominal, config);
    if (!cloud_world || cloud_world->empty()) {
        if (reject_reason != nullptr) {
            *reject_reason = "no_points_after_undistort";
        }
        return false;
    }
    cloud_world->width = static_cast<std::uint32_t>(cloud_world->size());
    cloud_world->height = 1;
    cloud_world->is_dense = true;

    const std::filesystem::path output_file_path(output_path);
    std::error_code ec;
    std::filesystem::create_directories(output_file_path.parent_path(), ec);
    if (ec) {
        if (reject_reason != nullptr) {
            *reject_reason = "output_dir_create_failed";
        }
        return false;
    }
    if (pcl::io::savePCDFileBinary(output_path, *cloud_world) != 0) {
        if (reject_reason != nullptr) {
            *reject_reason = "pcd_save_failed";
        }
        return false;
    }
    if (reject_reason != nullptr) {
        reject_reason->clear();
    }
    return true;
}

void MaybeExportWorldPcd(const LidarScanInfo& scan,
                         const PointCloudXYZI::Ptr& cloud_lidar,
                         const NominalState& nominal,
                         const GlobalConfig& config) {
    if (!config._out._output_world_pcd) {
        return;
    }
    std::string reject_reason;
    if (!ExportWorldPcd(cloud_lidar, nominal, config, WorldPcdOutputPath(config, scan),
                        &reject_reason)) {
        std::cerr << "[LiDAR] 世界系 PCD 导出失败: scan=" << scan._timestamp
                  << " reason=" << reject_reason << std::endl;
    }
}

}  // namespace lidar
}  // namespace msf
