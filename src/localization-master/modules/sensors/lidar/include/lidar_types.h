#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace msf {
namespace lidar {

using PointType = pcl::PointXYZI;
using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

struct EIGEN_ALIGN16 PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t ring;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct LidarPlaneConstraint {
    size_t point_index = 0;
    Eigen::Vector3d point_imu = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_enu = Eigen::Vector3d::Zero();
    double plane_d = 0.0;
};

struct LidarFeatureDiag {
    int filtered_points = 0;
    int candidate_features = 0;
    int effective_features = 0;
    double iterated_update_noise_var = 0.0;
};

}  // namespace lidar
}  // namespace msf

POINT_CLOUD_REGISTER_POINT_STRUCT(msf::lidar::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, ring, ring)
    (double, time, timestamp)
)
