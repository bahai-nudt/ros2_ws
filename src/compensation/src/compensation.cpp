#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"

#include "base_utils/coordinate.h"
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/qos.hpp"
#include "rclcpp/executors.hpp"

#include "compensation/compensation_data_buffer.h"
#include "base_utils/coordinate.h"
#include "base_utils/tools.h"

#include <cmath>
#include <mutex>
#include <iomanip>
#include <fstream>

struct PointXYZIRT
{
    PCL_ADD_POINT4D;                    // XYZ + padding
    float intensity;                    // 强度
    uint16_t ring;
    double timestamp;                   // 时间戳

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

class CompensationNode : public rclcpp::Node
{
public:
  CompensationNode() : Node("compensation_node")
  {
    init();
  }

  bool init()
  {
    initIO();
    return true;
  }

  bool initIO() {

    rclcpp::QoS qos(10);
    qos.reliable();
    qos.keep_last(10);

    ins_sub_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVAX>(
      "/bynav/inspvax", qos, std::bind(&CompensationNode::ins_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar_front_points", qos, std::bind(&CompensationNode::lidar_callback, this, std::placeholders::_1));

    return true;
  }

  void ins_callback(const novatel_oem7_msgs::msg::INSPVAX::SharedPtr msg)
  {
      Ins ins;
      ins._longitude = msg->longitude;
      ins._latitude = msg->latitude;
      ins._altitude = msg->height;
      ins._azimuth = msg->azimuth;
      ins._roll = msg->roll;
      ins._pitch = msg->pitch;
      ins._status = msg->pos_type.type;
      ins._timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec / 1e9);
      if (!lla_initialed) {
        init_lla << ins._longitude, ins._latitude, ins._altitude;
        lla_initialed = true;
        return ;
      }

      std::vector<double> local_enu = Coordinate::lla2enu(init_lla(0), init_lla(1), init_lla(2), ins._longitude, ins._latitude, ins._altitude);
      double yaw = 90.0 - msg->azimuth;

      // 规范化到 [0, 360) 范围
      while (yaw < 0.0) yaw += 360.0;
      while (yaw >= 360.0) yaw -= 360.0;

      Eigen::AngleAxisd rollAngle(msg->roll / 180 * M_PI, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd pitchAngle(msg->pitch / 180 * M_PI, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd yawAngle(yaw / 180 * M_PI, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
      Eigen::Vector3d pos(local_enu[0], local_enu[1], local_enu[2]);

      Pose pose(ins._timestamp, pos, q);
      ins._pose = pose;

      std::lock_guard<std::mutex> lock(SingletonDataBuffer::getInstance()._ins_mtx);
      SingletonDataBuffer::getInstance()._ins_buffer.push(ins);
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

      if (SingletonDataBuffer::getInstance()._ins_buffer.cache.size() < 80) {
        return;
      }

      std::cout << "lidar call back " << std::endl;

      double reference_time = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec / 1e9);

      pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>);
      pcl::fromROSMsg(*msg, *cloud);

      if (cloud->points.size() == 0) {
        return ;
      }

      std::deque<Ins> cache;
      if (1) {
        std::lock_guard<std::mutex> lock(SingletonDataBuffer::getInstance()._ins_mtx);
        cache = SingletonDataBuffer::getInstance()._ins_buffer.cache;
      }


      Pose pose_before;
      Pose pose_after;
      Pose pose;

      if (!BaseTools::get_before_after_pose(pose_before, pose_after, reference_time, cache)) {
        return;
      }
      Pose ref_pose = BaseTools::interpolatePose(pose_before, pose_after, reference_time);
      Eigen::Isometry3d T_ref = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d T_point = Eigen::Isometry3d::Identity();

      T_ref.translate(ref_pose.position);
      T_ref.rotate(ref_pose.orientation);
      
      // 相对变换：从点时刻到参考时刻
      Eigen::Isometry3d T_rel;

      int ind = 0;
      for (auto& point : cloud->points) {
        if (ind++ % 520 == 0) {
          double timestamp = point.timestamp;
          if (!BaseTools::get_before_after_pose(pose_before, pose_after, timestamp, cache)) {
            continue;
          }
          pose = BaseTools::interpolatePose(pose_before, pose_after, timestamp);
          T_point.translate(pose.position);
          T_point.rotate(pose.orientation);

          T_rel = T_ref.inverse() * T_point;
        }

        // 应用变换
        Eigen::Vector3d original_point(point.x, point.y, point.z);
        Eigen::Vector3d compensated_point = T_rel * original_point;

        point.x = compensated_point(0);
        point.y = compensated_point(1);
        point.z = compensated_point(2);
      }

      // std::ostringstream oss;
      // oss << std::fixed << std::setprecision(6) << reference_time;
      // std::string file_name = oss.str();

      // pcl::io::savePCDFileBinary("/home/zhouwang/dataset/jianlong01_brt105e101_20251223_131708/meta_data/" + file_name + ".pcd", *cloud);
  }

  bool lla_initialed = false;
  Eigen::Vector3d init_lla = Eigen::Vector3d(0.0, 0.0, 0.0);

  Eigen::Matrix4d T_vl;
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::SharedPtr ins_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<CompensationNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}