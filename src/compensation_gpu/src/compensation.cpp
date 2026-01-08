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
#include <pcl/io/pcd_io.h>

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


extern "C" void transformPointsGPU(float* h_points, float* h_results, float* h_T_wv, float* h_T_vl,  float* h_T_lw, int numPoints, int timestamp_size, int ring_num);
extern "C" void transformPointsGPU_OFFLINE(float* h_points, float* h_results, float* h_T_vl, float* h_T_lw, float* h_ins_pose, int numPoints);

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

struct PointXYZTRI
{
    PCL_ADD_POINT4D;                    // XYZ + padding
    double timestamp;                   // 时间戳
    int32_t ring;
    uint8_t intensity;                    // 强度

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZTRI,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (double, timestamp, timestamp)
    (int32_t, ring, ring)
    (uint8_t, intensity, intensity)
)



class CompensationGPUNode : public rclcpp::Node
{
public:
  CompensationGPUNode() : Node("compensation_node")
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
      "/bynav/inspvax", qos, std::bind(&CompensationGPUNode::ins_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar_front_points", qos, std::bind(&CompensationGPUNode::lidar_callback, this, std::placeholders::_1));

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

      std::cout << ins._timestamp << std::endl;

      std::lock_guard<std::mutex> lock(SingletonDataBuffer::getInstance()._ins_mtx);
      SingletonDataBuffer::getInstance()._ins_buffer.push(ins);
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

      if (SingletonDataBuffer::getInstance()._ins_buffer.cache.size() < 80) {
        return;
      }

      pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>);
      pcl::fromROSMsg(*msg, *cloud);

      int point_size = cloud->points.size();

      if (point_size != 624000) {
        RCLCPP_ERROR(this->get_logger(), "Lidar point num not 624000");
        return ;
      }

      double reference_time = cloud->points.at(point_size - 1).timestamp;

      std::deque<Ins> cache;
      if (1) {
        std::lock_guard<std::mutex> lock(SingletonDataBuffer::getInstance()._ins_mtx);
        cache = SingletonDataBuffer::getInstance()._ins_buffer.cache;
      }

      // RCLCPP_INFO(this->get_logger(), "start ins time: %f", cache[0]._timestamp);
      // RCLCPP_INFO(this->get_logger(), "end ins time: %f", cache[cache.size() - 1]._timestamp);
      // RCLCPP_INFO(this->get_logger(), "lidar last point time: %f", reference_time);

      Pose pose_before;
      Pose pose_after;
      Pose pose;

      if (!BaseTools::get_before_after_pose(pose_before, pose_after, reference_time, cache)) {
        // RCLCPP_ERROR(this->get_logger(), "Lidar time and ins time not alignment, compenation failed");
        return;
      }
      Pose ref_pose = BaseTools::interpolatePose(pose_before, pose_after, reference_time);
      Eigen::Matrix4d T_ref_wv = Eigen::Matrix4d::Identity();
      T_ref_wv.block(0, 0, 3, 3) = ref_pose.orientation.toRotationMatrix();
      T_ref_wv.block(0, 3, 3, 1) = ref_pose.position;

      Eigen::Matrix4d T_ref_lw = T_vl.inverse() * T_ref_wv.inverse();

      
      // 准备gpu数据
      std::vector<float> h_points(point_size * 3);
      std::vector<float> h_results(point_size * 3); 

      std::vector<float> h_T_wv(1200 * 12, 0); // 共有62400 个点， 520个点时间戳一致，需要 1200个转移矩阵
      std::vector<float> h_T_vl(12, 0);
      std::vector<float> h_T_lw(12, 0);

      for (int i = 0; i < 3; i++) {
        h_T_vl[i*4] = T_vl(i, 0);
        h_T_vl[i*4 + 1] = T_vl(i, 1);
        h_T_vl[i*4 + 2] = T_vl(i, 2);
        h_T_vl[i*4 + 3] = T_vl(i, 3);
      }

      for (int i = 0; i < 3; i++) {
        h_T_lw[i*4] = T_ref_lw(i, 0);
        h_T_lw[i*4 + 1] = T_ref_lw(i, 1);
        h_T_lw[i*4 + 2] = T_ref_lw(i, 2);
        h_T_lw[i*4 + 3] = T_ref_lw(i, 3);
      }


      rclcpp::Time ros_time = this->get_clock()->now(); // 获取当前 ROS 时间
      RCLCPP_INFO(this->get_logger(), "start ROS Time: %f", ros_time.seconds());

      for (int i = 0; i < point_size; i++) { 
        auto& point = cloud->points.at(i);
        h_points[i*3] = point.x;
        h_points[i*3 + 1] = point.y;
        h_points[i*3 + 2] = point.z;
        if (i % 520 == 0) {
          double timestamp = point.timestamp;
          if (!BaseTools::get_before_after_pose(pose_before, pose_after, timestamp, cache)) {
            // RCLCPP_ERROR(this->get_logger(), "Lidar time and ins time not alignment, compenation failed");
            return;
          }
          pose = BaseTools::interpolatePose(pose_before, pose_after, timestamp);          
          Eigen::Matrix3d R = pose.orientation.toRotationMatrix();
          
          int col_num = i / 520;
          for (int j = 0; j < 3; j++) {
            h_T_wv[col_num*12 + j*4] = R(j, 0);
            h_T_wv[col_num*12 + j*4 + 1] = R(j, 1);
            h_T_wv[col_num*12 + j*4 + 2] = R(j, 2);
            h_T_wv[col_num*12 + j*4 + 3] = pose.position(j);
          }
        }
      }


      rclcpp::Time ros_time2 = this->get_clock()->now(); // 获取当前 ROS 时间
      RCLCPP_INFO(this->get_logger(), "end ROS Time: %f", ros_time2.seconds());


      transformPointsGPU(h_points.data(), h_results.data(), h_T_wv.data(), h_T_vl.data(),  h_T_lw.data(), 624000, 1200, 520);

      // pcl::io::savePCDFileASCII("/home/zhouwang/dataset/output1.pcd", *cloud);
      for (int i = 0; i < point_size; i++) {
        auto& point = cloud->points.at(i);
        point.x = h_results[i*3];
        point.y = h_results[i*3 + 1];
        point.z = h_results[i*3 + 2];
      }


      
      // pcl::io::savePCDFileASCII("/home/zhouwang/dataset/output2.pcd", *cloud);
  }

  bool lla_initialed = false;
  Eigen::Vector3d init_lla = Eigen::Vector3d(0.0, 0.0, 0.0);

  Eigen::Matrix4d T_vl = Eigen::Matrix4d::Identity();
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::SharedPtr ins_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

};


void lidar_compensation(pcl::PointCloud<PointXYZTRI>::Ptr cloud, const std::vector<Pose>& vec_ins, double reference_time) {

      Eigen::Matrix4d T_vl = Eigen::Matrix4d::Identity();
      int point_size = cloud->points.size();
      double start_time = std::min(cloud->points.at(0).timestamp, reference_time);
      double end_time = std::max(cloud->points.at(point_size - 1).timestamp, reference_time);

      if (vec_ins[0].timestamp > start_time || vec_ins[vec_ins.size() - 1].timestamp < end_time) {
        std::cout << "时间戳错误" << std::endl;
        return;
      }

      int start_ind = 0;
      int end_ind = 0;
      for (int i = 0; i < vec_ins.size(); i++ ) {
        if (vec_ins[i].timestamp > start_time) {
          start_ind = i - 1;
          end_ind = start_ind + 15;
        }
      }

      std::vector<float> h_ins_pose(15 * 8, 0.0); // t, (w,x,y,z) , timestamp;
      for (int i = start_ind; i < end_ind; i++) {
        const Pose& pose = vec_ins[i];
        h_ins_pose[i * 8] = pose.position(0);
        h_ins_pose[i * 8 + 1] = pose.position(1);
        h_ins_pose[i * 8 + 2] = pose.position(2);

        h_ins_pose[i*8 + 3] = pose.orientation.w();
        h_ins_pose[i*8 + 4] = pose.orientation.x();
        h_ins_pose[i*8 + 5] = pose.orientation.y();
        h_ins_pose[i*8 + 6] = pose.orientation.z();
        h_ins_pose[i*8 + 7] = static_cast<float>(pose.timestamp - reference_time);
      }

      Pose pose_before;
      Pose pose_after;

      if (!BaseTools::get_before_after_pose(pose_before, pose_after, reference_time, vec_ins)) {
        // RCLCPP_ERROR(this->get_logger(), "Lidar time and ins time not alignment, compenation failed");
        return;
      }
      Pose ref_pose = BaseTools::interpolatePose(pose_before, pose_after, reference_time);
      Eigen::Matrix4d T_ref_wv = Eigen::Matrix4d::Identity();
      T_ref_wv.block(0, 0, 3, 3) = ref_pose.orientation.toRotationMatrix();
      T_ref_wv.block(0, 3, 3, 1) = ref_pose.position;

      Eigen::Matrix4d T_ref_lw = T_vl.inverse() * T_ref_wv.inverse();

      
      // 准备gpu数据
      std::vector<float> h_points(point_size * 3);
      std::vector<float> h_results(point_size * 4); 

      std::vector<float> h_T_vl(12, 0);
      std::vector<float> h_T_lw(12, 0);

      for (int i = 0; i < 3; i++) {
        h_T_vl[i*4] = T_vl(i, 0);
        h_T_vl[i*4 + 1] = T_vl(i, 1);
        h_T_vl[i*4 + 2] = T_vl(i, 2);
        h_T_vl[i*4 + 3] = T_vl(i, 3);
      }

      for (int i = 0; i < 3; i++) {
        h_T_lw[i*4] = T_ref_lw(i, 0);
        h_T_lw[i*4 + 1] = T_ref_lw(i, 1);
        h_T_lw[i*4 + 2] = T_ref_lw(i, 2);
        h_T_lw[i*4 + 3] = T_ref_lw(i, 3);
      }

      for (int i = 0; i < point_size; i++) { 
        auto& point = cloud->points.at(i);
        h_points[i*4] = point.x;
        h_points[i*4 + 1] = point.y;
        h_points[i*4 + 2] = point.z;
        h_points[i*4 + 3] = point.timestamp - reference_time;
      }

      transformPointsGPU_OFFLINE(h_points.data(), h_results.data(), h_T_vl.data(), h_T_lw.data(), h_ins_pose.data(), point_size);

      for (int i = 0; i < point_size; i++) {
        auto& point = cloud->points.at(i);
        point.x = h_results[i*3];
        point.y = h_results[i*3 + 1];
        point.z = h_results[i*3 + 2];
      }
}


std::vector<Pose> load_ins(std::string file_path) {
  std::ifstream file(file_path.c_str());  // 打开 IMU 数据文件
    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
    }

    Eigen::Vector3d init_lla = Eigen::Vector3d(0.0, 0.0, 0.0);
    lla_initialed = false;

    // 跳过文件头部（第一行）
    std::string header_line;
    std::getline(file, header_line);  // 读取并忽略第一行（头部说明）

    std::vector<Pose> vec_ins;  // 用于存储解析后的 IMU 数据
    std::string line;

    // 逐行读取文件并解析 IMU 数据
    while (std::getline(file, line)) {
        std::istringstream stream(line);

        stream >> ins._timestamp >> ins._latitude >> ins._longitude >> ins._altitude >> ins._roll >> ins._pitch >> ins._azimuth;
        ins._timestamp =/ 1e6;

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
        vec_ins.push_back(pose);
    }
    file.close();

    return vec_ins;
}

int main(int argc, char * argv[])
{
    bool ONLINE = false;
    if (ONLINE) {
      rclcpp::init(argc, argv);
      rclcpp::executors::MultiThreadedExecutor executor;
      auto node = std::make_shared<CompensationGPUNode>();
      executor.add_node(node);
      executor.spin();
      rclcpp::shutdown();
    } else {
      // 加载ins
      std::vector<Pose> vec_ins = load_ins("/home/zhouwang/dataset/raw_data/ins.txt");
      pcl::PointCloud<PointXYZTRI>::Ptr cloud(new pcl::PointCloud<PointXYZTRI>());
      pcl::io::loadPCDFile<PointXYZTRI>("/home/zhouwang/dataset/1766304294799853.pcd", *cloud);

      double reference_time = static_cast<double>(1766304294799853) / 1e6;
      lidar_compensation(cloud, vec_ins, reference_time);


      pcl::io::savePCDFileASCII("/home/zhouwang/dataset/1766304294799853_.pcd", *cloud);


    }



    return 0;
}