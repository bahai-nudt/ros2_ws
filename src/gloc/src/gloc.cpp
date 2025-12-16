#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "nav_msgs/msg/path.hpp"
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/executors.hpp"

#include "gloc/initializer.h"
#include "gloc/gps_processor.h"
#include "gloc/imu_processor.h"
#include "gloc/message.h"
#include "gloc/gloc_data_buffer.h"
#include "base_utils/coordinate.h"

#include <cmath>
#include <mutex>
#include <iomanip>
#include <fstream>


class GlocNode : public rclcpp::Node
{
public:
  GlocNode() : Node("gloc_node")
  {
    init();
    thread_ = std::thread([this]() { this->run(); });
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

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bynav/imu/data_raw", qos, std::bind(&GlocNode::imu_callback, this, std::placeholders::_1));

    ins_sub_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "/bynav/inspva", qos, std::bind(&GlocNode::ins_callback, this, std::placeholders::_1));


    gps_sub_ = this->create_subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>(
      "/bynav/bestgnsspos", 10, std::bind(&GlocNode::gps_callback, this, std::placeholders::_1));

    heading_sub_ = this->create_subscription<novatel_oem7_msgs::msg::HEADING2>(
      "/bynav/heading2", qos, std::bind(&GlocNode::heading_callback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);


    return true;
  }

  bool proc()
  {
    if (_initialed) {
      _message.header.stamp = this->get_clock()->now();
      _message.header.frame_id = "map";

      _message.poses.clear();
      for (const auto& pose : trajectory_) {
          _message.poses.push_back(pose);
      }

      path_pub_->publish(_message);

      std::fstream fs("output2.txt", std::ios::in | std::ios::out | std::ios::app);
      fs << _state._position(0) << "," << _state._position(1) << "," << _state._position(2) << "\n";
      fs.close();

      return true;
    } else {
      return false;
    }

  }

  void run()
  {
    std::shared_ptr<rclcpp::Rate> rate = std::make_shared<rclcpp::Rate>(100);
    while (rclcpp::ok()) {
      rate->sleep();
      proc();
    }
  }  

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ImuData imu_data;
    
    imu_data._timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    imu_data._accel << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    imu_data._gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    SingletonDataBuffer::getInstance()._imu_buffer.push(imu_data);

    if (!_initialed) {
      return;
    } else {
      std::lock_guard<std::mutex> lock(_state_mtx);
      _imu_processor.predict(imu_data, _state);

      // 创建轨迹点
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = _state._position(0);
      pose.pose.position.y = _state._position(1);
      pose.pose.position.z = _state._position(2);

      Eigen::Quaterniond q(_state._rotation);

      pose.pose.orientation.w = q.w();
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();

      trajectory_.push_back(pose);

      // 如果轨迹大小超过最大值，移除最旧的点
      if (trajectory_.size() > 1000) {
          trajectory_.pop_front();  // 移除最旧的轨迹点
      }
      
  
    }
  }

  void gps_callback(const novatel_oem7_msgs::msg::BESTGNSSPOS::SharedPtr msg)
  {
    GpsData gps_data;
    gps_data._timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;                   
    gps_data._lla << msg->lon, msg->lat, msg->hgt;
    gps_data._cov << msg->lon_stdev, 0, 0,
                     0, msg->lat_stdev, 0,
                     0, 0, msg->hgt_stdev;

    gps_data._status = msg->pos_type.type;

    if (gps_data._status != 50) {
      return;
    }


    // std::cout << "gps type:  " << gps_data._status << std::endl;

    if (!_initialed) {
      std::lock_guard<std::mutex> lock(_state_mtx);
      if (_initializer.init_by_gps(gps_data, _state)) {
        _initialed = true;
      }
    } else {
      std::lock_guard<std::mutex> lock(_state_mtx);
      _gps_processor.update(gps_data, _state);

      std::vector<double> local_coor = Coordinate::lla2enu(_state._init_lla(0), _state._init_lla(1), _state._init_lla(2), gps_data._lla(0), gps_data._lla(1), gps_data._lla(2));

      std::fstream fs("gps.txt", std::ios::in | std::ios::out | std::ios::app);
      fs << local_coor[0] << "," << local_coor[1] << "," << local_coor[2] << "\n";
      fs.close();
    }

    SingletonDataBuffer::getInstance()._gps_buffer.push(gps_data);
  }


  void ins_callback(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg)
  {
    if (!_initialed) {
      ;
    } else {
      double longitude = msg->longitude;
      double latitude = msg->latitude;
      double altitude = msg->height;

      std::vector<double> local_coor = Coordinate::lla2enu(_state._init_lla(0), _state._init_lla(1), _state._init_lla(2), longitude, latitude, altitude);
      std::fstream fs("ins.txt", std::ios::in | std::ios::out | std::ios::app);
      fs << local_coor[0] << "," << local_coor[1] << "," << local_coor[2] << "\n";
      fs.close();

    }
  }

  void heading_callback(const novatel_oem7_msgs::msg::HEADING2::SharedPtr msg) {
    Heading heading;
    heading._timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    heading._heading = msg->heading / 180.0 * M_PI;

    // std::cout << "heading: " << msg->heading << std::endl;

    SingletonDataBuffer::getInstance()._heading_buffer.push(heading);
  }


  ~GlocNode()
  {
      // 等待线程结束
      if (thread_.joinable()) {
          thread_.join();
      }
  }
  std::thread thread_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>::SharedPtr gps_sub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::HEADING2>::SharedPtr heading_sub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr ins_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  Initializer _initializer;
  ImuProcessor _imu_processor;
  GpsProcessor _gps_processor;

  std::mutex _state_mtx;
  State _state;

  nav_msgs::msg::Path _message;
  std::deque<geometry_msgs::msg::PoseStamped> trajectory_;

//   rclcpp::Publisher<novatel_oem7_msgs::msg::Localization>::SharedPtr gloc_pub_;
  bool _initialed = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GlocNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}