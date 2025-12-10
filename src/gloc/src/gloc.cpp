#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/executors.hpp"

#include "gloc/initializer.h"
#include "gloc/gps_processor.h"
#include "gloc/imu_processor.h"
#include "gloc/message.h"
#include "gloc/gloc_data_buffer.h"

#include <cmath>
#include <mutex>

class GlocNode : public rclcpp::Node
{
public:

  GlocNode() : Node("gloc_node")
  {
    std::cout << "构造函数执行" << std::endl;

    std::cout << "开始订阅" << std::endl;
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.keep_last(10);

    heading_sub_ = this->create_subscription<novatel_oem7_msgs::msg::HEADING2>(
      "/bynav/heading2", qos, std::bind(&GlocNode::heading_callback, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&GlocNode::gps_callback, this, std::placeholders::_1));

    init();
    run();    
  }


  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::cout << "imu callback" << std::endl;
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
    }
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std::cout << "gps callback" << std::endl;
    GpsData gps_data;
    gps_data._timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    gps_data._lla << msg->longitude, msg->latitude, msg->altitude;
    gps_data._cov << msg->position_covariance[0], msg->position_covariance[1], msg->position_covariance[2],
                     msg->position_covariance[3], msg->position_covariance[4], msg->position_covariance[5],
                     msg->position_covariance[6], msg->position_covariance[7], msg->position_covariance[8];

    gps_data._status = msg->status.status;

    if (!_initialed) {
      std::lock_guard<std::mutex> lock(_state_mtx);
      if (_initializer.init_by_gps(gps_data, _state)) {
        _initialed = true;
      }
    } else {
      std::lock_guard<std::mutex> lock(_state_mtx);
      _gps_processor.update(gps_data, _state);
    }
  }

  void heading_callback(const novatel_oem7_msgs::msg::HEADING2::SharedPtr msg) {

    std::cout << "heading callback" << std::endl;
    Heading heading;
    heading._timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    heading._heading = msg->heading / 180.0 * M_PI;

    SingletonDataBuffer::getInstance()._heading_buffer.push(heading);
  }

  bool init()
  {
      // initIO();
      return true;
  }

  bool initIO() {


    std::cout << "开始订阅" << std::endl;
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.keep_last(10);

    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //   "/bynav/imu/data_raw", qos, std::bind(&GlocNode::imu_callback, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&GlocNode::gps_callback, this, std::placeholders::_1));

    // heading_sub_ = this->create_subscription<novatel_oem7_msgs::msg::HEADING2>(
    //   "/bynav/heading2", qos, std::bind(&GlocNode::heading_callback, this, std::placeholders::_1));

      return true;
  }

  bool proc()
  {
    return true;
  }

  void run()
  {
    std::shared_ptr<rclcpp::Rate> rate = std::make_shared<rclcpp::Rate>(10);
    while (rclcpp::ok()) {
      rate->sleep();
      proc();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::HEADING2>::SharedPtr heading_sub_;

  Initializer _initializer;
  ImuProcessor _imu_processor;
  GpsProcessor _gps_processor;

  std::mutex _state_mtx;
  State _state;

//   rclcpp::Publisher<novatel_oem7_msgs::msg::Localization>::SharedPtr gloc_pub_;
  bool _initialed = false;
};

int main(int argc, char * argv[])
{

    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<GlocNode>());
    // rclcpp::shutdown();
    // return 0;

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GlocNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}