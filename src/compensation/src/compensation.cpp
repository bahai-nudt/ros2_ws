#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"
#include "nav_msgs/msg/path.hpp"
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/executors.hpp"

#include "gloc/message.h"
#include "gloc/gloc_data_buffer.h"
#include "base_utils/coordinate.h"

#include <cmath>
#include <mutex>
#include <iomanip>
#include <fstream>


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

    ins_sub_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "/bynav/inspvax", qos, std::bind(&CompensationNode::ins_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/iv_points", qos, std::bind(&CompensationNode::ins_callback, this, std::placeholders::_1));


    return true;
  }

  void ins_callback(const novatel_oem7_msgs::msg::INSPVAX::SharedPtr msg)
  {
      Ins ins;
      ins._longitude = msg->longitude;
      ins._latitude = msg->latitude;
      ins._altitude = msg->altitude;
      ins._azimuth = msg->azimuth;

      ins._roll = msg->roll;
      ins._pitch = msg->pitch;

      ins._status = msg->status;
      ins._timestamp = msg->timestamp;


    //Pose _pose;



      SingletonDataBuffer::getInstance()._ins_buffer.push(ins);
  }

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