#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/executors.hpp"

#include "gloc/initializer.h"
#include "gloc/gps_processor.h"
#include "gloc/imu_processor.h"
#include "gloc/message.h"

class GlocNode : public rclcpp::Node
{
public:

  GlocNode() : Node("gloc_node")
  {
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.keep_last(10);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bynav/imu/data_raw", qos, std::bind(&GlocNode::imu_callback, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", qos, std::bind(&GlocNode::gps_callback, this, std::placeholders::_1));
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ImuData imu_data;
    if (_initialed) {
      ;
    } else {
      _imu_processor.predict(imu_data, _state);
    }
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!_initialed) {

    } else {
      ;
    }
  }

  bool init()
  {
  }

  void proc()
  {
    // std::cout << "Fusionlocalization" << std::endl;
  }

  void run()
  {
    if (!_initialed)
    {
      if(init())
        _initialed = true;
    }
    
    proc();

  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  Initializer _initializer;
  ImuProcessor _imu_processor;
  GpsProcessor _gps_processor;

  State _state;

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