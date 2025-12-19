#ifndef INITIALIZER_H
#define INITIALIZER_H

#include "gloc/message.h"
#include <rclcpp/rclcpp.hpp>

class Initializer {
public:
    bool init_by_gps(const GpsData& gps_data, State& state);

    Eigen::Vector3d _lever_arm = Eigen::Vector3d(0.0, 0.0, 0.0);
    rclcpp::Logger logger = rclcpp::get_logger("my_logger");

};


#endif