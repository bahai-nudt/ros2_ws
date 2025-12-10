#ifndef INITIALIZER_H
#define INITIALIZER_H

#include "gloc/message.h"
#include <rclcpp/rclcpp.hpp>

class Initializer {
public:
    bool init_by_gps(const GpsData& gps_data, State& state);

    rclcpp::Logger logger = rclcpp::get_logger("my_logger");

};


#endif