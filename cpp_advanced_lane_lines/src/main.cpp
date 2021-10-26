#include <iostream>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "../include/cpp_lane_line_detection.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneLineDetection>());
  rclcpp::shutdown();
  return 0;
}