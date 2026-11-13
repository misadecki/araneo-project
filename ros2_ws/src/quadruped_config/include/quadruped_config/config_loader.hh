#ifndef CONFIG_LOADER_HH
#define CONFIG_LOADER_HH

#include <rclcpp/rclcpp.hpp>
#include "quadruped_kinematics/kinematics.hh"

extern inline constexpr std::array<const char*, quadruped::LEG_COUNT> LEG_NAMES = {
  "front_left", "front_right", "back_left", "back_right"
};
quadruped::RobotConfig loadConfig(rclcpp::Node &n);

#endif // CONFIG_LOADER_HH
