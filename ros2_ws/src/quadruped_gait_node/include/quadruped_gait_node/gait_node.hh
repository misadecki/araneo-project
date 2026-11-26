#ifndef GAIT_NODE_HH
#define GAIT_NODE_HH

#include "quadruped_gait/gait_generator.hh"
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "quadruped_msgs/msg/foot_targets.hpp"

class GaitNode : public rclcpp::Node {
  uint32_t seq = 0;
  rclcpp::TimerBase::SharedPtr tim;
  rclcpp::Publisher<quadruped_msgs::msg::FootTargets>::SharedPtr foot_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
  rclcpp::Time last_cmd, last_tick;
  double cmd_timeout, speed_scale, turn_scale;
  std::unique_ptr<GaitGenerator> gen;
  quadruped::RobotConfig cfg = {};
  void tick();
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
public:
  GaitNode();
};

#endif // GAIT_NODE_HH
