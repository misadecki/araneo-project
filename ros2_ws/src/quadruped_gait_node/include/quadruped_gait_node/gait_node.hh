#ifndef GAIT_NODE_HH
#define GAIT_NODE_HH

#include "sensor_msgs/msg/joint_state.hpp"
#include "quadruped_gait/gait_generator.hh"
#include "quadruped_msgs/msg/servo_cmd.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

class GaitNode : public rclcpp::Node {
  uint32_t seq = 0;
  rclcpp::TimerBase::SharedPtr tim;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
  rclcpp::Publisher<quadruped_msgs::msg::ServoCmd>::SharedPtr servo_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
  rclcpp::Time last_cmd, last_tick;
  double cmd_timeout, speed_scale;
  std::array<std::string, quadruped::LEG_COUNT * 2> names;
  std::unique_ptr<GaitGenerator> gen;
  quadruped::RobotConfig cfg = {};
  void tick();
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
public:
  GaitNode();
};

#endif // GAIT_NODE_HH
