#ifndef GAIT_NODE_HH
#define GAIT_NODE_HH

#include "sensor_msgs/msg/joint_state.hpp"
#include "quadruped_gait/trajectory.hh"
#include <rclcpp/rclcpp.hpp>

class GaitNode : public rclcpp::Node {
  rclcpp::TimerBase::SharedPtr tim;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
  rclcpp::Time t0;
  std::array<double, 4> q1_offset, q2_offset,
                        q1_sign, phase;
  std::array<std::string, 8> names;
  std::unique_ptr<FootTrajectory> traj;
  void tick();
public:
  GaitNode();
};

#endif // GAIT_NODE_HH
