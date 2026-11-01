#ifndef IK_NODE_HH
#define IK_NODE_HH

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "quadruped_kinematics/kinematics.hh"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace quadruped_control {
  class IkNode : public rclcpp::Node {
    void onTargets(const std_msgs::msg::Float64MultiArray& msg);
    quadruped::LegGeometry geom;
    const std::array<std::string, quadruped::LEG_COUNT> names = {"front_left",
      "front_right", "back_left", "back_right"};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
    //placeholder
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub;
  public:
    IkNode();
  };
}; // namespace quadruped_control

#endif // IK_NODE_HH
