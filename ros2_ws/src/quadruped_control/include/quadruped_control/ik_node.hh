#ifndef IK_NODE_HH
#define IK_NODE_HH

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "quadruped_kinematics/kinematics.hh"
#include "quadruped_msgs/msg/foot_targets.hpp"
#include "quadruped_msgs/msg/servo_cmd.hpp"

namespace quadruped_control {
  class IkNode : public rclcpp::Node {
    void onTargets(const quadruped_msgs::msg::FootTargets& msg);
    quadruped::LegGeometry geom;
    quadruped::RobotConfig cfg = {};
    int seq = 0;
    double ik_tol;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    rclcpp::Publisher<quadruped_msgs::msg::ServoCmd>::SharedPtr servo_pub;
    rclcpp::Subscription<quadruped_msgs::msg::FootTargets>::SharedPtr sub;
  public:
    IkNode();
  };
}; // namespace quadruped_control

#endif // IK_NODE_HH
