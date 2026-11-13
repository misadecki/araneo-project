#ifndef KINEMATICS_CHECK_HH
#define KINEMATICS_CHECK_HH

#include <rclcpp/rclcpp.hpp>
#include "quadruped_kinematics/kinematics.hh"
#include "sensor_msgs/msg/joint_state.hpp"
#include <array>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_listener.hpp>

namespace quadruped {
  class KinematicsCheck : public rclcpp::Node {
    const std::array<std::string, LEG_COUNT> q1_joint = {"left_front_coxa",
    "right_front_coxa", "left_back_coxa", "right_back_coxa"};
    const std::array<std::string, LEG_COUNT> q2_joint = {"left_front_tibia",
    "right_front_tibia", "left_back_tibia", "right_back_tibia"};
    const std::array<std::string, LEG_COUNT> foot_frame = {"left_front_foot",
    "right_front_foot", "left_back_foot", "right_back_foot"};
    const RobotConfig cfg = {};

    double pos(const sensor_msgs::msg::JointState& js, const std::string& name);
    void check(const sensor_msgs::msg::JointState& js);

    tf2_ros::Buffer buf;
    tf2_ros::TransformListener listener;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;

    sensor_msgs::msg::JointState last;
    bool has = false;
    rclcpp::TimerBase::SharedPtr tim;

  public:
    KinematicsCheck();
  };
}; // namespace quadruped

#endif // KINEMATICS_CHECK_HH
