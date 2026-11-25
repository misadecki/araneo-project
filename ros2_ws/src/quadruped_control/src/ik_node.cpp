#include "quadruped_control/ik_node.hh"
#include "quadruped_config/config_loader.hh"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <quadruped_kinematics/kinematics.hh>
#include <rclcpp/logging.hpp>

namespace quadruped_control {
  
  IkNode::IkNode() : rclcpp::Node("ik_node") {
    joint_pub = create_publisher<sensor_msgs::msg::JointState>("joint_cmd",
                                                         rclcpp::QoS(1).best_effort());
    servo_pub = create_publisher<quadruped_msgs::msg::ServoCmd>("servo_cmd",
                                                                rclcpp::QoS(1).best_effort());
    sub = create_subscription<quadruped_msgs::msg::FootTargets>("foot_targets",
                                                                rclcpp::QoS(1).best_effort(),
                                                                [this](quadruped_msgs::msg::FootTargets::SharedPtr
                                                                       msg)
                                                                {onTargets(*msg);
                                                                });
    cfg = loadConfig(*this);
    ik_tol = declare_parameter("ik_tol", 0.002);
  }

  void IkNode::onTargets(const quadruped_msgs::msg::FootTargets& msg) {
    using namespace quadruped;

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    quadruped_msgs::msg::ServoCmd cmd;
    cmd.seq = ++seq;
    
    for (std::size_t i = 0; i < LEG_COUNT; ++i) {
      const Vector3D body = {msg.position[3 * i], msg.position[3 * i + 1],
      msg.position[3 * i + 2]};
      const auto& c = cfg[i];
      const Vector3D p_leg = bodyToLegFrame(body, c.mount);
      const JointAngles j = legIK(p_leg, c.geom);

      const Vector3D back = legFK(j, c.geom);
      const double err = std::hypot(back.x - p_leg.x, back.y - p_leg.y, back.z -
                                    p_leg.z);
      
      if (!std::isfinite(j.q1) || !std::isfinite(j.q2) || err > ik_tol) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK: leg %zu out\
        of range (err = %.4f m)", i, err);
        return;
      }

      const JointAngles theta = modelToUrdf(j, c.mount);
      js.name.push_back(std::string(LEG_NAMES[i]) + "_coxa");
      js.name.push_back(std::string(LEG_NAMES[i]) + "_tibia");
      js.position.push_back(theta.q1);
      js.position.push_back(theta.q2);
      cmd.angle_rad[2 * i] = static_cast<float>(theta.q1);
      cmd.angle_rad[2 * i + 1] = static_cast<float>(theta.q2);
    }
    
    joint_pub->publish(js);
    servo_pub->publish(cmd);
  }
};
