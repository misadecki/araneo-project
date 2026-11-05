#include "quadruped_control/ik_node.hh"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <quadruped_kinematics/kinematics.hh>

namespace quadruped_control {
  
  IkNode::IkNode() : rclcpp::Node("ik_node") {
    pub = create_publisher<sensor_msgs::msg::JointState>("joint_out", 10);
    sub = create_subscription<std_msgs::msg::Float64MultiArray>("foot_targets",
                                                                10,
                                                                [this](std_msgs::msg::Float64MultiArray::SharedPtr
                                                                       msg)
                                                                {onTargets(*msg);
                                                                });
  
  }

  void IkNode::onTargets(const std_msgs::msg::Float64MultiArray& msg) {
    using namespace quadruped;

    if (msg.data.size() != 3 * LEG_COUNT)
      return;

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    
    for (std::size_t i = 0; i < LEG_COUNT; ++i) {
      const Leg leg = static_cast<Leg>(i);
      const Vector3D body = {msg.data[3 * i], 
                             msg.data[3 * i + 1],
                             msg.data[3 * i + 2]
      };

      const JointAngles q = legIK(bodyToLegFrame(body, leg), legGeometries[leg]);
      const JointAngles theta = modelToUrdf(q, leg);
      
      js.name.push_back(names[i] + "_coxa");
      js.name.push_back(names[i] + "_tibia");
      js.position.push_back(theta.q1);
      js.position.push_back(theta.q2);
    }
    
    pub->publish(js);
  }

};




