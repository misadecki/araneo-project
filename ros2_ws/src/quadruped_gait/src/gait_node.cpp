#include "quadruped_gait/gait_node.hh"
#include "quadruped_gait/trajectory.hh"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include "quadruped_gait/gait.hh"

GaitNode::GaitNode() : Node("gait_node"), last_cmd(now()), last_tick(now()) {
  TrajectoryParams p;
  p.q1_m = declare_parameter("q1_m", 0.3);
  p.q2_st = declare_parameter("q2_st", quadruped::PI - 0.3);
  p.Tsw = declare_parameter("Tsw", 0.3);
  p.dq2 = declare_parameter("dq2", -0.6);

  const auto gait_name = declare_parameter("gait", std::string("walk"));
  const double a_max = declare_parameter("a_max", 0.2);
  const double rate = declare_parameter("rate_hz", 50.0);

  speed_scale = declare_parameter("speed_scale", 1.0);
  cmd_timeout = declare_parameter("cmd_timeout", 0.5);

  gen = std::make_unique<GaitGenerator>(gait_by_name(gait_name), p, a_max);

  names = {"left_front_coxa", "left_front_tibia", "right_front_coxa",
    "right_front_tibia", "left_back_coxa", "left_back_tibia", "right_back_coxa",
    "right_back_tibia"};

  pub = create_publisher<sensor_msgs::msg::JointState>("joint_cmd",
                                                       rclcpp::QoS(1).best_effort());
  sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                          [this](geometry_msgs::msg::Twist::SharedPtr m) {on_cmd(m);});
  tim = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]
                          {tick();});
}

void GaitNode::on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
  gen->set_speed(msg->linear.x * speed_scale);
  last_cmd = now();
}

void GaitNode::tick() {
  const auto t = now();
  const double dt = std::min((t - last_tick).seconds(), 0.1);
  last_tick = t;

  if ((t - last_tick).seconds() > cmd_timeout)
    gen->set_speed(0.0);

  const auto targets = gen->foot_targets(dt);

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = t;
  msg.name.assign(names.begin(), names.end());

  for (std::size_t i = 0; i < quadruped::LEG_COUNT; ++i) {
    const auto th = quadruped::modelToUrdf({targets[i].q1, targets[i].q2},
       static_cast<quadruped::Leg>(i));
    msg.position.push_back(th.q1);
    msg.position.push_back(th.q2);
  }

  pub->publish(msg);
}
