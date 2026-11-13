#include "quadruped_control/kinematics_check.hh"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <quadruped_kinematics/kinematics.hh>

namespace quadruped {
  KinematicsCheck::KinematicsCheck() : Node("kinematics_check"), buf(get_clock()), listener(buf) {
    sub = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                                                            [this](sensor_msgs::msg::JointState::SharedPtr
                                                                   js)
                                                            {last = *js; has = true; });
    tim = create_wall_timer(std::chrono::milliseconds(5000), [this]() {if (has)
                                                                check(last);});
  }

  static double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
  }

  double KinematicsCheck::pos(const sensor_msgs::msg::JointState& js, const std::string& name) {
    for (size_t i = 0; i < js.name.size(); ++i) {
      if (js.name[i] == name)
        return js.position[i];
    }
    throw std::runtime_error("Lack of joint: " + name);
  }
  void KinematicsCheck::check(const sensor_msgs::msg::JointState& js) {
    for (size_t i = 2; i < 3; ++i) {
      const Leg leg = static_cast<Leg>(i);
      try {
        const LegConfig& c = cfg[leg];
        const JointAngles theta = {pos(js, q1_joint[i]), pos(js, q2_joint[i])};
        const JointAngles q = urdfToModel(theta, c.mount);
        const Vector3D manual = legToBodyFrame(legFK(q, c.geom), c.mount);
        const auto t = buf.lookupTransform("base_link", foot_frame[i],
                                      tf2::TimePointZero).transform.translation;
        const Vector3D foot_tf = {t.x, t.y, t.z};
        const JointAngles ik_model = legIK(bodyToLegFrame(foot_tf,
                              c.mount), c.geom);
        const JointAngles ik_urdf = urdfToModel(ik_model, c.mount);
        /*
        RCLCPP_INFO(get_logger(), "Leg %zu: manual \033[32m(%.4f, %.4f,\
                                  %.4f)\033[0m | TF \
                                  \033[32m(%.4f, %.4f, %.4f) | \033[31merror: %.2f mm", i, 
                                  manual.x, manual.y, manual.z, t.x, t.y, t.z,
                                  1000.0 * std::sqrt(std::pow((manual.x - t.x), 2)
                                  + std::pow((manual.y - t.y), 2)
                                  + std::pow((manual.z - t.z), 2)));
        */
      RCLCPP_INFO(get_logger(),
          "Leg %zu IK: q1 %.3f (slider %.3f, \033[31mdiff %.4f\033[0m) | "
          "q2 %.3f (slider %.3f, \033[31mdiff %.4f\033[0m)",
          i,
          ik_urdf.q1, theta.q1, wrap(ik_urdf.q1 - theta.q1),
          ik_urdf.q2, theta.q2, wrap(ik_urdf.q2 - theta.q2));
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Leg %zu: %s", i, e.what());
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadruped::KinematicsCheck>());
  rclcpp::shutdown();

  return 0;
}
