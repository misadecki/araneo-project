#include "quadruped_config/config_loader.hh"

quadruped::RobotConfig loadConfig(rclcpp::Node &n) {
  quadruped::RobotConfig cfg = {};
  for (std::size_t i = 0; i < quadruped::LEG_COUNT; ++i) {
    quadruped::LegConfig c = {};
    const std::string p = "legs." + std::string(LEG_NAMES[i]) + ".";

    const auto off = n.declare_parameter<std::vector<double>>(p + "offset");
    if (off.size() != 3) throw std::runtime_error(p + "offset need to have 3 elements");
    c.mount.offset = {off[0], off[1], off[2]};
    c.mount.yaw = n.declare_parameter<double>(p + "yaw");
    c.mount.q1_sign = n.declare_parameter<int>(p + "q1_sign");
    c.mount.q1_dir = n.declare_parameter<int>(p + "q1_dir");
    c.mount.q2_sign = n.declare_parameter<int>(p + "q2_sign");
    c.mount.q2_zero = n.declare_parameter<double>(p + "q2_zero");

    c.geom.femur_len = n.declare_parameter<double>(p + "femur_len");
    c.geom.femur_ang = n.declare_parameter<double>(p + "femur_ang");
    c.geom.lateral = n.declare_parameter<double>(p + "lateral");
    c.geom.tibia_len = n.declare_parameter<double>(p + "tibia_len");
    cfg[i] = c;
  }

  return cfg;
}
