#include "quadruped_gait_node/gait_node.hh"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitNode>());
  rclcpp::shutdown();

  return 0;
}
