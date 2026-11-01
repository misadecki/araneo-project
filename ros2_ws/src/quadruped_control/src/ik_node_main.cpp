#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include "quadruped_control/ik_node.hh"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadruped_control::IkNode>());
  rclcpp::shutdown();
  return 0;
}
