#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("generic_robot_node");
  RCLCPP_INFO(node->get_logger(), "Generic robot node started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}