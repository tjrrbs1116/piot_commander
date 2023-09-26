#include "main.hpp"




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<temp_commander>(options);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}