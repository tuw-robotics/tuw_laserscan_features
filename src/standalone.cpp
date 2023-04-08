#include <memory>
#include "tuw_laserscan_features/linedetection_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetectionNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
