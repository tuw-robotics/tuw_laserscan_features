#include <memory>
#include "tuw_laserscan_features/linedetection_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto subscriber_node = std::make_shared<LineDetectionNode>(options);
  exec.add_node(subscriber_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
