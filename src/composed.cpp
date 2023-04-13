#include <memory>
#include "tuw_laserscan_features/linedetection_node.hpp"
#include "tuw_laserscan_features/feature_viz_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto line_detection_node = std::make_shared<tuw::LineDetectionNode>(options);
  auto feature_viz_node = std::make_shared<tuw::FeatureVizNode>(options);
  exec.add_node(line_detection_node);
  exec.add_node(feature_viz_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
