#include <memory>
#include "tuw_laserscan_features/feature_viz_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tuw::FeatureVizNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
