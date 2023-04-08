#ifndef TUW_LASERSCAN_FEATURES__FEATURE_VIZ_NODE_HPP_
#define TUW_LASERSCAN_FEATURES__FEATURE_VIZ_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tuw_geometry/figure.hpp>
#include <tuw_geometry_msgs/msg/line_segments.hpp>
#include "tuw_laserscan_features/visibility.h"

namespace tuw {
class FeatureVizNode : public rclcpp::Node
{
public:
  TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC FeatureVizNode(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<tuw_geometry_msgs::msg::LineSegments>::SharedPtr sub_line_segments_;    /// line subscriber
  /// Callback function for incoming range measurements
  void callback_line_segments(const tuw_geometry_msgs::msg::LineSegments::SharedPtr msg);

  std::shared_ptr<tuw::Figure> figure_;
};
}
#endif  // TUW_LASERSCAN_FEATURES__FEATURE_VIZ_NODE_HPP_
