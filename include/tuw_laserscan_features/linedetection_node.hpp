#ifndef TUW_LASERSCAN_FEATURES__LINEDETECTION_NODE_HPP_
#define TUW_LASERSCAN_FEATURES__LINEDETECTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tuw_geometry_msgs/msg/line_segments.hpp>
#include "tuw_laserscan_features/visibility.h"

namespace tuw {
  class LineSegment2DDetector;
  class LineDetectionParameter;

class LineDetectionNode : public rclcpp::Node
{
public:
  TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC LineDetectionNode(rclcpp::NodeOptions options);

  int multiply(int a, int b)
  {
    return a * b;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;    /// laser subscriber
  /// Callback function for incoming range measurements
  void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);


  rclcpp::Publisher<tuw_geometry_msgs::msg::LineSegments>::SharedPtr pub_linesegments_;

  std::shared_ptr<tuw::LineSegment2DDetector> detector_;
  std::shared_ptr<tuw::LineDetectionParameter> param_;
};
}

#endif  // TUW_LASERSCAN_FEATURES__LINEDETECTION_NODE_HPP_
