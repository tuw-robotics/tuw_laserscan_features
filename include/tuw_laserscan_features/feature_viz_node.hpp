#ifndef TUW_LASERSCAN_FEATURES__FEATURE_VIZ_NODE_HPP_
#define TUW_LASERSCAN_FEATURES__FEATURE_VIZ_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tuw_geometry/figure.hpp>
#include <tuw_geometry_msgs/msg/line_segments.hpp>
#include "tuw_laserscan_features/visibility.h"

namespace tuw
{
  class FeatureVizParameter;

  class FeatureVizNode : public rclcpp::Node
  {
  public:
    TUW_LASERSCAN_FEATURES_PACKAGE_PUBLIC FeatureVizNode(rclcpp::NodeOptions options);

  private:
    rclcpp::TimerBase::SharedPtr timer_;                                                      /// timer to on_time()
    void on_timer();                                                                          /// Callback timer
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;                  /// laser subscriber
    sensor_msgs::msg::LaserScan msg_laser_scan_;
    void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);                    /// Callback range measurements
    rclcpp::Subscription<tuw_geometry_msgs::msg::LineSegments>::SharedPtr sub_line_segments_; /// line subscriber
    tuw_geometry_msgs::msg::LineSegments msg_line_segments_;
    void callback_line_segments(const tuw_geometry_msgs::msg::LineSegments::SharedPtr msg);   /// Callback line_segments

    std::shared_ptr<tuw::FeatureVizParameter> param_;
    std::shared_ptr<tuw::Figure> figure_;
  };
}
#endif // TUW_LASERSCAN_FEATURES__FEATURE_VIZ_NODE_HPP_
