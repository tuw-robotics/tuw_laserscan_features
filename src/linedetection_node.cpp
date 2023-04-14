// Copyright 2022 Markus Bader
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Markus Bader nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "tuw_laserscan_features/linedetection_node.hpp"
#include "tuw_laserscan_features/linedetection_parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tuw_geometry/linesegment2d_detector.hpp>

using std::placeholders::_1;
using namespace tuw;

LineDetectionNode::LineDetectionNode(rclcpp::NodeOptions options)
: Node("laserscan_linedetection", options)
{
  detector_ = std::make_shared<tuw::LineSegment2DDetector>();
  param_ = std::make_shared<tuw::LineDetectionParameter>(*this);
  param_->declare_parameters();

  sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10, std::bind(&LineDetectionNode::callback_laser, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribed to scan");

  pub_linesegments_ = this->create_publisher<tuw_geometry_msgs::msg::LineSegments>("line_segments", 10);
    
}

void LineDetectionNode::callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg_laserscan)
{
    //RCLCPP_INFO(this->get_logger(), "callback_laser");
    detector_->config_ = *param_;
    std::vector<Point2D> points;  /// laser measurments in cartesian robot coordinates
    std::vector<LineSegment2D> detected_segments;

    points.resize(msg_laserscan->ranges.size());
    for ( size_t i = 0; i < msg_laserscan->ranges.size(); i++ ) {
      double angle = msg_laserscan->angle_min + msg_laserscan->angle_increment * i;
      points[i].set(cos ( angle ) * msg_laserscan->ranges[i], sin ( angle ) * msg_laserscan->ranges[i]);
    }
    detector_->start(points,detected_segments);

    tuw_geometry_msgs::msg::LineSegments msg_line_segments;
    msg_line_segments.segments.resize(detected_segments.size());
    for(size_t i = 0; i < detected_segments.size(); i++){
      tuw_geometry_msgs::msg::LineSegment &des =  msg_line_segments.segments[i];
      LineSegment2D &src =  detected_segments[i];
      des.p0.x = src.x0(), des.p0.y = src.y0(), des.p0.z = 0. ;
      des.p1.x = src.x1(), des.p1.y = src.y1(), des.p1.z = 0. ;
    }
    msg_line_segments.header = msg_laserscan->header;
    pub_linesegments_->publish(msg_line_segments);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tuw::LineDetectionNode)
