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

#include <chrono>
#include <thread>
#include <memory>
#include "tuw_laserscan_features/feature_viz_node.hpp"
#include "tuw_laserscan_features/feature_viz_parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuw_geometry/tuw_geometry.hpp>

using std::placeholders::_1;
using namespace tuw;

FeatureVizNode::FeatureVizNode(rclcpp::NodeOptions options)
    : Node("debug_laser_features", options)
{

    param_ = std::make_shared<tuw::FeatureVizParameter>(*this);
    param_->declare_parameters();
    figure_ = std::make_shared<tuw::Figure>(this->get_name());
    figure_->init(param_->map_width, param_->map_height,
                  param_->map_min_x, param_->map_max_x,
                  param_->map_min_y, param_->map_max_y,
                  param_->map_rotation + M_PI,
                  param_->map_grid_x, param_->map_grid_y);

    sub_line_segments_ = create_subscription<tuw_geometry_msgs::msg::LineSegments>(
        "line_segments",
        10, std::bind(&FeatureVizNode::callback_line_segments, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribed to line_segments");

    sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10, std::bind(&FeatureVizNode::callback_laser, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribed to scan");

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(
        100ms, std::bind(&FeatureVizNode::on_timer, this));
}

void FeatureVizNode::on_timer()
{
    figure_->clear();
    for (size_t i = 0; i < msg_laser_scan_.ranges.size(); i++)
    {
        double angle = msg_laser_scan_.angle_min + msg_laser_scan_.angle_increment * i;
        tuw::Point2D p(cos(angle) * msg_laser_scan_.ranges[i],
                       sin(angle) * msg_laser_scan_.ranges[i]); 
        figure_->circle(p, 2, cv::Scalar(0, 255, 0) , 1, cv::LINE_AA);
    }
    for (size_t i = 0; i < msg_line_segments_.segments.size(); i++)
    {
        tuw::Point2D p0(msg_line_segments_.segments[i].p0.x,
                        msg_line_segments_.segments[i].p0.y); 
        tuw::Point2D p1(msg_line_segments_.segments[i].p1.x,
                        msg_line_segments_.segments[i].p1.y); 
        figure_->line(p0, p1, cv::Scalar(0, 0, 255) , 1, cv::LINE_AA);
    }

    cv::imshow(figure_->title(), figure_->view());
    cv::waitKey(1);
}
void FeatureVizNode::callback_line_segments(const tuw_geometry_msgs::msg::LineSegments::SharedPtr msg)
{
    msg_line_segments_ = *msg;
}

void FeatureVizNode::callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    msg_laser_scan_ = *msg;
}
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tuw::FeatureVizNode)
