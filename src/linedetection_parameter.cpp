#include "tuw_laserscan_features/linedetection_parameter.hpp"

using namespace tuw;

LineDetectionParameter::LineDetectionParameter(rclcpp::Node &node) : n_(node)
{
}

template <>
void LineDetectionParameter::declare_default_parameter<int>(
    const std::string &name,
    int value_default,
    int min, int max, int step,
    const std::string &desription)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.integer_range = {range};
    descriptor.description = desription;
    n_.declare_parameter<int>(name, value_default, descriptor);
}

template <>
void LineDetectionParameter::declare_default_parameter<double>(
    const std::string &name,
    double value_default,
    double min, double max, double step,
    const std::string &desription)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.floating_point_range = {range};
    descriptor.description = desription;
    n_.declare_parameter<double>(name, value_default, descriptor);
}

void LineDetectionParameter::declare_parameters()
{
    declare_default_parameter<bool>("threshold_split_neighbor", false, "..");
    declare_default_parameter<double>("threshold_split", 0.05, 0.01, 1., 0.001, "split threshold [m]");
    declare_default_parameter<double>("min_length", .1, 0.01, 5., 0.001, "min line length [m]");
    declare_default_parameter<int>("min_points_per_line", 20, 1, 100, 1, "minimal number of points supporting a line");
    declare_default_parameter<int>("min_points_per_unit", 10, 1, 100, 1, "minimal number of points supporting a unit");

    callback_update_parameters();
    using namespace std::chrono_literals;
    timer_update_parameter_ =
        n_.create_wall_timer(
            1000ms,
            std::bind(&LineDetectionParameter::callback_update_parameters, this));
}
void LineDetectionParameter::callback_update_parameters()
{
    n_.get_parameter<bool>("threshold_split_neighbor", threshold_split_neighbor);
    n_.get_parameter<double>("threshold_split", threshold_split);
    n_.get_parameter<double>("threshold_split", threshold_split);
    n_.get_parameter<double>("min_length", min_length);
    n_.get_parameter<int>("min_points_per_line", min_points_per_line);
    n_.get_parameter<int>("min_points_per_unit", min_points_per_unit);
}
