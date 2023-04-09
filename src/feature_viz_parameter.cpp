#include "tuw_laserscan_features/feature_viz_parameter.hpp"

using namespace tuw;

FeatureVizParameter::FeatureVizParameter(rclcpp::Node &node) : n_(node)
{
}

template <>
void FeatureVizParameter::declare_default_parameter<int>(
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
void FeatureVizParameter::declare_default_parameter<double>(
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

void FeatureVizParameter::declare_parameters()
{
    declare_default_parameter<double>("map_rotation",  0, -3.14, 3.14, 0.001, "map orientation [rad]");
    declare_default_parameter<double>("map_grid",  1,  0.001,  10, 0.001, "grid size [m]");
    declare_default_parameter<double>("map_max_x",  9, -20, 20, 0.001, "min y [m]");
    declare_default_parameter<double>("map_max_y",  9, -20, 20, 0.001, "min x [m]");
    declare_default_parameter<double>("map_min_y", -9, -20, 20, 0.001, "min y [m]");
    declare_default_parameter<double>("map_min_x", -9, -20, 20, 0.001, "min x [m]");
    declare_default_parameter<int>("map_width", 600, 200, 1000, 1, "view wight [pix]");
    declare_default_parameter<int>("map_height", 600, 200, 1000, 1, "view height [pix]");

    callback_update_parameters();
    using namespace std::chrono_literals;
    timer_update_parameter_ =
        n_.create_wall_timer(
            1000ms,
            std::bind(&FeatureVizParameter::callback_update_parameters, this));
}
void FeatureVizParameter::callback_update_parameters()
{
    n_.get_parameter<double>("map_grid", map_grid_x);
    n_.get_parameter<double>("map_grid", map_grid_y);
    n_.get_parameter<double>("map_rotation", map_rotation);
    n_.get_parameter<double>("map_max_x", map_max_x);
    n_.get_parameter<double>("map_max_y", map_max_y);
    n_.get_parameter<double>("map_min_y", map_min_y);
    n_.get_parameter<double>("map_min_x", map_min_x);
    n_.get_parameter<int>("map_width", map_width);
    n_.get_parameter<int>("map_height", map_height);
}
