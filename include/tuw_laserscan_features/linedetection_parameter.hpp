#ifndef TUW_LASERSCAN_FEATURES__LINEDETECTION_PARAMETER_NODE_HPP_
#define TUW_LASERSCAN_FEATURES__LINEDETECTION_PARAMETER_NODE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <tuw_geometry/linesegment2d_detector.hpp>

namespace tuw{
/**
 * class to handle particle filter parameter using ros parameter callbacks
 */
class LineDetectionParameter : public LineSegment2DDetectorParameter
{
public:
    /**
     * Constructor
     * @param node reference to the parent node
     */
    LineDetectionParameter(rclcpp::Node &node);
    void declare_parameters();         /// declares ros parameters
    void callback_update_parameters(); /// callback to check changes on the parameters
private:
    /**
     * Helper to declare numbered parameter
     * @param name
     * @param value_default
     * @param min
     * @param max
     * @param step
     * @param description
     */
    template <typename T>
    void declare_default_parameter(
        const std::string &name,
        T value_default,
        T min, T max, T step,
        const std::string &description);

    /**
     * Helper to declare string parameters
     * @param name
     * @param value_default
     * @param min
     * @param max
     * @param step
     * @param description
     */
    template <typename T>
    void declare_default_parameter(
        const std::string &name,
        const T &value_default,
        const std::string &description)
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = description;
        n_.declare_parameter<T>(name, value_default, descriptor);
    }

    rclcpp::Node &n_;                                     /// reference to parent node
    rclcpp::TimerBase::SharedPtr timer_update_parameter_; /// timer to check regularly for parameter changes
 };
/// some useful prototypes
using LineDetectionNodeParameterPtr = std::shared_ptr<LineDetectionParameter>;
using LineDetectionNodeParameterConstPtr = std::shared_ptr<LineDetectionParameter const>;
}
#endif // TUW_LASERSCAN_FEATURES__LINEDETECTION_PARAMETER_NODE_HPP_
