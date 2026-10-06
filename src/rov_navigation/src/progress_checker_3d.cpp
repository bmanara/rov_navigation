#include "rov_navigation/progress_checker_3d.hpp"

#include <cmath>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"

namespace rov_navigation
{
void ProgressChecker3D::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &plugin_name)
{
    auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("ProgressChecker3D plugin failed to lock node");
    }
    plugin_name_ = plugin_name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Load parameters
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".radius", rclcpp::ParameterValue(radius_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".time_allowance", rclcpp::ParameterValue(time_allowance_));

    radius_ = node->get_parameter(plugin_name_ + ".radius").as_double();
    time_allowance_ = node->get_parameter(plugin_name_ + ".time_allowance").as_double();

    if (radius_ <= 0.0) {
        throw std::runtime_error("ProgressChecker3D: radius must be positive");
    }
    if (time_allowance_ <= 0.0) {
        throw std::runtime_error("ProgressChecker3D: time_allowance must be positive");
    }

    RCLCPP_INFO(logger_, "ProgressChecker3D initialized with radius %.2f and time allowance %.2f",
                radius_, time_allowance_);
}

bool ProgressChecker3D::check(geometry_msgs::msg::PoseStamped &current_pose)
{
    if (!baseline_set_) {
        baseline_position_ = current_pose.pose.position;
        baseline_time_ = clock_->now();
        baseline_set_ = true;
        return true; // First check, consider progress made
    }

    double distance = std::sqrt(
        std::pow(current_pose.pose.position.x - baseline_position_.x, 2) +
        std::pow(current_pose.pose.position.y - baseline_position_.y, 2) +
        std::pow(current_pose.pose.position.z - baseline_position_.z, 2));

    if (distance > radius_) {
        baseline_position_ = current_pose.pose.position;
        baseline_time_ = clock_->now();
        return true; // Progress made
    }

    rclcpp::Duration time_since_baseline = clock_->now() - baseline_time_;
    if (time_since_baseline.seconds() > time_allowance_) {
        RCLCPP_WARN(logger_, "No progress made in the last %.2f seconds", time_since_baseline.seconds());
        return false; // No progress made within the time allowance
    }

    return true; // Progress is still being made
}

void ProgressChecker3D::reset()
{
    baseline_set_ = false;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rov_navigation::ProgressChecker3D, nav2_core::ProgressChecker)