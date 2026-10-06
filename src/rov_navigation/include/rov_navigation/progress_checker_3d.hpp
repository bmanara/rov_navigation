#pragma once

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rov_navigation
{

class ProgressChecker3D : public nav2_core::ProgressChecker
{
public:
    ProgressChecker3D() = default;
    ~ProgressChecker3D() override = default;

    void initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &plugin_name) override;

    bool check(geometry_msgs::msg::PoseStamped &current_pose) override;

    void reset() override;

private:
    rclcpp::Logger logger_{rclcpp::get_logger("ProgressChecker3D")};
    rclcpp::Clock::SharedPtr clock_;
    std::string plugin_name_;

    double radius_{0.5};
    double time_allowance_{10.0};

    bool baseline_set_{false};
    geometry_msgs::msg::Point baseline_position_;
    rclcpp::Time baseline_time_;
};

} // namespace rov_navigation
