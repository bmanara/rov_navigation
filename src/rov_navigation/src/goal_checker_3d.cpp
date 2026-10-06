#include "rov_navigation/goal_checker_3d.hpp"
#include "rov_navigation/goal_check_3d.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"


namespace rov_navigation
{
void GoalChecker3D::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("Goal Checker 3D: Failed to lock lifecycle node");
    }

    plugin_name_ = plugin_name;
    logger_ = node->get_logger();

    const auto p = [this](const std::string & param_name) {
        return plugin_name_ + "." + param_name;
    };

    // Load parameters
    nav2_util::declare_parameter_if_not_declared(
        node, p("xy_goal_tolerance"), rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
        node, p("z_goal_tolerance"), rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
        node, p("check_yaw"), rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node, p("yaw_goal_tolerance"), rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
        node, p("stateful"), rclcpp::ParameterValue(true));

    if (xy_goal_tolerance_ < 0.0 || z_goal_tolerance_ < 0.0 || yaw_goal_tolerance_ < 0.0) {
        throw std::runtime_error("Goal Checker 3D: Tolerances must be non-negative");
    }

    RCLCPP_INFO(logger_, "Goal Checker 3D initialized with xy_goal_tolerance: %.2f, z_goal_tolerance: %.2f, yaw_goal_tolerance: %.2f",
                xy_goal_tolerance_, z_goal_tolerance_, yaw_goal_tolerance_);
}

void GoalChecker3D::reset()
{
    position_reached_ = false;
}

bool GoalChecker3D::isGoalReached(
    const geometry_msgs::msg::Pose &query_pose,
    const geometry_msgs::msg::Pose &goal_pose,
    const geometry_msgs::msg::Twist & /*velocity*/)
{
    double current_x = query_pose.position.x;
    double current_y = query_pose.position.y;
    double current_z = query_pose.position.z;

    double goal_x = goal_pose.position.x;
    double goal_y = goal_pose.position.y;
    double goal_z = goal_pose.position.z;

    double current_yaw = yawFromQuaternion(
        query_pose.orientation.x, query_pose.orientation.y,
        query_pose.orientation.z, query_pose.orientation.w);
    double goal_yaw = yawFromQuaternion(
        goal_pose.orientation.x, goal_pose.orientation.y,
        goal_pose.orientation.z, goal_pose.orientation.w);

    bool position_reached = withinPositionTolerance(
        goal_x - current_x, goal_y - current_y, goal_z - current_z,
        xy_goal_tolerance_, z_goal_tolerance_);

    if (stateful_) {
        if (position_reached) {
            position_reached_ = true;
        }
        return position_reached_ && withinYawTolerance(current_yaw, goal_yaw, yaw_goal_tolerance_);
    } else {
        return position_reached && withinYawTolerance(current_yaw, goal_yaw, yaw_goal_tolerance_);
    }

}

bool GoalChecker3D::getTolerances(
    geometry_msgs::msg::Pose &pose_tolerance,
    geometry_msgs::msg::Twist &vel_tolerance)
{
    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = z_goal_tolerance_;
    pose_tolerance.orientation.x = 0.0;
    pose_tolerance.orientation.y = 0.0;
    pose_tolerance.orientation.z = std::sin(yaw_goal_tolerance_ / 2.0);
    pose_tolerance.orientation.w = std::cos(yaw_goal_tolerance_ / 2.0);

    // Not checking velocity
    vel_tolerance.linear.x = std::numeric_limits<double>::lowest();
    vel_tolerance.linear.y = std::numeric_limits<double>::lowest();
    vel_tolerance.linear.z = std::numeric_limits<double>::lowest();
    vel_tolerance.angular.x = std::numeric_limits<double>::lowest();
    vel_tolerance.angular.y = std::numeric_limits<double>::lowest();
    vel_tolerance.angular.z = std::numeric_limits<double>::lowest();

    return true;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rov_navigation::GoalChecker3D, nav2_core::GoalChecker)