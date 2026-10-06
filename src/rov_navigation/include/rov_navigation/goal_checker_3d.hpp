#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"


namespace rov_navigation
{

class GoalChecker3D : public nav2_core::GoalChecker
{
public:
    GoalChecker3D() = default;
    ~GoalChecker3D() override = default;

    void initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &plugin_name,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
    void reset() override;

    bool isGoalReached(
        const geometry_msgs::msg::Pose &query_pose,
        const geometry_msgs::msg::Pose &goal_pose,
        const geometry_msgs::msg::Twist &velocity) override;

    bool getTolerances(
        geometry_msgs::msg::Pose &pose_tolerance,
        geometry_msgs::msg::Twist &vel_tolerance) override;
    
    
private:
    rclcpp::Logger logger_{rclcpp::get_logger("GoalChecker3D")};
    std::string plugin_name_;

    double xy_goal_tolerance_{0.25};
    double z_goal_tolerance_{0.25};
    double yaw_goal_tolerance_{0.25};

    bool stateful_{true};

    bool position_reached_{false};
};

}
