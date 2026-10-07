#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rov_navigation
{
class PurePursuitController3D : public nav2_core::Controller
{
public:
    PurePursuitController3D() = default;
    ~PurePursuitController3D() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;

private:
    struct Command
    {
        double vx;
        double vy;
        double vz;
        double wz;
    };

    Command computeTargetVelocity(
        const geometry_msgs::msg::Point & robot_position,
        double robot_yaw,
        const geometry_msgs::msg::Point & lookahead_point,
        const geometry_msgs::msg::Point & goal_point,
        double goal_yaw
    );

    rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController3D")};
    rclcpp::Clock::SharedPtr clock_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::string base_frame_;

    double xy_vel_{0.4};
    double z_vel_{0.2};
    double yaw_vel_{0.4};
    double xy_accel_{0.2};
    double z_accel_{0.1};
    double yaw_accel_{0.2};
    double yaw_gain_{1.0};
    double lookahead_distance_{1.0};
    double rotate_to_heading_threshold_{0.6};
    double xy_stop_tolerance_{0.1};
    double z_stop_tolerance_{0.1};
    double transform_tolerance_{0.5};
    double controller_frequency_{20.0};
    bool holonomic_{false};

    nav_msgs::msg::Path plan_;
    size_t closest_index_{0};
    Command last_cmd_;
    rclcpp::Time last_time_;
    bool has_last_time_{false};
    double speed_scale_{1.0};
};

}
