#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "rov_navigation/pure_pursuit_controller_3d.hpp"


namespace rov_navigation
{

static constexpr double EPSILON = 1e-6;

static double limit(double value, double min, double max)
{
    return std::max(min, std::min(value, max));
}

static double wrapAngle(double angle)
{
    return std::remainder(angle, 2.0 * M_PI);
}

static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

static double distance3D(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) +
                     std::pow(p1.y - p2.y, 2) +
                     std::pow(p1.z - p2.z, 2));
}

static double rampTo(double current, double target, double max_step)
{
    return current + limit(target - current, -max_step, max_step);
}

static double stoppingSpeed(double distance, double deceleration)
{
    if (deceleration <= 0.0) {
        throw std::invalid_argument("Deceleration must be positive");
    }
    return std::sqrt(2.0 * deceleration * distance);
}


void PurePursuitController3D::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("PurePursuitController3D plugin failed to lock node");
    }
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    tf_ = tf;

    // Load parameters
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".xy_vel", rclcpp::ParameterValue(xy_vel_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".z_vel", rclcpp::ParameterValue(z_vel_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".yaw_vel", rclcpp::ParameterValue(yaw_vel_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".xy_accel", rclcpp::ParameterValue(xy_accel_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".z_accel", rclcpp::ParameterValue(z_accel_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".yaw_accel", rclcpp::ParameterValue(yaw_accel_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".yaw_gain", rclcpp::ParameterValue(yaw_gain_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(lookahead_distance_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".rotate_to_heading_threshold", rclcpp::ParameterValue(rotate_to_heading_threshold_));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".holonomic", rclcpp::ParameterValue(false));
    


    xy_vel_ = node->get_parameter(plugin_name_ + ".xy_vel").as_double();
    z_vel_ = node->get_parameter(plugin_name_ + ".z_vel").as_double();
    yaw_vel_ = node->get_parameter(plugin_name_ + ".yaw_vel").as_double();
    xy_accel_ = node->get_parameter(plugin_name_ + ".xy_accel").as_double();
    z_accel_ = node->get_parameter(plugin_name_ + ".z_accel").as_double();
    yaw_accel_ = node->get_parameter(plugin_name_ + ".yaw_accel").as_double();
    yaw_gain_ = node->get_parameter(plugin_name_ + ".yaw_gain").as_double();
    lookahead_distance_ = node->get_parameter(plugin_name_ + ".lookahead_distance").as_double();
    rotate_to_heading_threshold_ = node->get_parameter(plugin_name_ + ".rotate_to_heading_threshold").as_double();
    node->get_parameter(plugin_name_ + ".holonomic", holonomic_);

    if (xy_vel_ <= 0.0 || z_vel_ <= 0.0 || yaw_vel_ <= 0.0) {
        throw std::runtime_error("PurePursuitController3D: All velocity parameters must be positive");
    }
    if (xy_accel_ <= 0.0 || z_accel_ <= 0.0 || yaw_accel_ <= 0.0) {
        throw std::runtime_error("PurePursuitController3D: All acceleration parameters must be positive");
    }
    if (lookahead_distance_ <= 0.0) {
        throw std::runtime_error("PurePursuitController3D: lookahead_distance must be positive");
    }
    if (rotate_to_heading_threshold_ < 0.0 || rotate_to_heading_threshold_ > M_PI) {
        throw std::runtime_error("PurePursuitController3D: rotate_to_heading_threshold must be in [0, pi]");
    }
    if (yaw_gain_ < 0.0) {
        throw std::runtime_error("PurePursuitController3D: yaw_gain must be non-negative");
    }

    RCLCPP_INFO(logger_, "PurePursuitController3D initialized with parameters: xy_vel=%.2f, z_vel=%.2f, yaw_vel=%.2f, xy_accel=%.2f, z_accel=%.2f, yaw_accel=%.2f, yaw_gain=%.2f, lookahead_distance=%.2f, rotate_to_heading_threshold=%.2f, holonomic=%s",
                xy_vel_, z_vel_, yaw_vel_, xy_accel_, z_accel_, yaw_accel_, yaw_gain_, lookahead_distance_, rotate_to_heading_threshold_, holonomic_ ? "true" : "false");
}

void PurePursuitController3D::cleanup()
{
    plan_ = nav_msgs::msg::Path();
    closest_index_ = 0;
    has_last_time_ = false;
    RCLCPP_INFO(logger_, "PurePursuitController3D cleaned up");
}

void PurePursuitController3D::activate()
{
    last_cmd_ = Command();
    has_last_time_ = false;
}

void PurePursuitController3D::deactivate()
{
    last_cmd_ = Command();
    has_last_time_ = false;
}

void PurePursuitController3D::setPlan(const nav_msgs::msg::Path & path)
{
    plan_ = path;
    closest_index_ = 0;
    RCLCPP_INFO(logger_, "PurePursuitController3D received new plan with %zu poses", plan_.poses.size());
}

void PurePursuitController3D::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    if (speed_limit <= 0.0)
    {
        speed_scale_ = 1.0;
        return ;
    }
    speed_scale_ = percentage ? limit(speed_limit, 0.0, 1.0) : limit(speed_limit / xy_vel_, 0.0, 1.0);
    speed_scale_ = std::clamp(speed_scale_, 0.0, 1.0);
}
 
geometry_msgs::msg::TwistStamped PurePursuitController3D::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
    const rclcpp::Time now = clock_->now();
    
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now;
    out.header.frame_id = base_frame_;
    
    // Time step for acceleration limits. After a long gap (e.g. a new goal after
    // stopping), start again from rest.
    double dt = 1.0 / controller_frequency_;
    if (has_last_time_) {
        const double measured = (now - last_time_).seconds();
        if (measured > 0.5) {
        last_cmd_ = Command{};
        } else if (measured > 0.0) {
        dt = measured;
        }
    }
    last_time_ = now;
    has_last_time_ = true;
    
    if (plan_.poses.empty()) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "No plan to follow");
        last_cmd_ = Command{};
        return out;
    }
    
    // Work in the plan frame (usually map).
    geometry_msgs::msg::PoseStamped robot;
    if (!nav2_util::transformPoseInTargetFrame(
        pose, robot, *tf_, plan_.header.frame_id, transform_tolerance_))
    {
        RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "Could not transform robot pose into %s",
        plan_.header.frame_id.c_str());
        last_cmd_ = Command{};
        return out;
    }
    const auto & r = robot.pose.position;
    
    // Closest path pose, searching forward only so we never jump back along the path.
    double best = std::numeric_limits<double>::max();
    for (size_t i = closest_index_; i < plan_.poses.size(); ++i) {
        const double d = distance3D(r, plan_.poses[i].pose.position);
        if (d < best) {
        best = d;
        closest_index_ = i;
        }
    }
    
    // Lookahead: first pose at least lookahead_dist away (3D), else the goal.
    size_t look_idx = plan_.poses.size() - 1;
    for (size_t i = closest_index_; i < plan_.poses.size(); ++i) {
        if (distance3D(r, plan_.poses[i].pose.position) >= lookahead_distance_) {
        look_idx = i;
        break;
        }
    }
    
    const auto & goal = plan_.poses.back().pose;
    const Command target = computeTargetVelocity(
        r, yawFromQuaternion(robot.pose.orientation),
        plan_.poses[look_idx].pose.position,
        goal.position, yawFromQuaternion(goal.orientation));
    
    last_cmd_.vx = rampTo(last_cmd_.vx, target.vx, xy_accel_ * dt);
    last_cmd_.vy = rampTo(last_cmd_.vy, target.vy, xy_accel_ * dt);
    last_cmd_.vz = rampTo(last_cmd_.vz, target.vz, z_accel_ * dt);
    last_cmd_.wz = rampTo(last_cmd_.wz, target.wz, yaw_accel_ * dt);
    
    out.twist.linear.x = last_cmd_.vx;
    out.twist.linear.y = last_cmd_.vy;
    out.twist.linear.z = last_cmd_.vz;
    out.twist.angular.z = last_cmd_.wz;
    return out;
}

PurePursuitController3D::Command PurePursuitController3D::computeTargetVelocity(
    const geometry_msgs::msg::Point & robot_position,
    double robot_yaw,
    const geometry_msgs::msg::Point & lookahead_point,
    const geometry_msgs::msg::Point & goal_point,
    double goal_yaw)
{
    Command target;
    
    // Remaining distance to the goal per axis; each axis's speed is capped so it
    // can stop in time. Horizontal motion stops once inside xy_stop_tolerance.
    // Vertical motion only gets a deadband once we're also in position
    // horizontally, so depth keeps being corrected during transit.
    const double goal_dx = goal_point.x - robot_position.x;
    const double goal_dy = goal_point.y - robot_position.y;
    const double goal_xy = std::hypot(goal_dx, goal_dy);
    const double goal_dz = goal_point.z - robot_position.z;
    const bool xy_done = goal_xy < xy_stop_tolerance_;
    const bool z_done = xy_done && std::abs(goal_dz) < z_stop_tolerance_;
    
    const double xy_cap = xy_done ? 0.0 :
        std::min(xy_vel_ * speed_scale_, stoppingSpeed(goal_xy, xy_accel_));
    const double z_cap = z_done ? 0.0 :
        std::min(z_vel_, stoppingSpeed(std::abs(goal_dz), z_accel_));
    
    // Head straight for the lookahead point, scaling horizontal and vertical
    // speed so both components arrive together, each within its own cap.
    const double dx = lookahead_point.x - robot_position.x;
    const double dy = lookahead_point.y - robot_position.y;
    const double dz = lookahead_point.z - robot_position.z;
    const double look_xy = std::hypot(dx, dy);
    const double t_xy = xy_cap > EPSILON ? look_xy / xy_cap : 0.0;
    const double t_z = z_cap > EPSILON ? std::abs(dz) / z_cap : 0.0;
    const double t = std::max(t_xy, t_z);
    const double v_xy = (t > EPSILON && xy_cap > EPSILON) ? look_xy / t : 0.0;
    target.vz = (t > EPSILON && z_cap > EPSILON) ? dz / t : 0.0;
    
    if (holonomic_) {
        // Translate in any horizontal direction without turning.
        const double vx_world = look_xy > EPSILON ? v_xy * dx / look_xy : 0.0;
        const double vy_world = look_xy > EPSILON ? v_xy * dy / look_xy : 0.0;
        const double c = std::cos(robot_yaw);
        const double s = std::sin(robot_yaw);
        target.vx = c * vx_world + s * vy_world;
        target.vy = -s * vx_world + c * vy_world;
        // Turn to the goal heading only once in position.
        if (xy_done) {
        target.wz = limit(yaw_gain_ * wrapAngle(goal_yaw - robot_yaw), -yaw_vel_, yaw_vel_);
        }
        return target;
    }
    
    // Diff-drive style from here on.
    if (xy_done) {
        target.wz = limit(yaw_gain_ * wrapAngle(goal_yaw - robot_yaw), -yaw_vel_, yaw_vel_);
        return target;
    }
    
    // If the lookahead point is (nearly) straight above or below, e.g. during a
    // z-first dive, face the goal instead so we're lined up for the horizontal leg.
    const bool look_is_vertical = look_xy < xy_stop_tolerance_;
    const double aim = look_is_vertical ?
        std::atan2(goal_dy, goal_dx) : std::atan2(dy, dx);
    const double heading_error = wrapAngle(aim - robot_yaw);
    
    if (look_is_vertical || std::abs(heading_error) > rotate_to_heading_threshold_) {
        // Turn in place. Vertical motion no longer waits for horizontal.
        target.wz = limit(yaw_gain_ * heading_error, -yaw_vel_, yaw_vel_);
        target.vz = z_cap > EPSILON ?
        std::copysign(std::min(z_cap, stoppingSpeed(std::abs(dz), z_accel_)), dz) : 0.0;
        return target;
    }
    
    // Pure pursuit: follow the circular arc through the lookahead point,
    // slowing down the further we are off heading.
    target.vx = v_xy * std::cos(heading_error);
    const double curvature = 2.0 * std::sin(heading_error) / look_xy;
    target.wz = limit(target.vx * curvature, -yaw_vel_, yaw_vel_);
    return target;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rov_navigation::PurePursuitController3D, nav2_core::Controller)