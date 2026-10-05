// velocity_controller: /cmd_vel (+ optional /depth_setpoint) + odometry -> /body_wrench
//
// See README.md for the vertical / attitude mode-switching rules.

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "rov_control/velocity_control_law.hpp"

namespace rov_control
{

namespace
{
const char * to_string(VerticalMode m)
{
  return m == VerticalMode::kDepthHold ? "depth_hold" : "velocity";
}
const char * to_string(AttitudeMode m)
{
  return m == AttitudeMode::kRate ? "rate" : "hold_level";
}
}  // namespace

class VelocityControllerNode : public rclcpp::Node
{
public:
  VelocityControllerNode()
  : Node("velocity_controller")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const double rate = declare_parameter<double>("control_rate", 50.0);
    cmd_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);

    for (const char * axis : {"vx", "vy", "vz", "yaw_rate", "roll", "pitch", "roll_rate",
        "pitch_rate", "depth"})
    {
      declare_gains(axis);
    }
    declare_parameter<double>("vz_deadband", 1e-3);
    declare_parameter<double>("angular_deadband", 1e-3);
    declare_parameter<double>("net_buoyancy", 0.0);

    law_ = std::make_unique<VelocityControlLaw>(read_config());

    // Live gain tuning: any parameter change rebuilds the config (keeps integrators).
    param_cb_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {
        law_->set_config(read_config());
        RCLCPP_INFO(get_logger(), "Controller parameters updated");
      });

    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/body_wrench", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "~/mode", rclcpp::QoS(1).transient_local());

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        cmd_.linear = {msg->linear.x, msg->linear.y, msg->linear.z};
        cmd_.angular = {msg->angular.x, msg->angular.y, msg->angular.z};
        last_cmd_time_ = now();
      });

    depth_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/depth_setpoint", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        if (std::isfinite(msg->data)) {
          depth_setpoint_ = msg->data;
          RCLCPP_INFO(get_logger(), "Depth setpoint: %.2f m (positive down)", msg->data);
        } else {
          depth_setpoint_.reset();
          RCLCPP_INFO(get_logger(), "Depth hold disabled (NaN setpoint)");
        }
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p = msg->pose.pose;
        state_.orientation = Eigen::Quaterniond(
          p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        state_.z = p.position.z;
        // nav_msgs/Odometry twist is in child_frame_id (base_link).
        const auto & t = msg->twist.twist;
        state_.linear_velocity = {t.linear.x, t.linear.y, t.linear.z};
        state_.angular_velocity = {t.angular.x, t.angular.y, t.angular.z};
        last_odom_time_ = now();
      });

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {step();});

    RCLCPP_INFO(get_logger(), "velocity_controller up: odom=%s, rate=%.0f Hz",
      odom_topic_.c_str(), rate);
  }

private:
  void declare_gains(const std::string & axis)
  {
    const double inf = std::numeric_limits<double>::infinity();
    declare_parameter<double>(axis + ".kp", 0.0);
    declare_parameter<double>(axis + ".ki", 0.0);
    declare_parameter<double>(axis + ".kd", 0.0);
    declare_parameter<double>(axis + ".i_limit", inf);
    declare_parameter<double>(axis + ".out_limit", inf);
  }

  PidGains gains(const std::string & axis)
  {
    PidGains g;
    g.kp = get_parameter(axis + ".kp").as_double();
    g.ki = get_parameter(axis + ".ki").as_double();
    g.kd = get_parameter(axis + ".kd").as_double();
    g.i_limit = get_parameter(axis + ".i_limit").as_double();
    g.out_limit = get_parameter(axis + ".out_limit").as_double();
    return g;
  }

  ControlLawConfig read_config()
  {
    ControlLawConfig c;
    c.vx = gains("vx");
    c.vy = gains("vy");
    c.vz = gains("vz");
    c.yaw_rate = gains("yaw_rate");
    c.roll = gains("roll");
    c.pitch = gains("pitch");
    c.roll_rate = gains("roll_rate");
    c.pitch_rate = gains("pitch_rate");
    c.depth = gains("depth");
    c.vz_deadband = get_parameter("vz_deadband").as_double();
    c.angular_deadband = get_parameter("angular_deadband").as_double();
    c.net_buoyancy = get_parameter("net_buoyancy").as_double();
    return c;
  }

  void step()
  {
    const rclcpp::Time t = now();
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = base_frame_;

    const bool odom_ok = last_odom_time_.has_value() &&
      (t - *last_odom_time_).seconds() < odom_timeout_;
    if (!odom_ok) {
      // No state => no control. Publish zero wrench and start fresh when odom returns.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No odometry on %s; commanding zero wrench", odom_topic_.c_str());
      law_->reset();
      last_step_time_.reset();
      wrench_pub_->publish(msg);
      return;
    }

    VelocityCommand cmd = cmd_;
    if (!last_cmd_time_.has_value() || (t - *last_cmd_time_).seconds() > cmd_timeout_) {
      cmd = VelocityCommand{};  // stale command => hold still (depth hold stays active)
    }

    if (!last_step_time_.has_value()) {
      last_step_time_ = t;
      wrench_pub_->publish(msg);
      return;
    }
    const double dt = (t - *last_step_time_).seconds();
    last_step_time_ = t;
    if (dt <= 0.0) {
      return;  // sim time paused / reset
    }

    const ControlOutput out = law_->update(state_, cmd, depth_setpoint_, dt);
    msg.wrench.force.x = out.wrench(0);
    msg.wrench.force.y = out.wrench(1);
    msg.wrench.force.z = out.wrench(2);
    msg.wrench.torque.x = out.wrench(3);
    msg.wrench.torque.y = out.wrench(4);
    msg.wrench.torque.z = out.wrench(5);
    wrench_pub_->publish(msg);

    std::string mode = std::string("vertical=") + to_string(out.vertical_mode) +
      " roll=" + to_string(out.roll_mode) + " pitch=" + to_string(out.pitch_mode);
    if (mode != last_mode_) {
      RCLCPP_INFO(get_logger(), "Mode: %s", mode.c_str());
      std_msgs::msg::String m;
      m.data = mode;
      mode_pub_->publish(m);
      last_mode_ = mode;
    }
  }

  std::string odom_topic_, base_frame_;
  double cmd_timeout_{0.5}, odom_timeout_{0.5};
  std::unique_ptr<VelocityControlLaw> law_;
  VehicleState state_;
  VelocityCommand cmd_;
  std::optional<double> depth_setpoint_;
  std::optional<rclcpp::Time> last_cmd_time_, last_odom_time_, last_step_time_;
  std::string last_mode_;

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rov_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_control::VelocityControllerNode>());
  rclcpp::shutdown();
  return 0;
}
