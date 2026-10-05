// thruster_allocator: /body_wrench -> per-thruster thrust commands [N]
//
// Geometry and limits come from rov_description/config/thrusters.yaml.

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rov_control/thruster_allocation.hpp"

namespace rov_control
{

class ThrusterAllocatorNode : public rclcpp::Node
{
public:
  ThrusterAllocatorNode()
  : Node("thruster_allocator")
  {
    const auto names = declare_parameter<std::vector<std::string>>(
      "thruster_names", std::vector<std::string>{});
    const double max_fwd = declare_parameter<double>("max_forward_thrust", 0.0);
    const double max_rev = declare_parameter<double>("max_reverse_thrust", 0.0);
    const auto pattern = declare_parameter<std::string>(
      "command_topic_pattern", "/model/rov/joint/{name}_joint/cmd_thrust");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    timeout_ = declare_parameter<double>("wrench_timeout", 0.5);

    if (names.empty()) {
      throw std::runtime_error("thruster_names is empty; load rov_description/config/thrusters.yaml");
    }

    std::vector<Thruster> thrusters;
    for (const auto & name : names) {
      const auto pos = declare_parameter<std::vector<double>>(name + ".position", std::vector<double>{});
      const auto dir = declare_parameter<std::vector<double>>(name + ".direction", std::vector<double>{});
      if (pos.size() != 3 || dir.size() != 3) {
        throw std::runtime_error("Thruster '" + name + "' needs 3-element position and direction");
      }
      thrusters.push_back({name, {pos[0], pos[1], pos[2]}, {dir[0], dir[1], dir[2]}});

      std::string topic = pattern;
      const auto at = topic.find("{name}");
      if (at != std::string::npos) {
        topic.replace(at, 6, name);
      }
      thrust_pubs_.push_back(create_publisher<std_msgs::msg::Float64>(topic, 10));
    }

    allocator_ = std::make_unique<ThrusterAllocator>(thrusters, max_fwd, max_rev);
    RCLCPP_INFO(get_logger(), "Allocator up: %zu thrusters, rank %d, limits +%.1f / -%.1f N",
      names.size(), allocator_->rank(), max_fwd, max_rev);

    thrusts_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/thrusts", 10);
    achieved_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("~/achieved_wrench", 10);

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/body_wrench", 10, [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
        on_wrench(*msg);
      });

    watchdog_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {check_timeout();});
  }

private:
  void on_wrench(const geometry_msgs::msg::WrenchStamped & msg)
  {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ignoring wrench in frame '%s' (expected '%s')",
        msg.header.frame_id.c_str(), base_frame_.c_str());
      return;
    }
    Vector6d w;
    w << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
      msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;

    const Eigen::VectorXd raw = allocator_->allocate_unsaturated(w);
    const double scale = allocator_->saturation_scale(raw);
    const Eigen::VectorXd u = w.allFinite() ? Eigen::VectorXd(raw * scale) :
      Eigen::VectorXd::Zero(raw.size());
    if (scale < 1.0) {
      RCLCPP_DEBUG(get_logger(), "Saturated: scaled wrench by %.2f", scale);
    }
    publish(u);
    last_wrench_time_ = now();
    timed_out_ = false;

    geometry_msgs::msg::WrenchStamped achieved;
    achieved.header = msg.header;
    const Vector6d a = allocator_->wrench_from_thrusts(u);
    achieved.wrench.force.x = a(0);
    achieved.wrench.force.y = a(1);
    achieved.wrench.force.z = a(2);
    achieved.wrench.torque.x = a(3);
    achieved.wrench.torque.y = a(4);
    achieved.wrench.torque.z = a(5);
    achieved_pub_->publish(achieved);
  }

  void check_timeout()
  {
    const bool stale = !last_wrench_time_.has_value() ||
      (now() - *last_wrench_time_).seconds() > timeout_;
    if (stale) {
      if (!timed_out_ && last_wrench_time_.has_value()) {
        RCLCPP_WARN(get_logger(), "No /body_wrench for %.2fs; zeroing thrusters", timeout_);
      }
      timed_out_ = true;
      // Gazebo's Thruster holds the last command, so keep re-sending zero.
      publish(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(thrust_pubs_.size())));
    }
  }

  void publish(const Eigen::VectorXd & u)
  {
    std_msgs::msg::Float64MultiArray arr;
    arr.data.resize(static_cast<size_t>(u.size()));
    for (Eigen::Index i = 0; i < u.size(); ++i) {
      std_msgs::msg::Float64 m;
      m.data = u(i);
      thrust_pubs_[static_cast<size_t>(i)]->publish(m);
      arr.data[static_cast<size_t>(i)] = u(i);
    }
    thrusts_pub_->publish(arr);
  }

  std::string base_frame_;
  double timeout_{0.5};
  bool timed_out_{true};
  std::optional<rclcpp::Time> last_wrench_time_;
  std::unique_ptr<ThrusterAllocator> allocator_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> thrust_pubs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrusts_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr achieved_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace rov_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_control::ThrusterAllocatorNode>());
  rclcpp::shutdown();
  return 0;
}
