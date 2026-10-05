#pragma once

#include <limits>

namespace rov_control
{

struct PidGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double i_limit{std::numeric_limits<double>::infinity()};    // |integral term| clamp
  double out_limit{std::numeric_limits<double>::infinity()};  // |output| clamp
};

/// Scalar PID with integrator clamping and conditional integration (anti-windup).
///
/// Derivative: if `error_rate` is provided it is used directly (e.g. -measured
/// rate, which avoids derivative kick on setpoint steps); otherwise the error is
/// finite-differenced.
class Pid
{
public:
  Pid() = default;
  explicit Pid(const PidGains & gains) : gains_(gains) {}

  void set_gains(const PidGains & gains) { gains_ = gains; }
  const PidGains & gains() const { return gains_; }

  double update(double error, double dt,
                double error_rate = std::numeric_limits<double>::quiet_NaN());

  void reset();
  double integral_term() const { return integral_; }

private:
  PidGains gains_;
  double integral_{0.0};   // stores ki * integral(error), so gain changes don't jump
  double prev_error_{0.0};
  bool has_prev_{false};
};

}  // namespace rov_control
