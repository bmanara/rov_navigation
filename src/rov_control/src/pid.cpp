#include "rov_control/pid.hpp"

#include <algorithm>
#include <cmath>

namespace rov_control
{

double Pid::update(double error, double dt, double error_rate)
{
  if (!(dt > 0.0) || !std::isfinite(error)) {
    return 0.0;
  }

  double derivative = 0.0;
  if (std::isfinite(error_rate)) {
    derivative = error_rate;
  } else if (has_prev_) {
    derivative = (error - prev_error_) / dt;
  }
  prev_error_ = error;
  has_prev_ = true;

  const double p = gains_.kp * error;
  const double d = gains_.kd * derivative;

  // Conditional integration: don't integrate further into saturation.
  const double candidate = std::clamp(integral_ + gains_.ki * error * dt,
                                      -gains_.i_limit, gains_.i_limit);
  const double unsat = p + candidate + d;
  const bool saturated = std::abs(unsat) > gains_.out_limit;
  const bool pushing_further = (unsat > 0.0) == (error > 0.0);
  if (!(saturated && pushing_further)) {
    integral_ = candidate;
  }

  return std::clamp(p + integral_ + d, -gains_.out_limit, gains_.out_limit);
}

void Pid::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
  has_prev_ = false;
}

}  // namespace rov_control
