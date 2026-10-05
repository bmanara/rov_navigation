#include "rov_control/velocity_control_law.hpp"

#include <algorithm>
#include <cmath>

namespace rov_control
{

VerticalMode select_vertical_mode(double cmd_vz, double deadband, bool depth_setpoint_active)
{
  if (std::abs(cmd_vz) > deadband) {
    return VerticalMode::kVelocity;
  }
  return depth_setpoint_active ? VerticalMode::kDepthHold : VerticalMode::kVelocity;
}

AttitudeMode select_attitude_mode(double cmd_rate, double deadband)
{
  return std::abs(cmd_rate) > deadband ? AttitudeMode::kRate : AttitudeMode::kHoldLevel;
}

Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond & q_in)
{
  const Eigen::Quaterniond q = q_in.normalized();
  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = std::clamp(2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0);
  const double pitch = std::asin(sinp);

  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  return {roll, pitch, yaw};
}

VelocityControlLaw::VelocityControlLaw(const ControlLawConfig & config)
{
  set_config(config);
}

void VelocityControlLaw::set_config(const ControlLawConfig & config)
{
  config_ = config;
  vx_.set_gains(config.vx);
  vy_.set_gains(config.vy);
  vz_.set_gains(config.vz);
  yaw_rate_.set_gains(config.yaw_rate);
  roll_.set_gains(config.roll);
  pitch_.set_gains(config.pitch);
  roll_rate_.set_gains(config.roll_rate);
  pitch_rate_.set_gains(config.pitch_rate);
  depth_.set_gains(config.depth);
}

void VelocityControlLaw::reset()
{
  for (Pid * p : {&vx_, &vy_, &vz_, &yaw_rate_, &roll_, &pitch_, &roll_rate_, &pitch_rate_,
      &depth_})
  {
    p->reset();
  }
  last_vertical_mode_ = VerticalMode::kVelocity;
  last_roll_mode_ = AttitudeMode::kHoldLevel;
  last_pitch_mode_ = AttitudeMode::kHoldLevel;
}

ControlOutput VelocityControlLaw::update(
  const VehicleState & state, const VelocityCommand & cmd,
  std::optional<double> depth_setpoint, double dt)
{
  ControlOutput out;
  const Eigen::Matrix3d R = state.orientation.normalized().toRotationMatrix();  // body -> world
  const Eigen::Vector3d rpy = quaternion_to_rpy(state.orientation);
  const Eigen::Vector3d & v = state.linear_velocity;
  const Eigen::Vector3d & w = state.angular_velocity;

  // ---- Vertical mode ----
  const bool depth_active = depth_setpoint.has_value() && std::isfinite(*depth_setpoint);
  out.vertical_mode = select_vertical_mode(cmd.linear.z(), config_.vz_deadband, depth_active);
  if (out.vertical_mode != last_vertical_mode_ && out.vertical_mode == VerticalMode::kDepthHold) {
    depth_.reset();
  }
  last_vertical_mode_ = out.vertical_mode;

  double vz_ref = cmd.linear.z();
  if (out.vertical_mode == VerticalMode::kDepthHold) {
    // Depth is positive down; odom z is positive up. Convert explicitly.
    const double depth = -state.z;
    const double depth_rate = -(R * v).z();
    const double depth_error = *depth_setpoint - depth;  // >0: need to go deeper
    // PID output is the desired depth rate (positive = descend), limited by out_limit.
    const double depth_rate_ref = depth_.update(depth_error, dt, -depth_rate);
    const double world_vz_ref = -depth_rate_ref;
    // World vertical speed -> body z component (exact for the z axis; with
    // attitude held level R(2,2) ~ 1).
    vz_ref = R(2, 2) * world_vz_ref;
  }
  out.vz_reference = vz_ref;

  // ---- Translational velocity loops (body frame) ----
  Eigen::Vector3d force;
  force.x() = vx_.update(cmd.linear.x() - v.x(), dt);
  force.y() = vy_.update(cmd.linear.y() - v.y(), dt);
  force.z() = vz_.update(vz_ref - v.z(), dt);

  // Net buoyancy feed-forward: cancel +B (world up) => apply -B in world z.
  force += R.transpose() * Eigen::Vector3d(0.0, 0.0, -config_.net_buoyancy);

  // ---- Roll / pitch: hold level unless a rate is commanded ----
  Eigen::Vector3d torque;
  out.roll_mode = select_attitude_mode(cmd.angular.x(), config_.angular_deadband);
  out.pitch_mode = select_attitude_mode(cmd.angular.y(), config_.angular_deadband);
  if (out.roll_mode != last_roll_mode_) {roll_.reset(); roll_rate_.reset();}
  if (out.pitch_mode != last_pitch_mode_) {pitch_.reset(); pitch_rate_.reset();}
  last_roll_mode_ = out.roll_mode;
  last_pitch_mode_ = out.pitch_mode;

  // Small-angle: body rates p, q ~ roll/pitch rates. D term on measurement.
  torque.x() = out.roll_mode == AttitudeMode::kRate ?
    roll_rate_.update(cmd.angular.x() - w.x(), dt) :
    roll_.update(0.0 - rpy.x(), dt, -w.x());
  torque.y() = out.pitch_mode == AttitudeMode::kRate ?
    pitch_rate_.update(cmd.angular.y() - w.y(), dt) :
    pitch_.update(0.0 - rpy.y(), dt, -w.y());

  // ---- Yaw rate ----
  torque.z() = yaw_rate_.update(cmd.angular.z() - w.z(), dt);

  out.wrench << force, torque;
  return out;
}

}  // namespace rov_control
