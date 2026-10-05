#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <optional>

#include "rov_control/pid.hpp"
#include "rov_control/thruster_allocation.hpp"

namespace rov_control
{

/// Vertical control mode. The selection rule (see select_vertical_mode and the
/// package README) is deliberately a free function so it's explicit and tested.
enum class VerticalMode
{
  kVelocity,   // cmd_vel.linear.z drives body vz (zero => hold vz = 0)
  kDepthHold,  // depth PID drives vz towards /depth_setpoint
};

/// Rule:
///   |cmd_vz| > deadband          -> kVelocity  (operator/autonomy commands vz directly)
///   else if depth setpoint active -> kDepthHold
///   else                          -> kVelocity  (with vz = 0)
VerticalMode select_vertical_mode(double cmd_vz, double deadband, bool depth_setpoint_active);

/// Roll/pitch mode, chosen per axis.
enum class AttitudeMode
{
  kHoldLevel,  // angle PID to 0 (default)
  kRate,       // cmd_vel.angular.x / .y treated as body rate setpoint
};

AttitudeMode select_attitude_mode(double cmd_rate, double deadband);

/// ZYX Euler angles (roll, pitch, yaw) from a body->world quaternion.
Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond & q);

struct VehicleState
{
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};  // base_link -> odom
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};   // body frame [m/s]
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};  // body frame [rad/s]
  double z{0.0};  // odom-frame z [m], z UP. depth = -z.
};

struct VelocityCommand
{
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};   // body frame [m/s]
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};  // body frame [rad/s]
};

struct ControlLawConfig
{
  PidGains vx, vy, vz, yaw_rate;
  PidGains roll, pitch;            // attitude hold (angle -> torque)
  PidGains roll_rate, pitch_rate;  // used only when angular.x / .y commanded
  PidGains depth;                  // depth error [m] -> vertical speed ref [m/s]; out_limit = max depth rate
  double vz_deadband{1e-3};        // [m/s]
  double angular_deadband{1e-3};   // [rad/s] for roll/pitch rate commands
  double net_buoyancy{0.0};        // [N], positive = floats. Feed-forward, applied in world z.
};

struct ControlOutput
{
  Vector6d wrench{Vector6d::Zero()};  // [Fx Fy Fz Tx Ty Tz] in base_link
  VerticalMode vertical_mode{VerticalMode::kVelocity};
  AttitudeMode roll_mode{AttitudeMode::kHoldLevel};
  AttitudeMode pitch_mode{AttitudeMode::kHoldLevel};
  double vz_reference{0.0};  // body vz setpoint actually tracked
};

/// Velocity controller: PID per axis on body velocities (vx, vy, vz, yaw rate),
/// plus roll/pitch attitude hold and optional depth hold.
class VelocityControlLaw
{
public:
  explicit VelocityControlLaw(const ControlLawConfig & config);

  void set_config(const ControlLawConfig & config);

  /// depth_setpoint: meters, POSITIVE DOWN; std::nullopt = depth hold inactive.
  ControlOutput update(const VehicleState & state, const VelocityCommand & cmd,
                       std::optional<double> depth_setpoint, double dt);

  void reset();

private:
  ControlLawConfig config_;
  Pid vx_, vy_, vz_, yaw_rate_, roll_, pitch_, roll_rate_, pitch_rate_, depth_;
  VerticalMode last_vertical_mode_{VerticalMode::kVelocity};
  AttitudeMode last_roll_mode_{AttitudeMode::kHoldLevel};
  AttitudeMode last_pitch_mode_{AttitudeMode::kHoldLevel};
};

}  // namespace rov_control
