#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "rov_control/velocity_control_law.hpp"

using namespace rov_control;  // NOLINT

namespace
{
ControlLawConfig p_only_config()
{
  ControlLawConfig c;
  c.vx = {10.0, 0.0, 0.0};
  c.vy = {10.0, 0.0, 0.0};
  c.vz = {10.0, 0.0, 0.0};
  c.yaw_rate = {2.0, 0.0, 0.0};
  c.roll = {5.0, 0.0, 1.0};
  c.pitch = {5.0, 0.0, 1.0};
  c.roll_rate = {1.0, 0.0, 0.0};
  c.pitch_rate = {1.0, 0.0, 0.0};
  c.depth = {1.0, 0.0, 0.0};
  c.depth.out_limit = 0.3;
  return c;
}
constexpr double kDt = 0.02;
}  // namespace

// ---- Mode-selection rule (documented in README) ----

TEST(ModeSelection, VzCommandWinsOverDepthSetpoint)
{
  EXPECT_EQ(select_vertical_mode(0.2, 1e-3, true), VerticalMode::kVelocity);
  EXPECT_EQ(select_vertical_mode(-0.2, 1e-3, true), VerticalMode::kVelocity);
}

TEST(ModeSelection, DepthHoldWhenNoVzAndSetpointActive)
{
  EXPECT_EQ(select_vertical_mode(0.0, 1e-3, true), VerticalMode::kDepthHold);
  EXPECT_EQ(select_vertical_mode(5e-4, 1e-3, true), VerticalMode::kDepthHold);
}

TEST(ModeSelection, VelocityWhenNoSetpoint)
{
  EXPECT_EQ(select_vertical_mode(0.0, 1e-3, false), VerticalMode::kVelocity);
}

TEST(ModeSelection, AttitudeRateOnlyWhenCommanded)
{
  EXPECT_EQ(select_attitude_mode(0.0, 1e-3), AttitudeMode::kHoldLevel);
  EXPECT_EQ(select_attitude_mode(0.1, 1e-3), AttitudeMode::kRate);
}

// ---- Frame conversions ----

TEST(Frames, QuaternionToRpy)
{
  const double r = 0.1, p = -0.2, y = 1.3;
  const Eigen::Quaterniond q =
    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d rpy = quaternion_to_rpy(q);
  EXPECT_NEAR(rpy.x(), r, 1e-12);
  EXPECT_NEAR(rpy.y(), p, 1e-12);
  EXPECT_NEAR(rpy.z(), y, 1e-12);
}

// ---- Control law ----

TEST(ControlLaw, ZeroErrorZeroWrench)
{
  VelocityControlLaw law(p_only_config());
  const auto out = law.update(VehicleState{}, VelocityCommand{}, std::nullopt, kDt);
  EXPECT_TRUE(out.wrench.isZero(1e-12));
}

TEST(ControlLaw, AllFourAxesTracked)
{
  VelocityControlLaw law(p_only_config());
  VelocityCommand cmd;
  cmd.linear = {0.5, -0.2, 0.3};
  cmd.angular.z() = 0.4;
  const auto out = law.update(VehicleState{}, cmd, std::nullopt, kDt);
  EXPECT_NEAR(out.wrench(0), 5.0, 1e-9);
  EXPECT_NEAR(out.wrench(1), -2.0, 1e-9);
  EXPECT_NEAR(out.wrench(2), 3.0, 1e-9);
  EXPECT_NEAR(out.wrench(5), 0.8, 1e-9);
  EXPECT_EQ(out.vertical_mode, VerticalMode::kVelocity);
}

TEST(ControlLaw, DepthHoldDescendsWhenAboveSetpoint)
{
  VelocityControlLaw law(p_only_config());
  VehicleState s;
  s.z = -0.5;  // depth 0.5 m
  const auto out = law.update(s, VelocityCommand{}, 2.0, kDt);  // want 2 m depth
  EXPECT_EQ(out.vertical_mode, VerticalMode::kDepthHold);
  // Error 1.5 m * kp 1 -> 1.5 m/s descend, limited to 0.3 m/s; body vz ref = -0.3
  EXPECT_NEAR(out.vz_reference, -0.3, 1e-9);
  EXPECT_LT(out.wrench(2), 0.0);  // push down
}

TEST(ControlLaw, DepthHoldAscendsWhenBelowSetpoint)
{
  VelocityControlLaw law(p_only_config());
  VehicleState s;
  s.z = -3.0;
  const auto out = law.update(s, VelocityCommand{}, 2.9, kDt);
  EXPECT_NEAR(out.vz_reference, 0.1, 1e-9);
  EXPECT_GT(out.wrench(2), 0.0);
}

TEST(ControlLaw, VzCommandOverridesDepthHold)
{
  VelocityControlLaw law(p_only_config());
  VehicleState s;
  s.z = -0.5;
  VelocityCommand cmd;
  cmd.linear.z() = 0.2;  // go up even though setpoint is deeper
  const auto out = law.update(s, cmd, 2.0, kDt);
  EXPECT_EQ(out.vertical_mode, VerticalMode::kVelocity);
  EXPECT_NEAR(out.vz_reference, 0.2, 1e-9);
  EXPECT_GT(out.wrench(2), 0.0);
}

TEST(ControlLaw, NaNSetpointDisablesDepthHold)
{
  VelocityControlLaw law(p_only_config());
  const auto out = law.update(VehicleState{}, VelocityCommand{}, std::nan(""), kDt);
  EXPECT_EQ(out.vertical_mode, VerticalMode::kVelocity);
}

TEST(ControlLaw, AttitudeHoldOpposesRollAndPitch)
{
  VelocityControlLaw law(p_only_config());
  VehicleState s;
  s.orientation = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY());
  auto out = law.update(s, VelocityCommand{}, std::nullopt, kDt);
  EXPECT_NEAR(out.wrench(3), -0.5, 1e-6);  // -kp * roll

  VelocityControlLaw law2(p_only_config());
  s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()));
  s.angular_velocity.y() = 0.5;
  out = law2.update(s, VelocityCommand{}, std::nullopt, kDt);
  EXPECT_NEAR(out.wrench(4), 5.0 * 0.2 - 1.0 * 0.5, 1e-6);  // kp*err + kd*(-q)
}

TEST(ControlLaw, RollRateCommandSwitchesToRateMode)
{
  VelocityControlLaw law(p_only_config());
  VelocityCommand cmd;
  cmd.angular.x() = 0.3;
  const auto out = law.update(VehicleState{}, cmd, std::nullopt, kDt);
  EXPECT_EQ(out.roll_mode, AttitudeMode::kRate);
  EXPECT_EQ(out.pitch_mode, AttitudeMode::kHoldLevel);
  EXPECT_NEAR(out.wrench(3), 0.3, 1e-9);
}

TEST(ControlLaw, BuoyancyFeedForwardInWorldFrame)
{
  auto c = p_only_config();
  c.net_buoyancy = 2.0;
  VelocityControlLaw law(c);
  // Level: straight down in body z.
  auto out = law.update(VehicleState{}, VelocityCommand{}, std::nullopt, kDt);
  EXPECT_NEAR(out.wrench(2), -2.0, 1e-9);

  // Yawed 90 deg: still straight down (yaw doesn't affect world z).
  VelocityControlLaw law2(c);
  VehicleState s;
  s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  out = law2.update(s, VelocityCommand{}, std::nullopt, kDt);
  EXPECT_NEAR(out.wrench(0), 0.0, 1e-9);
  EXPECT_NEAR(out.wrench(1), 0.0, 1e-9);
  EXPECT_NEAR(out.wrench(2), -2.0, 1e-9);
}

TEST(ControlLaw, DepthHoldIntegratorResetOnReentry)
{
  auto c = p_only_config();
  c.depth = {1.0, 1.0, 0.0};
  VelocityControlLaw law(c);
  VehicleState s;
  s.z = -1.0;
  for (int i = 0; i < 50; ++i) {
    law.update(s, VelocityCommand{}, 2.0, kDt);  // accumulate depth integral
  }
  VelocityCommand up;
  up.linear.z() = 0.1;
  law.update(s, up, 2.0, kDt);  // leave depth hold
  const auto out = law.update(s, VelocityCommand{}, 2.0, kDt);  // re-enter
  // Fresh integral: only one step's worth (1*1*0.02) on top of kp*err
  EXPECT_NEAR(out.vz_reference, -(1.0 + 0.02), 1e-9);
}
