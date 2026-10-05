#include <gtest/gtest.h>

#include <cmath>

#include "rov_control/pid.hpp"

using rov_control::Pid;
using rov_control::PidGains;

TEST(Pid, ProportionalOnly)
{
  Pid pid({2.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(pid.update(1.5, 0.01), 3.0);
  EXPECT_DOUBLE_EQ(pid.update(-0.5, 0.01), -1.0);
}

TEST(Pid, IntegralAccumulates)
{
  Pid pid({0.0, 1.0, 0.0});
  double out = 0.0;
  for (int i = 0; i < 100; ++i) {
    out = pid.update(1.0, 0.01);
  }
  EXPECT_NEAR(out, 1.0, 1e-9);  // ki * integral(1 dt over 1 s)
}

TEST(Pid, IntegralClamped)
{
  PidGains g{0.0, 10.0, 0.0};
  g.i_limit = 2.0;
  Pid pid(g);
  for (int i = 0; i < 1000; ++i) {
    pid.update(1.0, 0.01);
  }
  EXPECT_NEAR(pid.integral_term(), 2.0, 1e-9);
}

TEST(Pid, AntiWindupStopsIntegratingIntoSaturation)
{
  PidGains g{1.0, 1.0, 0.0};
  g.out_limit = 1.5;
  Pid pid(g);
  for (int i = 0; i < 1000; ++i) {
    EXPECT_LE(pid.update(1.0, 0.01), 1.5);
  }
  // P alone is 1.0; integral can only grow until output saturates (~0.5).
  EXPECT_LE(pid.integral_term(), 0.5 + 1e-2);
  // Reversing the error unwinds immediately (output goes negative fast).
  EXPECT_LT(pid.update(-2.0, 0.01), 0.0);
}

TEST(Pid, DerivativeFromSuppliedRate)
{
  Pid pid({0.0, 0.0, 3.0});
  EXPECT_DOUBLE_EQ(pid.update(5.0, 0.01, -0.5), -1.5);
}

TEST(Pid, DerivativeFiniteDifference)
{
  Pid pid({0.0, 0.0, 1.0});
  EXPECT_DOUBLE_EQ(pid.update(0.0, 0.1), 0.0);  // first sample: no derivative
  EXPECT_NEAR(pid.update(1.0, 0.1), 10.0, 1e-9);
}

TEST(Pid, BadInputsReturnZero)
{
  Pid pid({1.0, 1.0, 1.0});
  EXPECT_EQ(pid.update(1.0, 0.0), 0.0);
  EXPECT_EQ(pid.update(1.0, -1.0), 0.0);
  EXPECT_EQ(pid.update(std::nan(""), 0.01), 0.0);
  EXPECT_EQ(pid.integral_term(), 0.0);
}

TEST(Pid, Reset)
{
  Pid pid({0.0, 1.0, 0.0});
  pid.update(1.0, 1.0);
  pid.reset();
  EXPECT_EQ(pid.integral_term(), 0.0);
}
