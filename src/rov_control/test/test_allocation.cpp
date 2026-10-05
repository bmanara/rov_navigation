#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "rov_control/thruster_allocation.hpp"

using rov_control::Thruster;
using rov_control::ThrusterAllocator;
using rov_control::Vector6d;

namespace
{
// Same geometry as rov_description/config/thrusters.yaml.
std::vector<Thruster> rov_thrusters()
{
  const double s = std::sqrt(0.5);
  return {
    {"thruster1", {0.15, -0.11, 0.0}, {s, s, 0.0}},
    {"thruster2", {0.15, 0.11, 0.0}, {s, -s, 0.0}},
    {"thruster3", {-0.15, -0.11, 0.0}, {s, -s, 0.0}},
    {"thruster4", {-0.15, 0.11, 0.0}, {s, s, 0.0}},
    {"thruster5", {0.12, -0.22, 0.0}, {0.0, 0.0, 1.0}},
    {"thruster6", {0.12, 0.22, 0.0}, {0.0, 0.0, 1.0}},
    {"thruster7", {-0.12, -0.22, 0.0}, {0.0, 0.0, 1.0}},
    {"thruster8", {-0.12, 0.22, 0.0}, {0.0, 0.0, 1.0}},
  };
}
constexpr double kFwd = 40.0;
constexpr double kRev = 30.0;
}  // namespace

TEST(Allocation, FullRank)
{
  ThrusterAllocator a(rov_thrusters(), kFwd, kRev);
  EXPECT_EQ(a.rank(), 6);
  EXPECT_EQ(a.allocation_matrix().rows(), 6);
  EXPECT_EQ(a.allocation_matrix().cols(), 8);
}

TEST(Allocation, MatrixColumnsAreForceAndMoment)
{
  ThrusterAllocator a(rov_thrusters(), kFwd, kRev);
  const auto & B = a.allocation_matrix();
  // thruster5: vertical at (0.12, -0.22, 0): moment = r x z_hat = (ry, -rx, 0)
  EXPECT_NEAR(B(2, 4), 1.0, 1e-12);
  EXPECT_NEAR(B(3, 4), -0.22, 1e-12);
  EXPECT_NEAR(B(4, 4), -0.12, 1e-12);
  EXPECT_NEAR(B(5, 4), 0.0, 1e-12);
}

TEST(Allocation, ReconstructsFeasibleWrenches)
{
  ThrusterAllocator a(rov_thrusters(), kFwd, kRev);
  std::srand(42);
  for (int k = 0; k < 200; ++k) {
    Vector6d w = Vector6d::Random();
    w.head<3>() *= 20.0;  // N
    w.tail<3>() *= 3.0;   // Nm
    const Eigen::VectorXd u = a.allocate_unsaturated(w);
    EXPECT_TRUE(a.wrench_from_thrusts(u).isApprox(w, 1e-9));
  }
}

TEST(Allocation, PureSurgeUsesOnlyHorizontals)
{
  ThrusterAllocator a(rov_thrusters(), kFwd, kRev);
  Vector6d w = Vector6d::Zero();
  w(0) = 20.0;
  const Eigen::VectorXd u = a.allocate(w);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(u(i), 20.0 / (4 * std::sqrt(0.5)), 1e-9);
  }
  for (int i = 4; i < 8; ++i) {
    EXPECT_NEAR(u(i), 0.0, 1e-9);
  }
}

TEST(Allocation, PureHeaveUsesOnlyVerticals)
{
  ThrusterAllocator a(rov_thrusters(), kFwd, kRev);
  Vector6d w = Vector6d::Zero();
  w(2) = -10.0;
  const Eigen::VectorXd u = a.allocate(w);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(u(i), 0.0, 1e-9);
  }
  for (int i = 4; i < 8; ++i) {
    EXPECT_NEAR(u(i), -2.5, 1e-9);
  }
}

TEST(Allocation, SaturationPreservesDirectionAndRespectsLimits)
{
  ThrusterAllocator a(rov_thrusters(), kFwd, kRev);
  Vector6d w;
  w << 500.0, -200.0, 300.0, 5.0, -8.0, 40.0;  // way beyond capability
  const Eigen::VectorXd u = a.allocate(w);
  EXPECT_LE(u.maxCoeff(), kFwd + 1e-9);
  EXPECT_GE(u.minCoeff(), -kRev - 1e-9);
  // At least one thruster at its limit (scaling is tight).
  const bool at_limit = std::abs(u.maxCoeff() - kFwd) < 1e-9 ||
    std::abs(u.minCoeff() + kRev) < 1e-9;
  EXPECT_TRUE(at_limit);
  // Achieved wrench is parallel to the request.
  const Vector6d achieved = a.wrench_from_thrusts(u);
  const double cos = achieved.dot(w) / (achieved.norm() * w.norm());
  EXPECT_NEAR(cos, 1.0, 1e-9);
}

TEST(Allocation, AsymmetricLimits)
{
  ThrusterAllocator a(rov_thrusters(), kFwd, kRev);
  Vector6d w = Vector6d::Zero();
  w(2) = -1000.0;  // all verticals in reverse
  const Eigen::VectorXd u = a.allocate(w);
  for (int i = 4; i < 8; ++i) {
    EXPECT_NEAR(u(i), -kRev, 1e-9);
  }
}

TEST(Allocation, NonFiniteWrenchGivesZero)
{
  ThrusterAllocator a(rov_thrusters(), kFwd, kRev);
  Vector6d w = Vector6d::Zero();
  w(1) = std::nan("");
  EXPECT_TRUE(a.allocate(w).isZero());
}

TEST(Allocation, RejectsUnderactuatedGeometry)
{
  auto t = rov_thrusters();
  t.resize(4);  // horizontals only: no heave/roll/pitch
  EXPECT_THROW(ThrusterAllocator(t, kFwd, kRev), std::invalid_argument);
  EXPECT_NO_THROW(ThrusterAllocator(t, kFwd, kRev, true));
}

TEST(Allocation, RejectsBadLimitsAndDirections)
{
  EXPECT_THROW(ThrusterAllocator(rov_thrusters(), 0.0, kRev), std::invalid_argument);
  auto t = rov_thrusters();
  t[0].direction.setZero();
  EXPECT_THROW(ThrusterAllocator(t, kFwd, kRev), std::invalid_argument);
}
