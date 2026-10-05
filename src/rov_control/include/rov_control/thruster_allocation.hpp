#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace rov_control
{

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct Thruster
{
  std::string name;
  Eigen::Vector3d position;   // [m] in base_link
  Eigen::Vector3d direction;  // unit vector; positive thrust pushes the vehicle this way
};

/// Maps a body wrench [Fx Fy Fz Tx Ty Tz] (base_link) to per-thruster thrusts.
///
///   tau = B u,   column i of B = [d_i ; r_i x d_i]
///   u   = B^+ tau            (minimum-norm least squares, Moore-Penrose)
///
/// Saturation: if any thrust exceeds its limit, the WHOLE vector is scaled down
/// uniformly. That preserves the wrench direction (the vehicle does less of
/// what was asked, not something different) at the cost of magnitude.
class ThrusterAllocator
{
public:
  /// Throws std::invalid_argument if the geometry can't produce all 6 DOF
  /// (unless allow_underactuated) or the limits are non-positive.
  ThrusterAllocator(std::vector<Thruster> thrusters, double max_forward, double max_reverse,
                    bool allow_underactuated = false);

  Eigen::VectorXd allocate(const Vector6d & wrench) const;

  /// Unsaturated least-squares solution.
  Eigen::VectorXd allocate_unsaturated(const Vector6d & wrench) const;

  /// Wrench actually produced by a thrust vector.
  Vector6d wrench_from_thrusts(const Eigen::VectorXd & thrusts) const;

  /// Scale factor in (0, 1] that brings `thrusts` within limits.
  double saturation_scale(const Eigen::VectorXd & thrusts) const;

  const Eigen::MatrixXd & allocation_matrix() const { return B_; }
  const Eigen::MatrixXd & pseudo_inverse() const { return B_pinv_; }
  const std::vector<Thruster> & thrusters() const { return thrusters_; }
  int rank() const { return rank_; }

private:
  std::vector<Thruster> thrusters_;
  double max_forward_;
  double max_reverse_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd B_pinv_;
  int rank_{0};
};

}  // namespace rov_control
