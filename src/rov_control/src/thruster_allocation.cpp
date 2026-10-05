#include "rov_control/thruster_allocation.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rov_control
{

ThrusterAllocator::ThrusterAllocator(
  std::vector<Thruster> thrusters, double max_forward, double max_reverse,
  bool allow_underactuated)
: thrusters_(std::move(thrusters)), max_forward_(max_forward), max_reverse_(max_reverse)
{
  if (thrusters_.empty()) {
    throw std::invalid_argument("ThrusterAllocator: no thrusters");
  }
  if (!(max_forward_ > 0.0) || !(max_reverse_ > 0.0)) {
    throw std::invalid_argument("ThrusterAllocator: thrust limits must be positive");
  }

  const auto n = static_cast<Eigen::Index>(thrusters_.size());
  B_.resize(6, n);
  for (Eigen::Index i = 0; i < n; ++i) {
    auto & t = thrusters_[static_cast<size_t>(i)];
    if (t.direction.norm() < 1e-9) {
      throw std::invalid_argument("ThrusterAllocator: zero direction for " + t.name);
    }
    t.direction.normalize();
    B_.block<3, 1>(0, i) = t.direction;
    B_.block<3, 1>(3, i) = t.position.cross(t.direction);
  }

  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(B_);
  cod.setThreshold(1e-9);
  rank_ = static_cast<int>(cod.rank());
  if (rank_ < 6 && !allow_underactuated) {
    throw std::invalid_argument(
            "ThrusterAllocator: allocation matrix rank " + std::to_string(rank_) +
            " < 6; geometry cannot produce all body wrenches");
  }
  B_pinv_ = cod.pseudoInverse();
}

Eigen::VectorXd ThrusterAllocator::allocate_unsaturated(const Vector6d & wrench) const
{
  return B_pinv_ * wrench;
}

double ThrusterAllocator::saturation_scale(const Eigen::VectorXd & thrusts) const
{
  double scale = 1.0;
  for (Eigen::Index i = 0; i < thrusts.size(); ++i) {
    const double u = thrusts(i);
    if (u > max_forward_) {
      scale = std::min(scale, max_forward_ / u);
    } else if (u < -max_reverse_) {
      scale = std::min(scale, max_reverse_ / -u);
    }
  }
  return scale;
}

Eigen::VectorXd ThrusterAllocator::allocate(const Vector6d & wrench) const
{
  if (!wrench.allFinite()) {
    return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(thrusters_.size()));
  }
  Eigen::VectorXd u = allocate_unsaturated(wrench);
  return u * saturation_scale(u);
}

Vector6d ThrusterAllocator::wrench_from_thrusts(const Eigen::VectorXd & thrusts) const
{
  return B_ * thrusts;
}

}  // namespace rov_control
