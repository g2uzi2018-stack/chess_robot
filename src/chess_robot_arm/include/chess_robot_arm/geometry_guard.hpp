#pragma once
#include "chess_robot_arm/core.hpp"
namespace chess_robot_arm {
// The table_contact profile is a vertical, bounded contact-stage operation.
void validateTableContactGoal(const Goal &, const Transform &start, const Config &);
// Sampled conservative checks using geometry exported from this exact URDF.
// This is not continuous collision checking or a calibrated environment model.
// Calls are serialized internally because FCL objects carry mutable transforms.
class GeometryGuard {
  struct Impl;
  std::shared_ptr<Impl> impl_;

public:
  GeometryGuard(const Config &,
                std::shared_ptr<const chess_robot_twist::Model>);
  void check(const Joints &) const;
  // Only moving finger hulls may use configured (at least 2mm) target-local clearance.
  // Caller must restrict this to a checked vertical DESCEND/RETREAT operation.
  void checkTableContact(const Joints &, const Eigen::Vector2d &target_xy) const;
};
} // namespace chess_robot_arm
