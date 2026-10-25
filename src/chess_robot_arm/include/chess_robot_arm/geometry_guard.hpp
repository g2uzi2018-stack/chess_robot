#pragma once
#include "chess_robot_arm/core.hpp"
namespace chess_robot_arm {
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
};
} // namespace chess_robot_arm
