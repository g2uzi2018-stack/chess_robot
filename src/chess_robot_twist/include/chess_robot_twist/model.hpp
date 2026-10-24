#pragma once
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace chess_robot_twist {
using Joints = std::array<double, 6>;
using Transform = Eigen::Isometry3d;

class Error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct MaterialSpan {
  Eigen::Vector3d start;
  Eigen::Vector3d end;
  Eigen::Vector3d
      director; // Unit material direction perpendicular to end-start.
};
struct PhaseSample {
  std::vector<double> phase;       // Principal local twist angles, radians.
  std::vector<double> dual_length; // Mean adjacent span lengths, metres.
};
struct Metrics {
  double net = 0.0;   // Signed sum of unwrapped local deviations, radians.
  double total = 0.0; // Sum of absolute local deviations, radians.
  double energy =
      0.0; // Sum beta_i^2 / dual_length_i; geometric proxy, rad^2/m.
  std::vector<double> local;
};
struct GeometryOptions {
  double length_epsilon = 1e-9;
  double antiparallel_epsilon = 1e-9;
};
struct TrackerOptions {
  double max_joint_step_rad = 0.1;
  double max_phase_step_rad = 1.5707963267948966;
};
struct Limits {
  double net_limit_rad = 6.283185307179586;
  double total_limit_rad = 6.283185307179586;
};

// Minimal-rotation transport between consecutive tangents. No forces/material
// properties are simulated. Invalid/antiparallel geometry throws Error.
PhaseSample measure(const std::vector<MaterialSpan> &spans,
                    GeometryOptions options = {});
Metrics metricsFromLocal(const std::vector<double> &local,
                         const std::vector<double> &dual_lengths);
bool withinLimits(const Metrics &metrics, const Limits &limits);

// FK geometry of a six-revolute-joint chain and fixed flange->TCP transform.
// Consecutive coincident origins collapse to the final outgoing material frame.
// Link-local directors are chosen once at Home, never dynamically reselected.
class Model {
public:
  Model(const std::string &urdf_file, const std::string &base_link,
        const std::string &flange_link, const Transform &flange_T_tcp,
        const Joints &home, GeometryOptions options = {});
  Transform flange(const Joints &q) const;
  Transform tcp(const Joints &q) const;
  std::vector<MaterialSpan> spans(const Joints &q) const;
  PhaseSample phases(const Joints &q) const;
  const PhaseSample &homePhases() const;
  const Joints &home() const;
  const Joints &lowerLimits() const;
  const Joints &upperLimits() const;
  const std::array<std::string, 6> &jointNames() const;
  const Transform &flangeToTcp() const;
  bool withinJointBounds(const Joints &q, double margin_rad = 0.0) const;

private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

// Copying a Tracker creates an independent planning snapshot. Planning MUST use
// a copy: only actual fresh feedback may advance the live tracker. No automatic
// initialization/recovery; initializeAtHome is an explicit operator operation.
class Tracker {
public:
  explicit Tracker(std::shared_ptr<const Model> model,
                   TrackerOptions options = {});
  void initializeAtHome(const Joints &measured, double tolerance_rad);
  void advance(const Joints &measured);
  void invalidate() noexcept;
  bool valid() const noexcept;
  const Joints &lastJoints() const noexcept;
  Metrics metrics() const; // Throws while reference invalid.
private:
  std::shared_ptr<const Model> model_;
  TrackerOptions options_;
  bool valid_ = false;
  Joints last_{};
  std::vector<double> previous_phase_;
  std::vector<double> local_;
  Metrics metrics_;
};
} // namespace chess_robot_twist
