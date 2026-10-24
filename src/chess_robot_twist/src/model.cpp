#include "chess_robot_twist/model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <urdf/model.h>
#include <utility>

namespace chess_robot_twist {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double two_pi = 2.0 * pi;

void require(bool condition, const std::string &message) {
  if (!condition)
    throw Error(message);
}
void validateOptions(const GeometryOptions &options) {
  require(std::isfinite(options.length_epsilon) && options.length_epsilon > 0,
          "length epsilon must be finite and positive");
  require(std::isfinite(options.antiparallel_epsilon) &&
              options.antiparallel_epsilon > 0 &&
              options.antiparallel_epsilon < 1,
          "antiparallel epsilon must be in (0,1)");
}
void finiteJoints(const Joints &q) {
  for (const double value : q)
    require(std::isfinite(value), "nonfinite joint position");
}
Transform origin(const urdf::Pose &pose) {
  Transform result = Transform::Identity();
  result.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quaternion(pose.rotation.w, pose.rotation.x,
                                pose.rotation.y, pose.rotation.z);
  require(quaternion.coeffs().allFinite() && quaternion.norm() > 1e-12,
          "invalid URDF origin rotation");
  result.linear() = quaternion.normalized().toRotationMatrix();
  require(result.matrix().allFinite(), "nonfinite URDF origin");
  return result;
}
void rigidTransform(const Transform &transform) {
  require(transform.matrix().allFinite(), "nonfinite flange to TCP transform");
  const auto rotation = transform.linear();
  require((rotation.transpose() * rotation)
                  .isApprox(Eigen::Matrix3d::Identity(), 1e-8) &&
              std::abs(rotation.determinant() - 1.0) < 1e-8,
          "flange to TCP rotation is not a proper rotation");
  require(
      transform.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-10),
      "invalid homogeneous transform bottom row");
}
} // namespace

PhaseSample measure(const std::vector<MaterialSpan> &spans,
                    GeometryOptions options) {
  validateOptions(options);
  require(spans.size() >= 2, "rod needs at least two nonzero spans");
  std::vector<Eigen::Vector3d> tangents, directors;
  std::vector<double> lengths;
  tangents.reserve(spans.size());
  directors.reserve(spans.size());
  lengths.reserve(spans.size());
  for (size_t i = 0; i < spans.size(); ++i) {
    const auto &span = spans[i];
    require(span.start.allFinite() && span.end.allFinite() &&
                span.director.allFinite(),
            "nonfinite rod span");
    if (i > 0)
      require((span.start - spans[i - 1].end).norm() <=
                  10 * options.length_epsilon,
              "rod centerline is not contiguous");
    const Eigen::Vector3d edge = span.end - span.start;
    const double length = edge.norm();
    const double director_length = span.director.norm();
    require(std::isfinite(length) && length > options.length_epsilon,
            "degenerate rod span");
    require(std::isfinite(director_length) &&
                director_length > options.length_epsilon,
            "degenerate material director");
    const Eigen::Vector3d tangent = edge / length;
    const Eigen::Vector3d director = span.director / director_length;
    require(std::abs(tangent.dot(director)) < 1e-7,
            "material director is not perpendicular to its span");
    tangents.push_back(tangent);
    directors.push_back(director);
    lengths.push_back(length);
  }
  PhaseSample result;
  result.phase.reserve(spans.size() - 1);
  result.dual_length.reserve(spans.size() - 1);
  for (size_t i = 0; i + 1 < spans.size(); ++i) {
    const auto &a = tangents[i];
    const auto &b = tangents[i + 1];
    const double cosine = std::clamp(a.dot(b), -1.0, 1.0);
    require(cosine > -1.0 + options.antiparallel_epsilon,
            "parallel transport undefined for antiparallel tangents");
    const Eigen::Vector3d cross = a.cross(b);
    const double sine = cross.norm();
    Eigen::Vector3d carried = directors[i];
    if (sine > 1e-12) {
      const Eigen::Vector3d axis = cross / sine;
      carried = cosine * carried + sine * axis.cross(carried) +
                (1.0 - cosine) * axis.dot(carried) * axis;
    }
    const double angle = std::atan2(b.dot(carried.cross(directors[i + 1])),
                                    carried.dot(directors[i + 1]));
    const double dual = 0.5 * (lengths[i] + lengths[i + 1]);
    require(std::isfinite(angle) && std::isfinite(dual) && dual > 0,
            "nonfinite rod phase or dual length");
    result.phase.push_back(angle);
    result.dual_length.push_back(dual);
  }
  return result;
}

Metrics metricsFromLocal(const std::vector<double> &local,
                         const std::vector<double> &dual_lengths) {
  require(!local.empty() && local.size() == dual_lengths.size(),
          "invalid twist vector size");
  Metrics result;
  result.local = local;
  for (size_t i = 0; i < local.size(); ++i) {
    require(std::isfinite(local[i]) && std::isfinite(dual_lengths[i]) &&
                dual_lengths[i] > 0,
            "invalid local twist or dual length");
    result.net += local[i];
    result.total += std::abs(local[i]);
    result.energy += local[i] * local[i] / dual_lengths[i];
  }
  require(std::isfinite(result.net) && std::isfinite(result.total) &&
              std::isfinite(result.energy),
          "twist metric overflow");
  return result;
}

bool withinLimits(const Metrics &metrics, const Limits &limits) {
  require(std::isfinite(limits.net_limit_rad) && limits.net_limit_rad > 0 &&
              std::isfinite(limits.total_limit_rad) &&
              limits.total_limit_rad > 0,
          "twist limits must be finite and positive");
  if (!std::isfinite(metrics.net) || !std::isfinite(metrics.total) ||
      !std::isfinite(metrics.energy) || metrics.total < 0 ||
      metrics.energy < 0 || metrics.total + 1e-10 < std::abs(metrics.net))
    return false;
  return std::abs(metrics.net) <= limits.net_limit_rad + 1e-10 &&
         metrics.total <= limits.total_limit_rad + 1e-10;
}

struct Model::Impl {
  struct Step {
    Transform origin;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    int index = -1;
    std::string child;
  };
  struct Node {
    std::string name;
    Transform transform;
  };
  std::vector<Step> steps;
  std::vector<std::string> node_names;
  std::vector<Eigen::Vector3d> local_directors;
  Joints home{}, lower{}, upper{};
  std::array<std::string, 6> joint_names{};
  Transform flange_T_tcp;
  GeometryOptions options;
  PhaseSample home_phases;
  std::string base;

  Transform flange(const Joints &q) const {
    finiteJoints(q);
    Transform transform = Transform::Identity();
    for (const auto &step : steps) {
      transform = transform * step.origin;
      if (step.index >= 0)
        transform.rotate(Eigen::AngleAxisd(q[step.index], step.axis));
    }
    return transform;
  }
  std::vector<Node> nodes(const Joints &q) const {
    finiteJoints(q);
    std::vector<Node> result{{base, Transform::Identity()}};
    Transform transform = Transform::Identity();
    const auto append = [&](const std::string &name, const Transform &pose) {
      // A zero-length sequence must retain its final outgoing frame. In Nova5
      // this is Link2, not Link1, at the coincident first/second joint origins.
      if ((pose.translation() - result.back().transform.translation()).norm() <=
          options.length_epsilon)
        result.back() = {name, pose};
      else
        result.push_back({name, pose});
    };
    for (const auto &step : steps) {
      transform = transform * step.origin;
      if (step.index >= 0)
        transform.rotate(Eigen::AngleAxisd(q[step.index], step.axis));
      append(step.child, transform);
    }
    append("__grasp_tcp__", transform * flange_T_tcp);
    return result;
  }
  std::vector<MaterialSpan> spans(const Joints &q) const {
    const auto current = nodes(q);
    require(current.size() == node_names.size(),
            "rod topology changed during FK");
    std::vector<MaterialSpan> output;
    output.reserve(local_directors.size());
    for (size_t i = 0; i < current.size(); ++i)
      require(current[i].name == node_names[i],
              "rod node identity changed during FK");
    for (size_t i = 0; i + 1 < current.size(); ++i)
      output.push_back({current[i].transform.translation(),
                        current[i + 1].transform.translation(),
                        current[i].transform.linear() * local_directors[i]});
    return output;
  }
};

Model::Model(const std::string &urdf_file, const std::string &base_link,
             const std::string &flange_link, const Transform &flange_T_tcp,
             const Joints &home, GeometryOptions options) {
  validateOptions(options);
  rigidTransform(flange_T_tcp);
  finiteJoints(home);
  require(
      flange_T_tcp.translation().norm() > options.length_epsilon,
      "TCP offset must define a nonzero terminal span to observe distal spin");
  urdf::Model parsed;
  require(parsed.initFile(urdf_file), "cannot parse robot URDF: " + urdf_file);
  require(parsed.getLink(base_link) != nullptr &&
              parsed.getLink(flange_link) != nullptr,
          "base/flange link missing from URDF");
  std::vector<urdf::JointConstSharedPtr> reverse;
  auto link = parsed.getLink(flange_link);
  while (link && link->name != base_link) {
    require(link->parent_joint != nullptr,
            "flange is not a descendant of requested base");
    reverse.push_back(link->parent_joint);
    link = parsed.getLink(link->parent_joint->parent_link_name);
  }
  require(link && link->name == base_link, "invalid base/flange chain");
  std::reverse(reverse.begin(), reverse.end());
  auto implementation = std::make_shared<Impl>();
  implementation->base = base_link;
  implementation->home = home;
  implementation->flange_T_tcp = flange_T_tcp;
  implementation->options = options;
  size_t movable = 0;
  for (const auto &joint : reverse) {
    Impl::Step step;
    step.origin = origin(joint->parent_to_joint_origin_transform);
    step.child = joint->child_link_name;
    if (joint->type != urdf::Joint::FIXED) {
      require(joint->type == urdf::Joint::REVOLUTE && joint->limits &&
                  movable < 6,
              "model requires exactly six bounded revolute joints");
      step.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
      require(step.axis.allFinite() && step.axis.norm() > 1e-12,
              "invalid joint axis");
      step.axis.normalize();
      step.index = static_cast<int>(movable);
      implementation->lower[movable] = joint->limits->lower;
      implementation->upper[movable] = joint->limits->upper;
      require(std::isfinite(joint->limits->lower) &&
                  std::isfinite(joint->limits->upper) &&
                  joint->limits->lower < joint->limits->upper,
              "invalid URDF joint position bounds");
      require(home[movable] >= joint->limits->lower &&
                  home[movable] <= joint->limits->upper,
              "Home is outside model position bounds");
      implementation->joint_names[movable] = joint->name;
      ++movable;
    }
    implementation->steps.push_back(step);
  }
  require(movable == 6, "model requires exactly six bounded revolute joints");
  const auto nodes = implementation->nodes(home);
  require(nodes.size() >= 3, "rod has too few nonzero spans");
  for (const auto &node : nodes)
    implementation->node_names.push_back(node.name);
  for (size_t i = 0; i + 1 < nodes.size(); ++i) {
    Eigen::Vector3d tangent =
        nodes[i + 1].transform.translation() - nodes[i].transform.translation();
    tangent.normalize();
    const Eigen::Vector3d local =
        nodes[i].transform.linear().transpose() * tangent;
    Eigen::Index axis = 0;
    local.cwiseAbs().minCoeff(&axis);
    const Eigen::Vector3d basis = Eigen::Vector3d::Unit(axis);
    Eigen::Vector3d director = basis - local.dot(basis) * local;
    require(director.norm() > options.length_epsilon,
            "cannot establish material director");
    implementation->local_directors.push_back(director.normalized());
  }
  implementation->home_phases = measure(implementation->spans(home), options);
  impl_ = std::move(implementation);
}

Transform Model::flange(const Joints &q) const { return impl_->flange(q); }
Transform Model::tcp(const Joints &q) const {
  return impl_->flange(q) * impl_->flange_T_tcp;
}
std::vector<MaterialSpan> Model::spans(const Joints &q) const {
  return impl_->spans(q);
}
PhaseSample Model::phases(const Joints &q) const {
  return measure(impl_->spans(q), impl_->options);
}
const PhaseSample &Model::homePhases() const { return impl_->home_phases; }
const Joints &Model::home() const { return impl_->home; }
const Joints &Model::lowerLimits() const { return impl_->lower; }
const Joints &Model::upperLimits() const { return impl_->upper; }
const std::array<std::string, 6> &Model::jointNames() const {
  return impl_->joint_names;
}
const Transform &Model::flangeToTcp() const { return impl_->flange_T_tcp; }
bool Model::withinJointBounds(const Joints &q, double margin_rad) const {
  require(std::isfinite(margin_rad) && margin_rad >= 0,
          "invalid joint position margin");
  for (size_t i = 0; i < q.size(); ++i)
    if (!std::isfinite(q[i]) || q[i] < impl_->lower[i] + margin_rad ||
        q[i] > impl_->upper[i] - margin_rad)
      return false;
  return true;
}

Tracker::Tracker(std::shared_ptr<const Model> model, TrackerOptions options)
    : model_(std::move(model)), options_(options) {
  require(model_ != nullptr, "null rod model");
  require(
      std::isfinite(options_.max_joint_step_rad) &&
          options_.max_joint_step_rad > 0 && options_.max_joint_step_rad < pi &&
          std::isfinite(options_.max_phase_step_rad) &&
          options_.max_phase_step_rad > 0 && options_.max_phase_step_rad < pi,
      "tracker step limits must be finite and inside (0,pi)");
}
void Tracker::initializeAtHome(const Joints &measured, double tolerance_rad) {
  invalidate();
  finiteJoints(measured);
  require(std::isfinite(tolerance_rad) && tolerance_rad >= 0 &&
              tolerance_rad <= options_.max_joint_step_rad,
          "invalid Home reference tolerance");
  for (size_t i = 0; i < measured.size(); ++i)
    require(std::abs(measured[i] - model_->home()[i]) <= tolerance_rad,
            "reference initialization requires measured Home proximity");
  previous_phase_ = model_->homePhases().phase;
  local_.assign(previous_phase_.size(), 0.0);
  last_ = model_->home();
  valid_ = true;
  advance(measured);
}
void Tracker::advance(const Joints &measured) {
  require(valid_, "twist reference is invalid");
  try {
    finiteJoints(measured);
    for (size_t i = 0; i < measured.size(); ++i)
      require(std::abs(measured[i] - last_[i]) <= options_.max_joint_step_rad,
              "joint discontinuity makes winding history ambiguous");
    const auto sample = model_->phases(measured);
    require(sample.phase.size() == previous_phase_.size(),
            "twist dimension changed");
    auto next = local_;
    for (size_t i = 0; i < next.size(); ++i) {
      const double change =
          std::remainder(sample.phase[i] - previous_phase_[i], two_pi);
      require(std::abs(change) <= options_.max_phase_step_rad,
              "material phase jump makes winding history ambiguous");
      next[i] += change;
    }
    auto metrics = metricsFromLocal(next, sample.dual_length);
    local_ = std::move(next);
    previous_phase_ = sample.phase;
    metrics_ = std::move(metrics);
    last_ = measured;
  } catch (...) {
    invalidate();
    throw;
  }
}
void Tracker::invalidate() noexcept { valid_ = false; }
bool Tracker::valid() const noexcept { return valid_; }
const Joints &Tracker::lastJoints() const noexcept { return last_; }
Metrics Tracker::metrics() const {
  require(valid_, "twist reference is invalid");
  return metrics_;
}
} // namespace chess_robot_twist
