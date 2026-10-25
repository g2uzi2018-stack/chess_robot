#include "chess_robot_arm/geometry_guard.hpp"
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <yaml-cpp/yaml.h>
namespace chess_robot_arm {
namespace {
using T = Eigen::Isometry3d;
using V = Eigen::Vector3d;
using M6 = Eigen::Matrix<double, 6, 6>;
T transform(const YAML::Node &n) {
  if (!n.IsSequence() || n.size() != 16)
    throw Failure(INVALID_GOAL, "guard transform shape");
  T t = T::Identity();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      t.matrix()(i, j) = n[4 * i + j].as<double>();
  if (!t.matrix().allFinite() ||
      (t.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-9 ||
      (t.linear().transpose() * t.linear() - Eigen::Matrix3d::Identity())
              .norm() > 1e-6 ||
      std::abs(t.linear().determinant() - 1) > 1e-6)
    throw Failure(INVALID_GOAL, "guard transform invalid");
  return t;
}
} // namespace
struct GeometryGuard::Impl {
  Config c;
  std::shared_ptr<const chess_robot_twist::Model> model;
  T origins[6], tcp;
  V axes[6];
  double floor, L;
  struct Hull {
    int link;
    std::vector<V> vertices;
    std::shared_ptr<fcl::CollisionObjectd> object;
  };
  std::vector<Hull> hulls;
  mutable std::mutex mutex;
  Impl(const Config &config, std::shared_ptr<const chess_robot_twist::Model> m)
      : c(config), model(std::move(m)) {
    auto y = YAML::LoadFile(c.safety_model_file);
    tcp = transform(y["tcp"]);
    if ((tcp.matrix() - c.flange_tcp.matrix()).norm() > 1e-8)
      throw Failure(INVALID_GOAL, "guard TCP disagrees with arm configuration");
    const double ground = y["ground_z_m"].as<double>(),
                 clearance = y["clearance_m"].as<double>();
    floor = ground + clearance;
    L = y["characteristic_length_m"].as<double>();
    if (!std::isfinite(floor) || !std::isfinite(L) || L <= 0 ||
        !std::isfinite(clearance) || clearance < .02)
      throw Failure(INVALID_GOAL, "guard floor/normalization invalid");
    for (int i = 0; i < 6; ++i) {
      origins[i] = transform(y["joint_origins"][i]);
      axes[i] = V(y["axes"][i][0].as<double>(), y["axes"][i][1].as<double>(),
                  y["axes"][i][2].as<double>());
      if (!axes[i].allFinite() || std::abs(axes[i].norm() - 1) > 1e-8)
        throw Failure(INVALID_GOAL, "guard joint axis invalid");
      if (std::abs(y["lower"][i].as<double>() - model->lowerLimits()[i]) >
              1e-9 ||
          std::abs(y["upper"][i].as<double>() - model->upperLimits()[i]) > 1e-9)
        throw Failure(INVALID_GOAL, "guard joint limits disagree with URDF");
    }
    bool seen[7] = {};
    for (const auto &n : y["hulls"]) {
      Hull h;
      h.link = n["link"].as<int>();
      if (h.link < 0 || h.link > 6)
        throw Failure(INVALID_GOAL, "guard hull link invalid");
      seen[h.link] = true;
      for (const auto &v : n["vertices"]) {
        V point(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
        if (!point.allFinite())
          throw Failure(INVALID_GOAL, "nonfinite guard vertex");
        h.vertices.push_back(point);
      }
      if (h.vertices.size() < 4)
        throw Failure(INVALID_GOAL, "empty guard hull");
      auto vertices = std::make_shared<const std::vector<V>>(h.vertices);
      auto faces = std::make_shared<std::vector<int>>();
      for (const auto &f : n["faces"]) {
        if (f.size() != 3)
          throw Failure(INVALID_GOAL, "guard face shape");
        faces->push_back(3);
        for (int k = 0; k < 3; ++k) {
          int index = f[k].as<int>();
          if (index < 0 || size_t(index) >= h.vertices.size())
            throw Failure(INVALID_GOAL, "guard face index");
          faces->push_back(index);
        }
      }
      if (faces->empty())
        throw Failure(INVALID_GOAL, "empty guard faces");
      h.object = std::make_shared<fcl::CollisionObjectd>(
          std::make_shared<fcl::Convexd>(vertices, faces->size() / 4, faces,
                                         true));
      hulls.push_back(std::move(h));
    }
    for (bool present : seen)
      if (!present)
        throw Failure(INVALID_GOAL, "missing arm guard hull");
    // The arm hull and fixed camera/gripper envelope must both be present.
    if (hulls.size() < 8)
      throw Failure(INVALID_GOAL, "missing tool guard hull");
    check(c.home);
  }
  void check(const Joints &q) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!model->withinJointBounds(q, 5 * pi / 180))
      throw Failure(PLANNING_FAILED, "guard joint limit margin");
    T frames[7];
    frames[0] = T::Identity();
    V positions[6], world_axes[6];
    for (int j = 0; j < 6; ++j) {
      T before = frames[j] * origins[j];
      positions[j] = before.translation();
      world_axes[j] = before.linear() * axes[j];
      frames[j + 1] = before * Eigen::AngleAxisd(q[j], axes[j]);
    }
    T tool = frames[6] * tcp;
    if ((tool.matrix() - model->tcp(q).matrix()).norm() > 1e-7)
      throw Failure(PLANNING_FAILED, "guard FK disagrees with actual URDF");
    if ((c.backend == "commissioning" || c.backend == "exploration") &&
        (jointDistance(q, c.home) > c.commissioning_joint_radius ||
         (tool.translation() - model->tcp(c.home).translation()).norm() >
             c.commissioning_tcp_radius))
      throw Failure(PLANNING_FAILED, "commissioning Home envelope");
    M6 J;
    for (int j = 0; j < 6; ++j) {
      J.block<3, 1>(0, j) =
          world_axes[j].cross(tool.translation() - positions[j]) / L;
      J.block<3, 1>(3, j) = world_axes[j];
    }
    Eigen::JacobiSVD<M6> svd(J);
    const auto singular = svd.singularValues();
    if (!singular.allFinite() || singular[5] < .05 ||
        singular[0] / singular[5] > 80)
      throw Failure(PLANNING_FAILED, "guard singularity threshold");
    for (auto &h : hulls) {
      if (h.link != 0)
        for (const auto &v : h.vertices)
          if ((frames[h.link] * v).z() < floor)
            throw Failure(PLANNING_FAILED, "guard table clearance");
      h.object->setTransform(frames[h.link]);
      h.object->computeAABB();
    }
    // Same link, the fixed tool group and directly adjacent arm groups are
    // excluded. This intentionally matches the documented offline envelope.
    for (size_t a = 0; a < hulls.size(); ++a)
      for (size_t b = a + 1; b < hulls.size(); ++b) {
        if (std::abs(hulls[a].link - hulls[b].link) <= 1)
          continue;
        const auto &aa = hulls[a].object->getAABB(),
                   &bb = hulls[b].object->getAABB();
        V gap =
            (aa.min_ - bb.max_).cwiseMax(bb.min_ - aa.max_).cwiseMax(V::Zero());
        double distance = gap.norm();
        if (distance < .01) {
          fcl::DistanceResultd result;
          fcl::distance(hulls[a].object.get(), hulls[b].object.get(),
                        fcl::DistanceRequestd(), result);
          distance = result.min_distance;
        }
        if (!std::isfinite(distance) || distance < .01)
          throw Failure(PLANNING_FAILED, "guard self clearance");
      }
  }
};
GeometryGuard::GeometryGuard(const Config &c,
                             std::shared_ptr<const chess_robot_twist::Model> m)
    : impl_(std::make_shared<Impl>(c, std::move(m))) {}
void GeometryGuard::check(const Joints &q) const { impl_->check(q); }
} // namespace chess_robot_arm
