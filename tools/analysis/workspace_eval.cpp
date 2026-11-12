// Offline kinematic/ground analysis only. No sockets, ROS or robot commands.
#include <Eigen/Dense>
#include <cmath>
#include <fcl/fcl.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
using T = Eigen::Isometry3d;
using V = Eigen::Vector3d;
using M6 = Eigen::Matrix<double, 6, 6>;
struct Hull {
  int link;
  V lo, hi;
  std::vector<V> vertices;
  std::shared_ptr<fcl::CollisionObjectd> collision;
};
T transform(const YAML::Node &n) {
  T t = T::Identity();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      t.matrix()(i, j) = n[4 * i + j].as<double>();
  return t;
}
int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "model.yaml input_f64[N,6] output_f64[N,12]\n";
    return 2;
  }
  try {
    auto model = YAML::LoadFile(argv[1]);
    T origins[6], tcp = transform(model["tcp"]);
    V axes[6];
    double lower[6], upper[6];
    for (int j = 0; j < 6; ++j) {
      origins[j] = transform(model["joint_origins"][j]);
      for (int k = 0; k < 3; ++k)
        axes[j][k] = model["axes"][j][k].as<double>();
      axes[j].normalize();
      lower[j] = model["lower"][j].as<double>();
      upper[j] = model["upper"][j].as<double>();
    }
    std::vector<Hull> hulls;
    for (const auto &node : model["hulls"]) {
      Hull h;
      h.link = node["link"].as<int>();
      h.lo = V::Constant(1e9);
      h.hi = V::Constant(-1e9);
      for (const auto &row : node["vertices"]) {
        V v(row[0].as<double>(), row[1].as<double>(), row[2].as<double>());
        h.vertices.push_back(v);
        h.lo = h.lo.cwiseMin(v);
        h.hi = h.hi.cwiseMax(v);
      }
      if (h.vertices.empty())
        throw std::runtime_error("empty hull");
      auto vertices = std::make_shared<const std::vector<V>>(h.vertices);
      auto faces = std::make_shared<std::vector<int>>();
      for (const auto &face : node["faces"]) {
        faces->push_back(3);
        for (int k = 0; k < 3; ++k)
          faces->push_back(face[k].as<int>());
      }
      auto geometry = std::make_shared<fcl::Convexd>(
          vertices, faces->size() / 4, faces, true);
      h.collision = std::make_shared<fcl::CollisionObjectd>(geometry);
      hulls.push_back(std::move(h));
    }
    const double L = model["characteristic_length_m"].as<double>(),
                 threshold = model["ground_z_m"].as<double>() +
                             model["clearance_m"].as<double>();
    std::ifstream input(argv[2], std::ios::binary);
    std::ofstream output(argv[3], std::ios::binary);
    if (!input || !output)
      throw std::runtime_error("file open failed");
    double q[6];
    size_t count = 0;
    while (input.read(reinterpret_cast<char *>(q), sizeof(q))) {
      T frames[7];
      frames[0] = T::Identity();
      V positions[6], world_axes[6];
      double margin = 1e9;
      bool finite = true;
      for (int j = 0; j < 6; ++j) {
        finite = finite && std::isfinite(q[j]);
        margin = std::min(margin, std::min(q[j] - lower[j], upper[j] - q[j]));
        T before = frames[j] * origins[j];
        positions[j] = before.translation();
        world_axes[j] = before.linear() * axes[j];
        frames[j + 1] = before * Eigen::AngleAxisd(q[j], axes[j]);
      }
      if (!finite)
        throw std::runtime_error("nonfinite joint input");
      T tool = frames[6] * tcp;
      M6 J;
      for (int j = 0; j < 6; ++j) {
        J.block<3, 1>(0, j) =
            world_axes[j].cross(tool.translation() - positions[j]);
        J.block<3, 1>(3, j) = world_axes[j];
      }
      M6 normalized = J;
      normalized.topRows<3>() /= L;
      Eigen::JacobiSVD<M6> svd(normalized,
                               Eigen::ComputeFullU | Eigen::ComputeFullV);
      auto singular = svd.singularValues();
      double sigma = singular[5],
             condition = sigma > 1e-14 ? singular[0] / sigma : 1e15;
      Eigen::Matrix<double, 6, 1> velocity =
          Eigen::Matrix<double, 6, 1>::Zero();
      velocity[2] = 1 / L;
      double gain =
          sigma > 1e-12 ? svd.solve(velocity).cwiseAbs().maxCoeff() : 1e15;
      double minz = 1e9;
      for (const auto &h : hulls) {
        if (h.link == 0)
          continue;
        const auto &pose = frames[h.link];
        V r = pose.linear().row(2).transpose(), center = (h.lo + h.hi) / 2,
          extent = (h.hi - h.lo) / 2;
        double z =
            pose.translation().z() + r.dot(center) - r.cwiseAbs().dot(extent);
        if (z < threshold) {
          z = 1e9;
          for (const V &v : h.vertices)
            z = std::min(z, pose.translation().z() + r.dot(v));
        }
        minz = std::min(minz, z);
        if (minz < threshold)
          break;
      }
      double self_clear = -1, distance_evidence = 1e9;
      if (minz >= threshold && margin >= 0) {
        self_clear = 1;
        for (auto &h : hulls) {
          h.collision->setTransform(frames[h.link]);
          h.collision->computeAABB();
        }
        for (size_t a = 0; a < hulls.size() && self_clear > 0; ++a)
          for (size_t b = a + 1; b < hulls.size(); ++b) {
            if (std::abs(hulls[a].link - hulls[b].link) <= 1)
              continue;
            const auto &aa = hulls[a].collision->getAABB();
            const auto &bb = hulls[b].collision->getAABB();
            V gap = (aa.min_ - bb.max_)
                        .cwiseMax(bb.min_ - aa.max_)
                        .cwiseMax(V::Zero());
            double d = gap.norm();
            if (d < .01) {
              fcl::DistanceResultd answer;
              fcl::distance(hulls[a].collision.get(), hulls[b].collision.get(),
                            fcl::DistanceRequestd(), answer);
              d = answer.min_distance;
            }
            distance_evidence = std::min(distance_evidence, d);
            if (!std::isfinite(d) || d < .01) {
              self_clear = 0;
              break;
            }
          }
      }
      double result[12] = {tool.translation().x(),
                           tool.translation().y(),
                           tool.translation().z(),
                           frames[6].translation().z(),
                           minz,
                           margin,
                           sigma,
                           condition,
                           gain,
                           minz >= threshold ? 1. : 0.,
                           self_clear,
                           distance_evidence};
      output.write(reinterpret_cast<char *>(result), sizeof(result));
      ++count;
    }
    if (!input.eof() || input.gcount() != 0 || !output)
      throw std::runtime_error("incomplete I/O");
    std::cout << count << " configurations evaluated\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
