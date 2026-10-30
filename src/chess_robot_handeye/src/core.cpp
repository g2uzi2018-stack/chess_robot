#include "chess_robot_handeye/core.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <set>

namespace chess_robot_handeye {
namespace {
std::string resolvePath(const std::string &value,
                        const std::filesystem::path &directory, bool package_ok) {
  if (value.empty()) {
    throw Error("empty configuration path");
  }
  constexpr const char *prefix = "package://";
  if (value.rfind(prefix, 0) == 0) {
    if (!package_ok) {
      throw Error("output_directory must be a filesystem path");
    }
    const auto rest = value.substr(10);
    const auto slash = rest.find('/');
    if (slash == std::string::npos || slash == 0 || slash + 1 == rest.size()) {
      throw Error("package URI requires package name and resource path");
    }
    const std::filesystem::path resource(rest.substr(slash + 1));
    for (const auto &part : resource) {
      if (part == "..") {
        throw Error("package URI cannot contain parent traversal");
      }
    }
    if (resource.is_absolute()) {
      throw Error("package resource must be relative");
    }
    try {
      return (std::filesystem::path(ament_index_cpp::get_package_share_directory(
                  rest.substr(0, slash))) / resource).lexically_normal().string();
    } catch (const std::exception &e) {
      throw Error(std::string("cannot resolve package URI: ") + e.what());
    }
  }
  const std::filesystem::path p(value);
  return (p.is_absolute() ? p : directory / p).lexically_normal().string();
}
template <class V> void read(const YAML::Node &n, const char *k, V &v) {
  if (n[k]) {
    v = n[k].as<V>();
  }
}
template <int N> cv::Vec<double, N> vec(const YAML::Node &n) {
  if (!n.IsSequence() || n.size() != N) {
    throw Error("invalid vector length");
  }
  cv::Vec<double, N> v;
  for (int i = 0; i < N; ++i) {
    v[i] = n[i].as<double>();
    if (!std::isfinite(v[i])) {
      throw Error("nonfinite vector");
    }
  }
  return v;
}
cv::Matx33d rot(const T &t) {
  return {t(0, 0), t(0, 1), t(0, 2), t(1, 0), t(1, 1),
          t(1, 2), t(2, 0), t(2, 1), t(2, 2)};
}
cv::Vec3d tr(const T &t) { return {t(0, 3), t(1, 3), t(2, 3)}; }
T compose(const cv::Mat &r, const cv::Mat &t) {
  T a = T::eye();
  for (int i = 0; i < 3; ++i) {
    a(i, 3) = t.at<double>(i);
    for (int j = 0; j < 3; ++j) {
      a(i, j) = r.at<double>(i, j);
    }
  }
  return a;
}
} // namespace
Config loadConfig(const std::string &path) {
  auto n = YAML::LoadFile(path);
  Config c;
  read(n, "mode", c.mode);
  read(n, "output_directory", c.output);
  auto e = n["execution"];
  read(e, "enabled", c.enabled);
  read(e, "fixture", c.fixture);
  read(e, "poses_confirmed", c.poses_confirmed);
  auto a = n["arm"];
  read(a, "action", c.action);
  read(a, "state_topic", c.state);
  read(a, "joint_topic", c.joints);
  read(a, "parameter_node", c.arm_node);
  read(a, "profile", c.profile);
  read(a, "expected_approach_clearance_m", c.clearance);
  auto m = n["model"];
  read(m, "urdf", c.urdf);
  read(m, "base_frame", c.base);
  read(m, "flange_frame", c.flange);
  read(m, "home_rad", c.home);
  if (m["flange_to_tcp_translation_m"]) {
    c.tcp_p = vec<3>(m["flange_to_tcp_translation_m"]);
  }
  if (m["flange_to_tcp_rpy_rad"]) {
    c.tcp_rpy = vec<3>(m["flange_to_tcp_rpy_rad"]);
  }
  auto v = n["vision"];
  read(v, "image_topic", c.image);
  read(v, "camera_info_topic", c.info);
  read(v, "frame_info_topic", c.frame);
  read(v, "optical_frame", c.optical);
  auto b = n["board"];
  read(b, "confirmed", c.board.confirmed);
  read(b, "squares_x", c.board.squares_x);
  read(b, "squares_y", c.board.squares_y);
  read(b, "square_length_m", c.board.square_m);
  read(b, "marker_length_m", c.board.marker_m);
  read(b, "dictionary_id", c.board.dictionary);
  read(b, "legacy_pattern", c.board.legacy);
  read(b, "marker_ids", c.board.ids);
  auto q = n["quality"];
#define R(key, field) read(q, key, c.field)
  R("stationary_dwell_s", dwell);
  R("feedback_freshness_s", fresh);
  R("maximum_feedback_gap_s", gap);
  R("pair_tolerance_s", pair);
  R("stationary_speed_rad_s", speed);
  R("settle_timeout_s", settle_timeout);
  R("capture_timeout_s", capture_timeout);
  R("action_timeout_s", action_timeout);
  R("position_tolerance_m", position_tol);
  R("rotation_tolerance_rad", rotation_tol);
  R("maximum_reprojection_px", reprojection_px);
  R("minimum_rotation_rad", minimum_rotation_rad);
  R("minimum_axis_sine", axis_sine);
  R("validation_translation_m", validation_m);
  R("validation_rotation_rad", validation_rad);
  R("minimum_corners", min_corners);
  R("minimum_training", min_training);
  R("minimum_validation", min_validation);
#undef R
  if (n["poses"]) {
    for (const auto &p : n["poses"]) {
      Pose s;
      s.id = p["id"].as<std::string>();
      s.role = p["role"].as<std::string>();
      s.p = vec<3>(p["position_m"]);
      s.q = vec<4>(p["quaternion_xyzw"]);
      c.poses.push_back(s);
    }
  }
  const auto directory = std::filesystem::absolute(path).parent_path();
  c.output = resolvePath(c.output, directory, false);
  c.urdf = resolvePath(c.urdf, directory, true);
  validateConfig(c, false);
  return c;
}
void validateConfig(const Config &c, bool ready) {
  if (c.mode != "validate" && c.mode != "plan" && c.mode != "execute") {
    throw Error("mode must be validate, plan, execute");
  }
  if (!std::filesystem::path(c.output).is_absolute() ||
      !std::filesystem::path(c.urdf).is_absolute()) {
    throw Error("absolute output_directory and model.urdf required");
  }
  if (c.base.empty() || c.flange.empty() || c.optical.empty() ||
      c.home.size() != 6) {
    throw Error("frames and six model home joints required");
  }
  for (double x : c.home) {
    if (!std::isfinite(x)) {
      throw Error("nonfinite home");
    }
  }
  for (double x :
       {c.clearance, c.dwell, c.fresh, c.gap, c.pair, c.speed, c.settle_timeout,
        c.capture_timeout, c.action_timeout, c.position_tol, c.rotation_tol,
        c.reprojection_px, c.minimum_rotation_rad, c.axis_sine, c.validation_m,
        c.validation_rad}) {
    if (!std::isfinite(x) || x < 0) {
      throw Error("invalid quality setting");
    }
  }
  if (c.dwell < 0.4 || c.fresh <= 0 || c.fresh > 0.2 || c.gap <= 0 ||
      c.gap > c.fresh || c.pair <= 0 || c.pair > c.fresh || c.speed <= 0 ||
      c.settle_timeout <= c.dwell || c.capture_timeout <= 0 ||
      c.action_timeout <= 0 || c.position_tol <= 0 || c.rotation_tol <= 0 ||
      c.reprojection_px <= 0 || c.minimum_rotation_rad <= 0 ||
      c.axis_sine <= 0 || c.axis_sine > 1 || c.validation_m <= 0 ||
      c.validation_rad <= 0 || c.min_corners < 6 || c.min_training < 3 ||
      c.min_validation < 1) {
    throw Error("quality bounds invalid");
  }
  std::set<std::string> names;
  for (const auto &p : c.poses) {
    if (p.id.empty() || !names.insert(p.id).second ||
        p.id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTU"
                               "VWXYZ0123456789_-") != std::string::npos) {
      throw Error("invalid/duplicate pose id");
    }
    if (p.role != "train" && p.role != "validation") {
      throw Error("pose role must be train or validation");
    }
    poseMatrix(p.p, p.q);
  }
  if (c.mode != "validate" && c.poses.empty()) {
    throw Error("plan/execute requires a finite nonempty pose list");
  }
  const std::vector<std::string> endpoints = {
      c.action, c.state, c.joints, c.arm_node, c.image, c.info, c.frame};
  for (const auto &s : endpoints) {
    if (s.empty() || s[0] != '/') {
      throw Error("absolute ROS endpoints required");
    }
    bool sim = s.rfind("/simulation/", 0) == 0;
    if (sim != c.fixture) {
      throw Error("fixture endpoints must all use /simulation/; real endpoints "
                  "must not");
    }
  }
  if (ready) {
    if (!c.enabled || !c.poses_confirmed || !c.board.confirmed) {
      throw Error("execute requires explicit enabled, poses_confirmed and "
                  "board.confirmed");
    }
    makeBoard(c.board);
    int train = 0, val = 0;
    for (auto &p : c.poses) {
      (p.role == "train" ? train : val)++;
    }
    if (train < c.min_training || val < c.min_validation) {
      throw Error("insufficient configured training/validation poses");
    }
  }
}
T poseMatrix(const cv::Vec3d &p, const cv::Vec4d &q) {
  for (double x : p.val) {
    if (!std::isfinite(x)) {
      throw Error("nonfinite position");
    }
  }
  for (double x : q.val) {
    if (!std::isfinite(x)) {
      throw Error("nonfinite quaternion");
    }
  }
  if (std::abs(cv::norm(q) - 1) > 1e-6) {
    throw Error("quaternion must be normalized");
  }
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  T t = T::eye();
  cv::Matx33d r(
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y));
  for (int i = 0; i < 3; ++i) {
    t(i, 3) = p[i];
    for (int j = 0; j < 3; ++j) {
      t(i, j) = r(i, j);
    }
  }
  return t;
}
T inverse(const T &t) {
  T r = T::eye();
  auto R = rot(t).t();
  auto p = -(R * tr(t));
  for (int i = 0; i < 3; ++i) {
    r(i, 3) = p[i];
    for (int j = 0; j < 3; ++j) {
      r(i, j) = R(i, j);
    }
  }
  return r;
}
double rotationDistance(const T &a, const T &b) {
  auto r = rot(a).t() * rot(b);
  return std::acos(std::clamp((cv::trace(r) - 1) * 0.5, -1.0, 1.0));
}
double translationDistance(const T &a, const T &b) {
  return cv::norm(tr(a) - tr(b));
}
bool validTransform(const T &t) {
  for (double x : t.val) {
    if (!std::isfinite(x)) {
      return false;
    }
  }
  auto r = rot(t);
  return cv::norm(cv::Mat(r.t() * r - cv::Matx33d::eye())) < 1e-6 &&
         std::abs(cv::determinant(r) - 1) < 1e-6 &&
         std::abs(t(3, 0)) + std::abs(t(3, 1)) + std::abs(t(3, 2)) +
                 std::abs(t(3, 3) - 1) <
             1e-9;
}
cv::Ptr<cv::aruco::CharucoBoard> makeBoard(const Board &b) {
  if (b.squares_x < 3 || b.squares_y < 3 || b.square_m <= 0 ||
      b.marker_m <= 0 || b.marker_m >= b.square_m ||
      !std::isfinite(b.square_m) || !std::isfinite(b.marker_m) ||
      b.dictionary < 0 || b.dictionary > 16) {
    throw Error("invalid/unconfirmed board geometry or dictionary (OpenCV "
                "predefined 0..16)");
  }
  if (!b.legacy) {
    throw Error(
        "this implementation uses OpenCV 4.5 legacy ChArUco layout; nonlegacy "
        "boards require an explicit compatible implementation");
  }
  auto d = cv::aruco::getPredefinedDictionary(b.dictionary);
  auto board = cv::aruco::CharucoBoard::create(b.squares_x, b.squares_y,
                                               b.square_m, b.marker_m, d);
  if (!b.ids.empty()) {
    if (b.ids.size() != board->ids.size()) {
      throw Error("marker_ids size must match every board marker");
    }
    std::set<int> ids;
    for (int x : b.ids) {
      if (x < 0 || x >= d->bytesList.rows || !ids.insert(x).second) {
        throw Error("invalid/duplicate marker id");
      }
    }
    board->ids = b.ids;
  }
  if (board->ids.back() >= d->bytesList.rows) {
    throw Error("dictionary has fewer markers than board");
  }
  return board;
}
Detection detectBoard(const cv::Mat &image, const cv::Matx33d &K,
                      const Board &b, int minimum, double limit) {
  if (image.empty() || image.type() != CV_8UC3 || !cv::checkRange(cv::Mat(K)) ||
      K(0, 0) <= 0 || K(1, 1) <= 0) {
    throw Error("invalid BGR image or intrinsics");
  }
  auto board = makeBoard(b);
  std::vector<std::vector<cv::Point2f>> markers;
  std::vector<std::vector<cv::Point2f>> rejected;
  std::vector<int> marker_ids;
  auto params = cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(image, board->dictionary, markers, marker_ids,
                           params, rejected);
  if (markers.empty()) {
    throw Error("no board markers detected");
  }
  const bool board_anchor =
      std::any_of(marker_ids.begin(), marker_ids.end(), [&](int id) {
        return std::find(board->ids.begin(), board->ids.end(), id) !=
               board->ids.end();
      });
  if (!board_anchor) {
    throw Error("no configured-board marker anchor detected");
  }
  Detection out;
  cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
  // Recover rejected marker candidates using the known board and original
  // rectified image. Do not change detector defaults or quality thresholds.
  cv::aruco::refineDetectedMarkers(image, board, markers, marker_ids, rejected,
                                   cv::Mat(K), D);
  cv::aruco::interpolateCornersCharuco(markers, marker_ids, image, board,
                                       out.corners, out.ids, cv::Mat(K), D);
  if (int(out.ids.size()) < minimum ||
      cv::aruco::testCharucoCornersCollinear(board, out.ids)) {
    throw Error("insufficient/distributed ChArUco corners");
  }
  std::vector<cv::Point3f> objects;
  for (int id : out.ids) {
    if (id < 0 || size_t(id) >= board->chessboardCorners.size()) {
      throw Error("corner id out of range");
    }
    objects.push_back(board->chessboardCorners[id]);
  }
  cv::Vec3d r, t;
  if (!cv::solvePnP(objects, out.corners, K, D, r, t)) {
    throw Error("PnP failed");
  }
  cv::Mat R;
  cv::Rodrigues(r, R);
  out.camera_T_board = compose(R, cv::Mat(t));
  for (auto &p : objects) {
    cv::Vec3d q = rot(out.camera_T_board) * cv::Vec3d(p.x, p.y, p.z) + t;
    if (q[2] <= 0) {
      throw Error("board behind camera");
    }
  }
  std::vector<cv::Point2f> projected;
  cv::projectPoints(objects, r, t, K, D, projected);
  double sum = 0;
  for (size_t i = 0; i < projected.size(); ++i) {
    auto v = projected[i] - out.corners[i];
    sum += v.dot(v);
  }
  out.reprojection_px = std::sqrt(sum / projected.size());
  if (!validTransform(out.camera_T_board) ||
      !std::isfinite(out.reprojection_px) || out.reprojection_px > limit) {
    throw Error("PnP reprojection/transform rejected");
  }
  return out;
}
Solution solve(const std::vector<Sample> &samples, const Config &c) {
  std::vector<Sample> train, val;
  for (auto &s : samples) {
    if (s.role != "train" && s.role != "validation") {
      throw Error("unknown sample role");
    }
    if (!validTransform(s.base_T_flange) || !validTransform(s.camera_T_board)) {
      throw Error("invalid sample transform");
    }
    (s.role == "train" ? train : val).push_back(s);
  }
  if (int(train.size()) < c.min_training ||
      int(val.size()) < c.min_validation) {
    throw Error("insufficient train/independent validation samples");
  }
  std::vector<cv::Vec3d> axes;
  for (size_t i = 0; i < train.size(); ++i) {
    for (size_t j = i + 1; j < train.size(); ++j) {
      cv::Mat rv;
      cv::Rodrigues(
          rot(inverse(train[i].base_T_flange) * train[j].base_T_flange), rv);
      cv::Vec3d r(rv.at<double>(0), rv.at<double>(1), rv.at<double>(2));
      double a = cv::norm(r);
      if (a >= c.minimum_rotation_rad && a < 3.0) {
        axes.push_back(r / a);
      }
    }
  }
  bool diverse = false;
  for (auto &a : axes) {
    for (auto &b : axes) {
      if (cv::norm(a.cross(b)) >= c.axis_sine) {
        diverse = true;
      }
    }
  }
  if (!diverse) {
    throw Error("degenerate rotation set: need nonparallel rotation axes");
  }
  std::vector<cv::Mat> Rg, tg, Rc, tc;
  for (auto &s : train) {
    Rg.emplace_back(rot(s.base_T_flange));
    tg.emplace_back(tr(s.base_T_flange));
    Rc.emplace_back(rot(s.camera_T_board));
    tc.emplace_back(tr(s.camera_T_board));
  }
  cv::Mat R, t;
  cv::calibrateHandEye(Rg, tg, Rc, tc, R, t, cv::CALIB_HAND_EYE_PARK);
  Solution out;
  out.flange_T_camera = compose(R, t);
  if (!validTransform(out.flange_T_camera)) {
    throw Error("nonfinite/nonrigid hand-eye result");
  }
  // Use a training-only board estimate, never fit the held-out validation
  // views.
  T reference = train.front().base_T_flange * out.flange_T_camera *
                train.front().camera_T_board;
  for (auto &s : train) {
    T board = s.base_T_flange * out.flange_T_camera * s.camera_T_board;
    out.train_max_m =
        std::max(out.train_max_m, translationDistance(board, reference));
    out.train_max_rad =
        std::max(out.train_max_rad, rotationDistance(board, reference));
  }
  for (auto &s : val) {
    T board = s.base_T_flange * out.flange_T_camera * s.camera_T_board;
    out.validation_max_m =
        std::max(out.validation_max_m, translationDistance(board, reference));
    out.validation_max_rad =
        std::max(out.validation_max_rad, rotationDistance(board, reference));
  }
  out.training = train.size();
  out.validation = val.size();
  out.accepted = out.train_max_m <= c.validation_m &&
                 out.train_max_rad <= c.validation_rad &&
                 out.validation_max_m <= c.validation_m &&
                 out.validation_max_rad <= c.validation_rad;
  return out;
}
YAML::Node matrixYaml(const T &t) {
  YAML::Node n;
  for (int i = 0; i < 4; ++i) {
    YAML::Node row;
    for (int j = 0; j < 4; ++j) {
      row.push_back(t(i, j));
    }
    n.push_back(row);
  }
  return n;
}
void StillWindow::reset() { first_ = last_ = -1; }
bool StillWindow::update(double stamp, bool healthy) {
  if (!healthy || !std::isfinite(stamp) || stamp <= 0) {
    reset();
    return false;
  }
  if (last_ >= 0 && (stamp <= last_ || stamp - last_ > gap_)) {
    first_ = stamp;
    last_ = stamp;
    return false;
  }
  if (first_ < 0) {
    first_ = stamp;
  }
  last_ = stamp;
  return stamp - first_ >= dwell_;
}
} // namespace chess_robot_handeye
