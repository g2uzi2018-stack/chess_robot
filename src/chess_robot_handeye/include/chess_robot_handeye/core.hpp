#pragma once
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace chess_robot_handeye {
using T = cv::Matx44d;
struct Error : std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct Pose {
  std::string id, role;
  cv::Vec3d p;
  cv::Vec4d q;
};
struct Board {
  int squares_x = 0, squares_y = 0, dictionary = -1;
  double square_m = 0, marker_m = 0;
  bool confirmed = false, legacy = true;
  std::vector<int> ids;
};
struct Config {
  std::string mode = "validate", output, urdf, base = "base_link",
              flange = "Link6";
  std::string action = "/arm/move", state = "/arm/state",
              joints = "/arm/joint_states", arm_node = "/chess_robot_arm";
  std::string image = "/vision/color/image_rect",
              info = "/vision/color/camera_info", frame = "/vision/frame_info";
  std::string optical = "d405_color_optical_frame", profile = "low";
  bool fixture = false, enabled = false, poses_confirmed = false;
  double clearance = 0.25, dwell = 0.6, fresh = 0.1, gap = 0.1, pair = 0.025,
         speed = 0.002;
  double settle_timeout = 10, capture_timeout = 10, action_timeout = 90,
         position_tol = 0.001, rotation_tol = 0.005;
  double reprojection_px = 1.0, minimum_rotation_rad = 0.05, axis_sine = 0.2;
  double validation_m = 0.005, validation_rad = 0.03;
  int min_corners = 12, min_training = 8, min_validation = 3;
  std::vector<double> home;
  cv::Vec3d tcp_p{0, 0, 0.2},
      tcp_rpy{1.5707963267948966, -1.5707963267948966, 0};
  Board board;
  std::vector<Pose> poses;
};
Config loadConfig(const std::string &path);
void validateConfig(const Config &, bool require_ready);
T poseMatrix(const cv::Vec3d &, const cv::Vec4d &xyzw);
T inverse(const T &);
double rotationDistance(const T &, const T &);
double translationDistance(const T &, const T &);
bool validTransform(const T &);
cv::Ptr<cv::aruco::CharucoBoard> makeBoard(const Board &);
struct Detection {
  T camera_T_board;
  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  double reprojection_px = 0;
};
Detection detectBoard(const cv::Mat &bgr, const cv::Matx33d &K, const Board &,
                      int min_corners, double max_rms);
struct Sample {
  T base_T_flange, camera_T_board;
  std::string id, role;
};
struct Solution {
  T flange_T_camera;
  double train_max_m = 0, train_max_rad = 0, validation_max_m = 0,
         validation_max_rad = 0;
  size_t training = 0, validation = 0;
  bool accepted = false;
};
Solution solve(const std::vector<Sample> &, const Config &);
YAML::Node matrixYaml(const T &);
// A continuous receipt-time window. Duplicate/backward samples and gaps reset
// it.
class StillWindow {
public:
  StillWindow(double dwell, double max_gap) : dwell_(dwell), gap_(max_gap) {}
  bool update(double stamp, bool healthy);
  void reset();

private:
  double dwell_, gap_, first_ = -1, last_ = -1;
};
} // namespace chess_robot_handeye