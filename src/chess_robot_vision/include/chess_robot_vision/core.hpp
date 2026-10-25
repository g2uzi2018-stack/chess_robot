#pragma once
#include <opencv2/core.hpp>
#include <vector>

namespace chess_robot_vision {
struct DetectorConfig {
  double min_distance_px{25.0}, edge_threshold{100.0}, accumulator_threshold{30.0};
  int min_radius_px{8}, max_radius_px{90};
};
struct DepthConfig {
  double min_m{0.07}, max_m{0.50}, min_valid_fraction{0.6}, max_mad_m{0.003};
  int min_samples{9};
};
struct DepthEstimate {
  bool valid{false};
  double z_m{0.0}, mad_m{0.0}, valid_fraction{0.0};
  unsigned samples{0};
};
void validate(const DetectorConfig &c);
void validate(const DepthConfig &c);
std::vector<cv::Vec3f> detect_circles(const cv::Mat &bgr, const DetectorConfig &config);
DepthEstimate estimate_depth(const cv::Mat &depth_z16, double scale_m,
                             cv::Point2f center, int patch_radius, const DepthConfig &config);
}
