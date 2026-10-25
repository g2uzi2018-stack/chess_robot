#include "chess_robot_vision/core.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace chess_robot_vision {
namespace {
double median(std::vector<double> v) {
  const auto n = v.size();
  std::nth_element(v.begin(), v.begin() + n / 2, v.end());
  const double upper = v[n / 2];
  if (n % 2) return upper;
  return (upper + *std::max_element(v.begin(), v.begin() + n / 2)) * 0.5;
}
}
void validate(const DetectorConfig &c) {
  if (!std::isfinite(c.min_distance_px) || c.min_distance_px <= 0 ||
      !std::isfinite(c.edge_threshold) || c.edge_threshold <= 0 ||
      !std::isfinite(c.accumulator_threshold) || c.accumulator_threshold <= 0 ||
      c.min_radius_px < 1 || c.max_radius_px < c.min_radius_px)
    throw std::invalid_argument("invalid circle detector parameters");
}
void validate(const DepthConfig &c) {
  if (!std::isfinite(c.min_m) || !std::isfinite(c.max_m) || c.min_m <= 0 || c.max_m <= c.min_m ||
      !std::isfinite(c.min_valid_fraction) || c.min_valid_fraction <= 0 || c.min_valid_fraction > 1 ||
      !std::isfinite(c.max_mad_m) || c.max_mad_m < 0 || c.min_samples < 1)
    throw std::invalid_argument("invalid depth quality parameters");
}
std::vector<cv::Vec3f> detect_circles(const cv::Mat &bgr, const DetectorConfig &c) {
  validate(c);
  if (bgr.empty() || bgr.type() != CV_8UC3) throw std::invalid_argument("expected BGR8 image");
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(5,5), 1.5);
  std::vector<cv::Vec3f> out;
  cv::HoughCircles(gray, out, cv::HOUGH_GRADIENT, 1, c.min_distance_px,
                   c.edge_threshold, c.accumulator_threshold, c.min_radius_px, c.max_radius_px);
  std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
    return a[1] == b[1] ? a[0] < b[0] : a[1] < b[1];
  });
  return out;
}
DepthEstimate estimate_depth(const cv::Mat &depth, double scale, cv::Point2f center,
                             int radius, const DepthConfig &c) {
  validate(c);
  if (depth.empty() || depth.type() != CV_16UC1 || !std::isfinite(scale) || scale <= 0 || radius < 1 || radius > 100)
    throw std::invalid_argument("invalid Z16 depth image/scale/patch");
  DepthEstimate out;
  if (!std::isfinite(center.x) || !std::isfinite(center.y) || center.x < 0 || center.y < 0 ||
      center.x >= depth.cols || center.y >= depth.rows) return out;
  const int cx = cvRound(center.x), cy = cvRound(center.y);
  std::vector<double> values;
  unsigned total = 0;
  for (int dy=-radius; dy<=radius; ++dy) for (int dx=-radius; dx<=radius; ++dx) {
    if (dx*dx+dy*dy > radius*radius) continue;
    ++total;  // Out-of-image samples count as invalid, never clamp onto the border.
    const int x=cx+dx, y=cy+dy;
    if (x<0 || y<0 || x>=depth.cols || y>=depth.rows) continue;
    const double z = depth.at<uint16_t>(y,x) * scale;
    if (z >= c.min_m && z <= c.max_m) values.push_back(z);
  }
  out.samples = values.size();
  out.valid_fraction = static_cast<double>(values.size()) / total;
  if (values.empty()) return out;
  out.z_m = median(values);
  std::vector<double> deviations;
  for (double z : values) deviations.push_back(std::abs(z-out.z_m));
  out.mad_m = median(deviations);
  out.valid = static_cast<int>(values.size()) >= c.min_samples &&
              out.valid_fraction >= c.min_valid_fraction && out.mad_m <= c.max_mad_m;
  return out;
}
}
