#pragma once
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
namespace chess_robot_vision {
// Map an undistorted pinhole output plane back into the SDK-calibrated color image.
// Inverse Brown coefficients are never passed off as ROS plumb_bob coefficients.
void rectification_maps(const rs2_intrinsics &source, cv::Mat &map_x, cv::Mat &map_y);
rs2_intrinsics pinhole_intrinsics(rs2_intrinsics source);
}
