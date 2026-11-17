// 用相机已有内参生成校正映射，不在这里重新标定相机。

#include "chess_robot_vision/calibration.hpp"
#include <librealsense2/rsutil.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace chess_robot_vision {

// 返回无畸变的内参副本；只改参数描述，不处理图像。
rs2_intrinsics pinhole_intrinsics(rs2_intrinsics source) {
  source.model = RS2_DISTORTION_NONE;
  std::fill(std::begin(source.coeffs), std::end(source.coeffs), 0.0f);
  return source;
}

// 计算每个校正后像素在原图中的采样位置，写入 map_x、map_y。
void rectification_maps(const rs2_intrinsics &s, cv::Mat &map_x, cv::Mat &map_y) {
  // 尺寸或焦距不大于 0、焦距或主点含 NaN/无穷大、畸变模型不支持时，停止计算。
  // 仅支持无畸变、Brown-Conrady 和 inverse-Brown-Conrady 模型。
  if (s.width < 1 || s.height < 1 || !std::isfinite(s.fx) || !std::isfinite(s.fy) ||
      !std::isfinite(s.ppx) || !std::isfinite(s.ppy) || s.fx <= 0 || s.fy <= 0 ||
      (s.model != RS2_DISTORTION_NONE && s.model != RS2_DISTORTION_BROWN_CONRADY &&
       s.model != RS2_DISTORTION_INVERSE_BROWN_CONRADY))
    throw std::invalid_argument("unsupported or invalid source calibration");
  // 任一畸变系数是 NaN 或无穷大时，停止计算。
  for (float c : s.coeffs)
    if (!std::isfinite(c))
      throw std::invalid_argument("nonfinite distortion coefficient");

  map_x.create(s.height, s.width, CV_32F);
  map_y.create(s.height, s.width, CV_32F);

  for (int y = 0; y < s.height; ++y)
    for (int x = 0; x < s.width; ++x) {
      // 从校正后的像素位置，反查原图中的采样位置。
      const float ray[3] = {(x - s.ppx) / s.fx, (y - s.ppy) / s.fy, 1};
      float pixel[2];
      rs2_project_point_to_pixel(pixel, &s, ray);

      // 有畸变时，用 SDK 反投影结果修正采样位置，减小正反投影偏差。
      // 最多修正 5 次；次数上限与误差阈值的依据待确认。
      if (s.model != RS2_DISTORTION_NONE)
        for (int iteration = 0; iteration < 5; ++iteration) {
          float back[3];
          rs2_deproject_pixel_to_point(back, &s, pixel, 1);
          const float dx = (ray[0] - back[0]) * s.fx, dy = (ray[1] - back[1]) * s.fy;
          // 两轴偏差都小于 0.001 像素时，结束本像素的修正。
          if (std::abs(dx) < 0.001f && std::abs(dy) < 0.001f)
            break;
          pixel[0] += dx;
          pixel[1] += dy;
        }

      map_x.at<float>(y, x) = pixel[0];
      map_y.at<float>(y, x) = pixel[1];
    }
}

}
