#pragma once
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
namespace chess_robot_vision {
// 生成去畸变映射：map_x、map_y 为输出像素对应的原图坐标，单位为像素。
// 输出尺寸与 source 相同，覆盖原有映射；校验失败时抛出 std::invalid_argument。
void rectification_maps(const rs2_intrinsics &source, cv::Mat &map_x, cv::Mat &map_y);

// 返回无畸变的内参副本，保留焦距、主点和尺寸；不处理图像。
rs2_intrinsics pinhole_intrinsics(rs2_intrinsics source);
}
