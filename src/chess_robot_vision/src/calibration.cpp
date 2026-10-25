#include "chess_robot_vision/calibration.hpp"
#include <librealsense2/rsutil.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace chess_robot_vision {
rs2_intrinsics pinhole_intrinsics(rs2_intrinsics source) {
  source.model=RS2_DISTORTION_NONE;
  std::fill(std::begin(source.coeffs),std::end(source.coeffs),0.0f);
  return source;
}
void rectification_maps(const rs2_intrinsics &s,cv::Mat &map_x,cv::Mat &map_y) {
  if (s.width<1 || s.height<1 || !std::isfinite(s.fx) || !std::isfinite(s.fy) ||
      !std::isfinite(s.ppx) || !std::isfinite(s.ppy) || s.fx<=0 || s.fy<=0 ||
      (s.model!=RS2_DISTORTION_NONE && s.model!=RS2_DISTORTION_BROWN_CONRADY &&
       s.model!=RS2_DISTORTION_INVERSE_BROWN_CONRADY))
    throw std::invalid_argument("unsupported or invalid source calibration");
  for(float c:s.coeffs) if(!std::isfinite(c)) throw std::invalid_argument("nonfinite distortion coefficient");
  map_x.create(s.height,s.width,CV_32F); map_y.create(s.height,s.width,CV_32F);
  for(int y=0;y<s.height;++y) for(int x=0;x<s.width;++x) {
    const float ray[3]={(x-s.ppx)/s.fx,(y-s.ppy)/s.fy,1};
    float pixel[2]; rs2_project_point_to_pixel(pixel,&s,ray);
    // SDK inverse-Brown projection/deprojection can differ slightly in tangential terms.
    // Refine the inverse map in pixel coordinates and verify against SDK deprojection.
    if(s.model!=RS2_DISTORTION_NONE) for(int iteration=0;iteration<5;++iteration) {
      float back[3]; rs2_deproject_pixel_to_point(back,&s,pixel,1);
      const float dx=(ray[0]-back[0])*s.fx, dy=(ray[1]-back[1])*s.fy;
      if(std::abs(dx)<0.001f && std::abs(dy)<0.001f) break;
      pixel[0]+=dx; pixel[1]+=dy;
    }
    map_x.at<float>(y,x)=pixel[0]; map_y.at<float>(y,x)=pixel[1];
  }
}
}
