#include "chess_robot_vision/calibration.hpp"
#include <librealsense2/rsutil.h>
#include <gtest/gtest.h>
using namespace chess_robot_vision;
TEST(Calibration, PinholeIdentity) {
  rs2_intrinsics source{80,60,39,29,50,50,RS2_DISTORTION_NONE,{0,0,0,0,0}};
  cv::Mat x,y; rectification_maps(source,x,y);
  EXPECT_NEAR(x.at<float>(5,10),10,0.0001); EXPECT_NEAR(y.at<float>(5,10),5,0.0001);
}
TEST(Calibration, MeasuredD405InverseBrownRoundtrip) {
  rs2_intrinsics source{640,480,323.067f,246.950f,393.755f,393.575f,
    RS2_DISTORTION_INVERSE_BROWN_CONRADY,{-0.0504277f,0.0634278f,0.000625182f,0.000809469f,-0.0227374f}};
  cv::Mat mx,my; rectification_maps(source,mx,my);
  for(int y=0;y<480;y+=19) for(int x=0;x<640;x+=23) {
    const float pixel[2]={mx.at<float>(y,x),my.at<float>(y,x)};
    float ray[3]; rs2_deproject_pixel_to_point(ray,&source,pixel,1);
    EXPECT_NEAR(ray[0]*source.fx+source.ppx,x,0.01);
    EXPECT_NEAR(ray[1]*source.fy+source.ppy,y,0.01);
  }
  auto rect=pinhole_intrinsics(source);
  EXPECT_EQ(rect.model,RS2_DISTORTION_NONE);
  for(float c:rect.coeffs) EXPECT_EQ(c,0);
}
