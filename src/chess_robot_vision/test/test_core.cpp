#include "chess_robot_vision/core.hpp"
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <limits>
using namespace chess_robot_vision;
TEST(Depth, ReadsSdkScaleAndRejectsMissing) {
  cv::Mat image(41,41,CV_16UC1,cv::Scalar(2000));
  auto e=estimate_depth(image,0.0001,{20,20},4,{});
  EXPECT_TRUE(e.valid); EXPECT_NEAR(e.z_m,0.2,1e-9); EXPECT_EQ(e.mad_m,0);
  image.setTo(0); e=estimate_depth(image,0.0001,{20,20},4,{});
  EXPECT_FALSE(e.valid); EXPECT_EQ(e.samples,0u);
}
TEST(Depth, RobustToMinorityOutliers) {
  cv::Mat image(41,41,CV_16UC1,cv::Scalar(2000));
  image.at<uint16_t>(20,20)=4000;
  auto e=estimate_depth(image,0.0001,{20,20},4,{});
  EXPECT_TRUE(e.valid); EXPECT_NEAR(e.z_m,0.2,1e-9);
}
TEST(Depth, RejectsRangeHolesAndBoundaryClamping) {
  cv::Mat image(41,41,CV_16UC1,cv::Scalar(9000));
  EXPECT_FALSE(estimate_depth(image,0.0001,{20,20},4,{}).valid);
  image.setTo(2000);
  EXPECT_FALSE(estimate_depth(image,0.0001,{0,0},4,{}).valid);
  EXPECT_FALSE(estimate_depth(image,0.0001,{-1,10},4,{}).valid);
  EXPECT_FALSE(estimate_depth(image,0.0001,{std::numeric_limits<float>::quiet_NaN(),10},4,{}).valid);
  image(cv::Rect(0,0,22,41)).setTo(0);
  EXPECT_FALSE(estimate_depth(image,0.0001,{20,20},4,{}).valid);
}
TEST(Depth, RejectsNoisySurface) {
  cv::Mat image(41,41,CV_16UC1);
  for (int y=0;y<41;++y) for (int x=0;x<41;++x) image.at<uint16_t>(y,x)=1700+(x%7)*100;
  EXPECT_FALSE(estimate_depth(image,0.0001,{20,20},6,{}).valid);
}
TEST(Depth, ValidatesConfiguration) {
  cv::Mat image(10,10,CV_16UC1,cv::Scalar(2000));
  EXPECT_THROW(estimate_depth(image,0,{5,5},2,{}),std::invalid_argument);
  DepthConfig bad; bad.min_valid_fraction=0;
  EXPECT_THROW(validate(bad),std::invalid_argument);
  bad={}; bad.max_mad_m=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate(bad),std::invalid_argument);
}
TEST(Detector, BlankImageAndKnownCircle) {
  cv::Mat image(240,320,CV_8UC3,cv::Scalar(230,230,230));
  DetectorConfig c; c.min_radius_px=20; c.max_radius_px=40; c.accumulator_threshold=25;
  EXPECT_TRUE(detect_circles(image,c).empty());
  cv::circle(image,{150,120},30,{30,30,30},3,cv::LINE_AA);
  auto circles=detect_circles(image,c);
  ASSERT_EQ(circles.size(),1u);
  EXPECT_NEAR(circles[0][0],150,3); EXPECT_NEAR(circles[0][1],120,3);
  EXPECT_NEAR(circles[0][2],30,4);
}
