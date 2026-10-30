#include "chess_robot_handeye/core.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
namespace he = chess_robot_handeye;
namespace {
struct ConfigFile {
  std::filesystem::path directory = std::filesystem::temp_directory_path() /
      ("handeye_config_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  YAML::Node yaml = YAML::LoadFile(HANDEYE_TEST_CONFIG);
  ConfigFile() { std::filesystem::create_directory(directory); }
  ~ConfigFile() { std::filesystem::remove_all(directory); }
  std::string save() {
    const auto path = directory / "config.yaml";
    std::ofstream(path) << yaml;
    return path.string();
  }
};
}
TEST(ConfigPaths, RelativePathsUseYamlDirectory) {
  ConfigFile file;
  file.yaml["output_directory"] = "sessions/new";
  file.yaml["model"]["urdf"] = "models/robot.urdf";
  const auto c = he::loadConfig(file.save());
  EXPECT_EQ(c.output, (file.directory / "sessions/new").string());
  EXPECT_EQ(c.urdf, (file.directory / "models/robot.urdf").string());
}
TEST(ConfigPaths, AbsolutePathsRemainAbsolute) {
  ConfigFile file;
  file.yaml["output_directory"] = "/tmp/handeye_absolute_output";
  file.yaml["model"]["urdf"] = "/tmp/handeye_absolute.urdf";
  const auto c = he::loadConfig(file.save());
  EXPECT_EQ(c.output, "/tmp/handeye_absolute_output");
  EXPECT_EQ(c.urdf, "/tmp/handeye_absolute.urdf");
}
TEST(ConfigPaths, PackageResourceUsesAmentIndex) {
  ConfigFile file;
  file.yaml["model"]["urdf"] = "package://ament_index_cpp/package.xml";
  const auto c = he::loadConfig(file.save());
  EXPECT_EQ(c.urdf, ament_index_cpp::get_package_share_directory("ament_index_cpp") + "/package.xml");
}
TEST(ConfigPaths, RejectMalformedPackagePathsAndOutputUri) {
  ConfigFile file;
  for (const auto *uri : {"package://", "package://pkg", "package://pkg/",
                          "package://pkg/../escape", "package://pkg//absolute",
                          "package://nonexistent_handeye_test_package/robot.urdf"}) {
    file.yaml["model"]["urdf"] = uri;
    EXPECT_THROW(he::loadConfig(file.save()), he::Error) << uri;
  }
  file.yaml["model"]["urdf"] = "robot.urdf";
  file.yaml["output_directory"] = "package://ament_index_cpp/session";
  EXPECT_THROW(he::loadConfig(file.save()), he::Error);
}
he::T transform(double x, double y, double z, double rx, double ry, double rz) {
  cv::Mat R;
  cv::Rodrigues(cv::Vec3d(rx, ry, rz), R);
  he::T t = he::T::eye();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      t(i, j) = R.at<double>(i, j);
    }
  }
  t(0, 3) = x;
  t(1, 3) = y;
  t(2, 3) = z;
  return t;
}
std::vector<he::Sample> data() {
  auto X = transform(.04, -.03, .08, .2, -.1, .15);
  auto B = transform(.1, .2, .5, .1, .2, -.15);
  std::vector<he::Sample> s;
  for (int i = 0; i < 14; ++i) {
    auto G = transform(.1 * std::sin(i), .1 * std::cos(i), .2 + .01 * i,
                       .3 * std::sin(i * .8), .4 * std::cos(i * .6),
                       .25 * std::sin(i * .4));
    s.push_back({G, he::inverse(G * X) * B, std::to_string(i),
                 i < 10 ? "train" : "validation"});
  }
  return s;
}
TEST(Solver, KnownTruthAndHeldOut) {
  he::Config c;
  auto r = he::solve(data(), c);
  auto truth = transform(.04, -.03, .08, .2, -.1, .15);
  EXPECT_TRUE(r.accepted);
  EXPECT_LT(he::translationDistance(r.flange_T_camera, truth), 1e-6);
  EXPECT_LT(he::rotationDistance(r.flange_T_camera, truth), 1e-6);
  EXPECT_EQ(r.validation, 4u);
}
TEST(Solver, HeldOutErrorCannotBeFittedAway) {
  he::Config c;
  auto s = data();
  s.back().camera_T_board(0, 3) += .03;
  auto r = he::solve(s, c);
  EXPECT_FALSE(r.accepted);
  EXPECT_GT(r.validation_max_m, .02);
}
TEST(Solver, TranslationOnlyRejected) {
  he::Config c;
  auto s = data();
  for (auto &v : s) {
    v.base_T_flange = transform(v.base_T_flange(0, 3), 0, 0, 0, 0, 0);
  }
  EXPECT_THROW(he::solve(s, c), he::Error);
}
TEST(Solver, ParallelRotationAxesRejected) {
  he::Config c;
  auto s = data();
  for (size_t i = 0; i < s.size(); ++i) {
    s[i].base_T_flange = transform(0, 0, .2, 0, 0, .1 * i);
  }
  EXPECT_THROW(he::solve(s, c), he::Error);
}
TEST(Solver, MissingValidationRejected) {
  he::Config c;
  auto s = data();
  s.resize(10);
  EXPECT_THROW(he::solve(s, c), he::Error);
}
TEST(Solver, NonfiniteRejected) {
  he::Config c;
  auto s = data();
  s[0].base_T_flange(0, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(he::solve(s, c), he::Error);
}
TEST(Still, NeedsContinuousDwell) {
  he::StillWindow w(.4, .11);
  EXPECT_FALSE(w.update(1, true));
  EXPECT_FALSE(w.update(1.1, true));
  EXPECT_FALSE(w.update(1.2, true));
  EXPECT_FALSE(w.update(1.3, true));
  EXPECT_TRUE(w.update(1.41, true));
}
TEST(Still, DuplicateGapAndBadReset) {
  he::StillWindow w(.4, .11);
  w.update(1, true);
  w.update(1.1, true);
  EXPECT_FALSE(w.update(1.1, true));
  EXPECT_FALSE(w.update(1.5, true));
  EXPECT_FALSE(w.update(1.6, false));
  EXPECT_FALSE(w.update(1.7, true));
  EXPECT_FALSE(w.update(1.8, true));
}
TEST(Board, UnknownOrIncompatibleRejected) {
  he::Board b;
  EXPECT_THROW(he::makeBoard(b), he::Error);
  b.squares_x = 11;
  b.squares_y = 8;
  b.square_m = .015;
  b.marker_m = .01125;
  b.dictionary = cv::aruco::DICT_5X5_100;
  b.legacy = false;
  EXPECT_THROW(he::makeBoard(b), he::Error);
}
TEST(Board, DetectGeneratedBoard) {
  he::Board b;
  b.squares_x = 11;
  b.squares_y = 8;
  b.square_m = .015;
  b.marker_m = .01125;
  b.dictionary = cv::aruco::DICT_5X5_100;
  auto board = he::makeBoard(b);
  cv::Mat gray, image;
  board->draw(cv::Size(550, 400), gray, 20);
  cv::cvtColor(gray, image, cv::COLOR_GRAY2BGR);
  cv::Matx33d K(600, 0, 275, 0, 600, 200, 0, 0, 1);
  auto d = he::detectBoard(image, K, b, 12, 1.0);
  EXPECT_GE(d.ids.size(), 12u);
  EXPECT_TRUE(he::validTransform(d.camera_T_board));
}
TEST(Board, BlankImageRejected) {
  he::Board b;
  b.squares_x = 11;
  b.squares_y = 8;
  b.square_m = .015;
  b.marker_m = .01125;
  b.dictionary = 5;
  cv::Mat im(480, 640, CV_8UC3, cv::Scalar::all(255));
  EXPECT_THROW(he::detectBoard(im,
                               cv::Matx33d(600, 0, 320, 0, 600, 240, 0, 0, 1),
                               b, 12, 1),
               he::Error);
}
TEST(Pose, NonunitQuaternionRejected) {
  EXPECT_THROW(he::poseMatrix({0, 0, 0}, {0, 0, 0, 2}), he::Error);
}
TEST(Config, FixtureCannotUseRealTopics) {
  he::Config c;
  c.fixture = true;
  c.output = "/tmp/new";
  c.urdf = "/tmp/model";
  c.home = {0, 0, 0, 0, 0, 0};
  EXPECT_THROW(he::validateConfig(c, false), he::Error);
}
TEST(Config, ExecutionNeedsExplicitConfirmation) {
  he::Config c;
  c.output = "/tmp/new";
  c.urdf = "/tmp/model";
  c.home = {0, 0, 0, 0, 0, 0};
  EXPECT_THROW(he::validateConfig(c, true), he::Error);
}
