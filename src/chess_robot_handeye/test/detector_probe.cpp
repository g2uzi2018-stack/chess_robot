// Test-only CLI: actual product detectBoard on a saved rectified frame.
// No ROS initialization, camera driver, USB access, robot or motion interfaces.
#include "chess_robot_handeye/core.hpp"
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace he = chess_robot_handeye;
int main(int argc, char **argv) {
  YAML::Node result;
  result["accepted"] = false;
  try {
    if (argc != 6 && argc != 10) {
      throw he::Error("usage: detector_probe IMAGE CAPTURE_JSON SQUARES_X "
                      "SQUARES_Y DICTIONARY_ID [MASK_X MASK_Y MASK_W MASK_H]");
    }
    const auto capture = YAML::LoadFile(argv[2]);
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
    if (image.empty() || capture["encoding"].as<std::string>() != "bgr8" ||
        capture["width"].as<int>() != image.cols ||
        capture["height"].as<int>() != image.rows) {
      throw he::Error("saved raw image/capture mismatch");
    }
    if (argc == 10) {
      const cv::Rect mask(std::stoi(argv[6]), std::stoi(argv[7]),
                          std::stoi(argv[8]), std::stoi(argv[9]));
      if ((mask & cv::Rect(0, 0, image.cols, image.rows)) != mask ||
          mask.empty()) {
        throw he::Error("mask outside image");
      }
      image(mask).setTo(cv::Scalar::all(255));
      for (int i = 6; i < 10; ++i)
        result["mask_xywh"].push_back(std::stoi(argv[i]));
    }
    cv::Matx33d K;
    if (capture["K"].size() != 9)
      throw he::Error("K size invalid");
    for (int i = 0; i < 9; ++i)
      K.val[i] = capture["K"][i].as<double>();
    for (const auto &d : capture["D"]) {
      if (std::abs(d.as<double>()) > 1e-12)
        throw he::Error("saved D must be zero");
    }
    he::Board spec;
    spec.squares_x = std::stoi(argv[3]);
    spec.squares_y = std::stoi(argv[4]);
    spec.dictionary = std::stoi(argv[5]);
    spec.square_m = .015;
    spec.marker_m = .01125;
    spec.legacy = true;
    spec.confirmed = true;
    auto board = he::makeBoard(spec);
    // Native counts are diagnostics only. Pass/fail invokes the product
    // function below, not a copy of its refinement or PnP implementation.
    std::vector<std::vector<cv::Point2f>> native_markers;
    std::vector<int> native_ids;
    auto params = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(image, board->dictionary, native_markers,
                             native_ids, params);
    result["native_marker_count"] = native_ids.size();
    result["adaptive_threshold_constant"] = params->adaptiveThreshConstant;
    result["minimum_corners"] = 12;
    result["maximum_reprojection_px"] = 1.0;
    result["opencv_version"] = CV_VERSION;
    if (!native_markers.empty()) {
      std::vector<cv::Point2f> native_corners;
      std::vector<int> native_corner_ids;
      try {
        cv::aruco::interpolateCornersCharuco(
            native_markers, native_ids, image, board, native_corners,
            native_corner_ids, K, cv::Mat::zeros(1, 5, CV_64F));
      } catch (const cv::Exception &e) {
        result["native_diagnostic_error"] = e.what();
      }
      result["native_charuco_corner_count"] = native_corner_ids.size();
    } else {
      result["native_charuco_corner_count"] = 0;
    }
    result["product_called"] = true;
    const auto detected = he::detectBoard(image, K, spec, 12, 1.0);
    result["accepted"] = true;
    result["product_charuco_corner_count"] = detected.ids.size();
    result["reprojection_rms_px"] = detected.reprojection_px;
    result["camera_T_board"] = he::matrixYaml(detected.camera_T_board);
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    out << result;
    std::cout << out.c_str() << '\n';
    return 0;
  } catch (const std::exception &e) {
    result["error"] = e.what();
    std::cout << result << '\n';
    return 2;
  }
}
