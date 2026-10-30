// Synthetic-only integration peer. Every endpoint is under /simulation/.
#include "chess_robot_handeye/core.hpp"
#include <Eigen/Geometry>
#include <atomic>
#include <chess_robot_interfaces/action/move_arm.hpp>
#include <chess_robot_interfaces/msg/arm_state.hpp>
#include <chess_robot_twist/model.hpp>
#include <chess_robot_vision/msg/frame_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

namespace he = chess_robot_handeye;
using Move = chess_robot_interfaces::action::MoveArm;
using FixtureHandle = rclcpp_action::ServerGoalHandle<Move>;
using Arm = chess_robot_interfaces::msg::ArmState;
using Joint = sensor_msgs::msg::JointState;
using Frame = chess_robot_vision::msg::FrameInfo;
void atomicYaml(const std::string &path, const YAML::Node &node) {
  const std::string temporary = path + ".tmp";
  {
    std::ofstream out(temporary);
    out << node;
    out.flush();
    if (!out)
      throw std::runtime_error("fixture YAML write failed: " + path);
  }
  std::filesystem::rename(temporary, path);
}
he::T cvT(const Eigen::Isometry3d &t) {
  he::T v;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      v(i, j) = t(i, j);
    }
  }
  return v;
}
geometry_msgs::msg::Pose rosPose(const Eigen::Isometry3d &t) {
  geometry_msgs::msg::Pose p;
  p.position.x = t.translation().x();
  p.position.y = t.translation().y();
  p.position.z = t.translation().z();
  Eigen::Quaterniond q(t.linear());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}
class Fixture : public rclcpp::Node {
public:
  Fixture(const std::string &config, const std::string &mode)
      : Node("arm", "/simulation/handeye"), config_(config), mode_(mode) {
    c_ = he::loadConfig(config);
    chess_robot_twist::Joints home;
    std::copy(c_.home.begin(), c_.home.end(), home.begin());
    q_ = home;
    Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
    tcp.translation() = Eigen::Vector3d(c_.tcp_p[0], c_.tcp_p[1], c_.tcp_p[2]);
    tcp.linear() = (Eigen::AngleAxisd(c_.tcp_rpy[2], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(c_.tcp_rpy[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(c_.tcp_rpy[0], Eigen::Vector3d::UnitX()))
                       .toRotationMatrix();
    model_ = std::make_unique<chess_robot_twist::Model>(c_.urdf, c_.base,
                                                        c_.flange, tcp, home);
    declare_parameter("approach_clearance_m", c_.clearance);
    declare_parameter(
        "tcp.translation_m",
        std::vector<double>{c_.tcp_p[0], c_.tcp_p[1], c_.tcp_p[2]});
    declare_parameter(
        "tcp.rpy_rad",
        std::vector<double>{c_.tcp_rpy[0], c_.tcp_rpy[1], c_.tcp_rpy[2]});
    declare_parameter("urdf_file", c_.urdf);
    generateConfig(home);
    he::T C0 = he::T::eye();
    C0(2, 3) = .2;
    X_ = he::inverse(cvT(model_->flange(home))) * C0;
    B_ = C0;
    B_(0, 3) = -.0825;
    B_(1, 3) = -.06;
    B_(2, 3) += .45;
    auto board = he::makeBoard(c_.board);
    board->draw(cv::Size(1100, 800), board_image_, 0);
    state_pub_ = create_publisher<Arm>(c_.state, 10);
    joint_pub_ = create_publisher<Joint>(c_.joints, rclcpp::SensorDataQoS());
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        c_.image, rclcpp::SensorDataQoS());
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        c_.info, rclcpp::SensorDataQoS());
    frame_pub_ = create_publisher<Frame>(c_.frame, rclcpp::SensorDataQoS());
    server_ = rclcpp_action::create_server<Move>(
        this, c_.action,
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const Move::Goal> goal) {
          int index = requests_.fetch_add(1);
          bool valid = index < int(targets_.size()) &&
                       goal->operation == Move::Goal::PREGRASP &&
                       goal->target.header.frame_id == c_.base;
          if (valid) {
            auto p = goal->target.pose;
            p.position.z += c_.clearance;
            auto sent =
                he::poseMatrix({p.position.x, p.position.y, p.position.z},
                               {p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w});
            auto expected = cvT(model_->tcp(targets_[index]));
            valid = he::translationDistance(sent, expected) < 1e-8 &&
                    he::rotationDistance(sent, expected) < 1e-6;
          }
          if (!valid)
            ++bad_goals_;
          saveStats();
          return mode_ == "reject" || !valid
                     ? rclcpp_action::GoalResponse::REJECT
                     : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](auto) {
          ++cancels_;
          saveStats();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](auto h) {
          if (worker_.joinable()) {
            worker_.join();
          }
          worker_ = std::thread([this, h] { execute(h); });
        });
    timer_ =
        create_wall_timer(std::chrono::milliseconds(20), [this] { publish(); });
    saveStats();
    YAML::Node ready;
    ready["ready"] = true;
    ready["poses"] = targets_.size();
    atomicYaml(config_ + ".ready.yaml", ready);
  }
  ~Fixture() override {
    stop_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  he::Config c_;
  std::string config_, mode_;
  std::unique_ptr<chess_robot_twist::Model> model_;
  std::vector<chess_robot_twist::Joints> targets_;
  chess_robot_twist::Joints q_;
  he::T X_, B_;
  cv::Mat board_image_;
  std::mutex mutex_, stats_mutex_;
  bool busy_ = false;
  std::atomic<bool> stop_{false};
  std::atomic<int> requests_{0}, cancels_{0};
  std::atomic<int> bad_goals_{0};
  int frames_ = 0;
  builtin_interfaces::msg::Time previous_feedback_stamp_;
  rclcpp::Publisher<Arm>::SharedPtr state_pub_;
  rclcpp::Publisher<Joint>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<Frame>::SharedPtr frame_pub_;
  rclcpp_action::Server<Move>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread worker_;
  void saveStats() {
    std::lock_guard<std::mutex> l(stats_mutex_);
    YAML::Node n;
    n["goals_received"] = requests_.load();
    n["cancels_received"] = cancels_.load();
    n["invalid_goals"] = bad_goals_.load();
    atomicYaml(config_ + ".stats.yaml", n);
  }
  void generateConfig(const chess_robot_twist::Joints &home) {
    auto n = YAML::LoadFile(config_);
    n["poses"] = YAML::Node(YAML::NodeType::Sequence);
    for (int i = 0; i < 12; ++i) {
      auto q = home;
      const double offsets[12][6] = {
          {.10, 0, 0, 0, 0, 0},    {-.10, 0, 0, 0, 0, 0},
          {0, .08, 0, 0, 0, 0},    {0, -.08, 0, 0, 0, 0},
          {0, 0, .10, 0, 0, 0},    {0, 0, 0, .12, 0, 0},
          {0, 0, 0, 0, .10, 0},    {0, 0, 0, 0, 0, .15},
          {.06, .05, 0, 0, 0, 0},  {-.05, 0, .06, 0, 0, 0},
          {0, 0, 0, -.08, .06, 0}, {.04, -.04, 0, 0, 0, .08}};
      for (int j = 0; j < 6; ++j) {
        q[j] += offsets[i][j];
      }
      targets_.push_back(q);
      auto tcp = model_->tcp(q);
      auto p = rosPose(tcp);
      YAML::Node pose;
      pose["id"] = "fixture_" + std::to_string(i);
      pose["role"] = i < 8 ? "train" : "validation";
      pose["position_m"].push_back(p.position.x);
      pose["position_m"].push_back(p.position.y);
      pose["position_m"].push_back(p.position.z);
      for (double x : {p.orientation.x, p.orientation.y, p.orientation.z,
                       p.orientation.w}) {
        pose["quaternion_xyzw"].push_back(x);
      }
      n["poses"].push_back(pose);
    }
    atomicYaml(config_, n);
    c_ = he::loadConfig(config_);
  }
  void execute(std::shared_ptr<FixtureHandle> h) {
    auto goal = h->get_goal();
    Move::Result r;
    if (goal->plan_only) {
      r.code = Move::Result::PLAN_READY;
      r.executed = false;
      h->succeed(std::make_shared<Move::Result>(r));
      return;
    }
    {
      std::lock_guard<std::mutex> l(mutex_);
      busy_ = true;
    }
    int index = requests_.load() - 1;
    for (int i = 0; i < (mode_.rfind("cancel", 0) == 0 ? 500 : 15) && !stop_;
         ++i) {
      if (h->is_canceling()) {
        std::lock_guard<std::mutex> l(mutex_);
        busy_ = false;
        r.code = Move::Result::CANCELED;
        r.executed = true;
        h->canceled(std::make_shared<Move::Result>(r));
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    {
      std::lock_guard<std::mutex> l(mutex_);
      if (mode_ != "no_motion") {
        q_ = targets_.at(index);
      }
      busy_ = false;
      r.final_tcp.pose = rosPose(model_->tcp(q_));
    }
    r.code = Move::Result::COMPLETED;
    r.executed = true;
    r.final_tcp_valid = true;
    r.final_tcp.header.frame_id = c_.base;
    r.final_tcp.header.stamp = now();
    h->succeed(std::make_shared<Move::Result>(r));
  }
  void publish() {
    chess_robot_twist::Joints q;
    bool busy;
    {
      std::lock_guard<std::mutex> l(mutex_);
      if (cancels_.load() > 0 && mode_ == "cancel_moving_zero_velocity") {
        q_[0] += 0.005;
        std::ofstream drift(config_ + ".drift.csv", std::ios::app);
        drift.precision(17);
        drift << now().nanoseconds() << ',' << q_[0] << '\n';
      }
      q = q_;
      busy = busy_;
    }
    auto stamp = now();
    auto tcp = model_->tcp(q);
    Arm a;
    a.header.stamp = stamp;
    a.header.frame_id = c_.base;
    a.tcp.header = a.header;
    a.tcp.pose = rosPose(tcp);
    a.backend = "synthetic_fixture";
    a.busy = busy;
    a.feedback_fresh = true;
    a.reference_valid = true;
    a.simulated = true;
    a.controller_mode = busy ? 7 : 5;
    Joint j;
    j.header = a.header;
    auto names = model_->jointNames();
    j.name.assign(names.begin(), names.end());
    j.position.assign(q.begin(), q.end());
    j.velocity.assign(6, busy ? .01 : 0.0);
    if (cancels_.load() > 0) {
      if (mode_ == "cancel_duplicate_stamp" && frames_ % 2 == 0) {
        a.header.stamp = previous_feedback_stamp_;
        a.tcp.header = a.header;
        j.header = a.header;
      }
      if (mode_ == "cancel_backward_stamp" && frames_ % 2 == 0) {
        a.header.stamp = rclcpp::Time(stamp.nanoseconds() - 80000000LL);
        a.tcp.header = a.header;
        j.header = a.header;
      }
      if (mode_ == "cancel_invalid_feedback" && frames_ % 2 == 0) {
        a.reference_valid = false;
      }
      if (mode_ == "cancel_unpaired_feedback") {
        j.header.stamp = rclcpp::Time(stamp.nanoseconds() - 10000000LL);
      }
    }
    previous_feedback_stamp_ = stamp;
    if (mode_ == "stale") {
      a.header.stamp = rclcpp::Time(stamp.nanoseconds() - 1000000000LL);
      a.tcp.header = a.header;
      j.header = a.header;
    }
    state_pub_->publish(a);
    joint_pub_->publish(j);
    if (mode_ == "no_vision" || mode_ == "plan")
      return;
    if ((frames_++ % 3) != 0) {
      return;
    }
    auto camera_T_board = he::inverse(cvT(model_->flange(q)) * X_) * B_;
    cv::Mat R(3, 3, CV_64F), t(3, 1, CV_64F);
    for (int x = 0; x < 3; ++x) {
      t.at<double>(x) = camera_T_board(x, 3);
      for (int y = 0; y < 3; ++y) {
        R.at<double>(x, y) = camera_T_board(x, y);
      }
    }
    cv::Mat rv;
    cv::Rodrigues(R, rv);
    cv::Matx33d K(600, 0, 320, 0, 600, 240, 0, 0, 1);
    std::vector<cv::Point3f> p{
        {0, 0, 0}, {.165f, 0, 0}, {.165f, .12f, 0}, {0, .12f, 0}};
    std::vector<cv::Point2f> img;
    cv::projectPoints(p, rv, t, K, cv::Mat(), img);
    std::vector<cv::Point2f> src{{0, 0}, {1099, 0}, {1099, 799}, {0, 799}};
    auto H = cv::getPerspectiveTransform(src, img);
    cv::Mat gray, bgr;
    cv::warpPerspective(board_image_, gray, H, cv::Size(640, 480),
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = c_.optical;
    auto im = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
    sensor_msgs::msg::CameraInfo ci;
    ci.header = header;
    ci.width = 640;
    ci.height = 480;
    ci.distortion_model = "plumb_bob";
    ci.d.assign(5, 0);
    for (int x = 0; x < 9; ++x) {
      ci.k[x] = K.val[x];
    }
    for (int x = 0; x < 3; ++x) {
      for (int y = 0; y < 3; ++y) {
        ci.p[x * 4 + y] = K(x, y);
      }
    }
    Frame f;
    f.header = header;
    f.color_frame_number = frames_;
    f.depth_frame_number = frames_;
    f.device_timestamp_ms = stamp.seconds() * 1000;
    f.timestamp_domain = "synthetic";
    f.depth_scale_m = .0001;
    f.stream_ok = mode_ != "camera_unhealthy";
    if (mode_ == "mismatch") {
      ci.header.stamp = rclcpp::Time(stamp.nanoseconds() - 1000000);
    }
    image_pub_->publish(*im);
    info_pub_->publish(ci);
    frame_pub_->publish(f);
  }
};
int main(int argc, char **argv) {
  if (argc < 3) {
    return 1;
  }
  auto config = std::string(argv[1]), mode = std::string(argv[2]);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Fixture>(config, mode);
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
