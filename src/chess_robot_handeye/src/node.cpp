#include "chess_robot_handeye/core.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chess_robot_interfaces/action/move_arm.hpp>
#include <chess_robot_interfaces/msg/arm_state.hpp>
#include <chess_robot_twist/model.hpp>
#include <chess_robot_vision/msg/frame_info.hpp>
#include <chrono>
#include <csignal>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

namespace he = chess_robot_handeye;
using Arm = chess_robot_interfaces::msg::ArmState;
using Joint = sensor_msgs::msg::JointState;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Frame = chess_robot_vision::msg::FrameInfo;
using Move = chess_robot_interfaces::action::MoveArm;
using Handle = rclcpp_action::ClientGoalHandle<Move>;
using Clock = std::chrono::steady_clock;
namespace {
std::atomic<bool> signal_abort{false};
void signalHandler(int) { signal_abort.store(true); }
int64_t ns(const builtin_interfaces::msg::Time &t) {
  return int64_t(t.sec) * 1000000000LL + t.nanosec;
}
double secs(int64_t n) { return n * 1e-9; }
std::string file(const std::string &p) {
  std::ifstream f(p, std::ios::binary);
  if (!f) {
    throw he::Error("cannot read " + p);
  }
  return {std::istreambuf_iterator<char>(f), {}};
}
void writeYaml(const std::filesystem::path &p, const YAML::Node &n) {
  std::ofstream f(p);
  f.precision(17);
  YAML::Emitter e;
  e.SetDoublePrecision(17);
  e << n;
  f << e.c_str() << "\n";
  if (!f) {
    throw he::Error("cannot write " + p.string());
  }
}
template <class Msg>
void saveRos(const std::filesystem::path &p, const Msg &m) {
  rclcpp::Serialization<Msg> codec;
  rclcpp::SerializedMessage serialized;
  codec.serialize_message(&m, &serialized);
  const auto &b = serialized.get_rcl_serialized_message();
  std::ofstream f(p, std::ios::binary);
  f.write(reinterpret_cast<const char *>(b.buffer), b.buffer_length);
  if (!f) {
    throw he::Error("cannot save original ROS message " + p.string());
  }
}
he::T eigenT(const Eigen::Isometry3d &t) {
  he::T r;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      r(i, j) = t(i, j);
    }
  }
  return r;
}
he::T poseT(const geometry_msgs::msg::Pose &p) {
  return he::poseMatrix(
      {p.position.x, p.position.y, p.position.z},
      {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w});
}
template <class Msg> YAML::Node headerYaml(const Msg &m) {
  YAML::Node n;
  n["stamp_ns"] = ns(m.header.stamp);
  n["frame_id"] = m.header.frame_id;
  return n;
}
} // namespace

class HandeyeNode final : public rclcpp::Node {
public:
  HandeyeNode() : Node("chess_robot_handeye") {
    path_ = declare_parameter<std::string>("config_file", "");
    if (path_.empty()) {
      throw he::Error("config_file parameter is required");
    }
    config_text_ = file(path_);
    c_ = he::loadConfig(path_);
    if (c_.mode == "execute") {
      he::validateConfig(c_, true);
    }
    if (!std::filesystem::create_directory(c_.output)) {
      throw he::Error("output directory must be NEW, with an existing parent");
    }
    std::ofstream(c_.output + "/config.input.yaml") << config_text_;
    auto resolved = YAML::Load(config_text_);
    resolved["output_directory"] = c_.output;
    resolved["model"]["urdf"] = c_.urdf;
    writeYaml(c_.output + "/config.yaml", resolved);
    report_["source_config_file"] = std::filesystem::absolute(path_).string();
    report_["resolved_urdf"] = c_.urdf;
    report_["resolved_output_directory"] = c_.output;
    report_["status"] = "starting";
    report_["mode"] = c_.mode;
    report_["fixture"] = c_.fixture;
    report_["source_fingerprint"] = HANDEYE_SOURCE_FINGERPRINT;
    report_["opencv_version"] = CV_VERSION;
    report_["synchronization"] = "stopped-pose host receipt timestamps; "
                                 "exposure synchronization NOT certified";
    report_["physical_accuracy_verified"] = false;
    report_["production_tf_published"] = false;
    report_["board_parameters_confirmed"] = c_.board.confirmed;
    report_["poses_confirmed"] = c_.poses_confirmed;
    if (c_.mode == "validate") {
      try {
        he::makeBoard(c_.board);
        report_["board_geometry_valid"] = true;
      } catch (const std::exception &e) {
        report_["board_geometry_valid"] = false;
        report_["board_error"] = e.what();
      }
      report_["status"] = "configuration_only_no_arm_or_vision_io";
      report_["execution_ready"] = false;
      persist();
      finished_ = true;
      return;
    }
    chess_robot_twist::Joints home;
    std::copy(c_.home.begin(), c_.home.end(), home.begin());
    Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
    tcp.translation() = Eigen::Vector3d(c_.tcp_p[0], c_.tcp_p[1], c_.tcp_p[2]);
    tcp.linear() = (Eigen::AngleAxisd(c_.tcp_rpy[2], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(c_.tcp_rpy[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(c_.tcp_rpy[0], Eigen::Vector3d::UnitX()))
                       .toRotationMatrix();
    model_ = std::make_unique<chess_robot_twist::Model>(c_.urdf, c_.base,
                                                        c_.flange, tcp, home);
    std::ofstream(c_.output + "/robot.urdf") << file(c_.urdf);
    auto sensor = rclcpp::SensorDataQoS();
    arm_sub_ = create_subscription<Arm>(
        c_.state, rclcpp::QoS(20), [this](Arm::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          auto t = ns(m->header.stamp);
          if (!arms_.empty() &&
              (t <= ns(arms_.back()->header.stamp) ||
               secs(t - ns(arms_.back()->header.stamp)) > c_.gap)) {
            ++bad_generation_;
          }
          if (!stateHealthy(*m, false)) {
            ++bad_generation_;
          }
          append(arms_, m);
        });
    joint_sub_ = create_subscription<Joint>(
        c_.joints, sensor, [this](Joint::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          try {
            auto q = joints(*m);
            if (!joints_.empty()) {
              auto &p = *joints_.back();
              double dt = secs(ns(m->header.stamp) - ns(p.header.stamp));
              if (dt <= 0 || dt > c_.gap) {
                ++bad_generation_;
              } else {
                auto old = joints(p);
                for (size_t i = 0; i < 6; ++i) {
                  if (std::abs(q[i] - old[i]) / dt > c_.speed) {
                    ++bad_generation_;
                  }
                }
              }
            }
          } catch (...) {
            ++bad_generation_;
          }
          append(joints_, m);
        });
    image_sub_ = create_subscription<Image>(
        c_.image, sensor, [this](Image::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          append(images_, m);
        });
    info_sub_ = create_subscription<Info>(
        c_.info, sensor, [this](Info::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          append(infos_, m);
        });
    frame_sub_ = create_subscription<Frame>(
        c_.frame, sensor, [this](Frame::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          append(frames_, m);
        });
    client_ = rclcpp_action::create_client<Move>(this, c_.action);
    parameters_ =
        std::make_shared<rclcpp::AsyncParametersClient>(this, c_.arm_node);
    abort_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/abort",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          abort_ = true;
          response->success = true;
          response->message = "abort requested; cancellation is not proof of "
                              "physical stop; see session report";
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      timer_->cancel();
      worker_ = std::thread([this] { run(); });
    });
  }
  ~HandeyeNode() override {
    abort_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }
  bool finished() const { return finished_; }
  bool passed() const { return passed_; }
  void abort() { abort_ = true; }

private:
  he::Config c_;
  std::string path_, config_text_;
  YAML::Node report_;
  std::vector<he::Sample> samples_;
  std::unique_ptr<chess_robot_twist::Model> model_;
  std::mutex mutex_;
  uint64_t bad_generation_ = 0;
  std::deque<Arm::ConstSharedPtr> arms_;
  std::deque<Joint::ConstSharedPtr> joints_;
  std::deque<Image::ConstSharedPtr> images_;
  std::deque<Info::ConstSharedPtr> infos_;
  std::deque<Frame::ConstSharedPtr> frames_;
  rclcpp::Subscription<Arm>::SharedPtr arm_sub_;
  rclcpp::Subscription<Joint>::SharedPtr joint_sub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_;
  rclcpp::Subscription<Frame>::SharedPtr frame_sub_;
  rclcpp_action::Client<Move>::SharedPtr client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread worker_;
  std::atomic<bool> abort_{false}, finished_{false}, passed_{false};
  Handle::SharedPtr owned_;
  template <class D, class M> void append(D &d, const M &m) {
    if (ns(m->header.stamp) <= 0) {
      return;
    }
    if (!d.empty() && ns(m->header.stamp) <= ns(d.back()->header.stamp)) {
      return;
    }
    d.push_back(m);
    while (d.size() > 1000 ||
           (!d.empty() &&
            ns(d.back()->header.stamp) - ns(d.front()->header.stamp) >
                10000000000LL)) {
      d.pop_front();
    }
  }
  void persist() { writeYaml(c_.output + "/session.yaml", report_); }
  void checkAbort() {
    if (abort_ || signal_abort || !rclcpp::ok()) {
      throw he::Error("operator/shutdown abort");
    }
  }
  void pause() { std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  template <class F> void wait(F &f, double timeout, bool abortable = true) {
    auto end = Clock::now() + std::chrono::duration<double>(timeout);
    while (f.wait_for(std::chrono::milliseconds(5)) !=
           std::future_status::ready) {
      if (abortable) {
        checkAbort();
      }
      if (Clock::now() > end) {
        throw he::Error("bounded ROS request timed out");
      }
    }
  }
  bool fresh(int64_t stamp) const {
    double age = secs(now().nanoseconds() - stamp);
    return stamp > 0 && age >= -0.01 && age <= c_.fresh;
  }
  bool stateHealthy(const Arm &s, bool check_fresh = true) const {
    return s.header.frame_id == c_.base && s.tcp.header.frame_id == c_.base &&
           ns(s.tcp.header.stamp) == ns(s.header.stamp) &&
           s.simulated == c_.fixture && s.feedback_fresh && s.reference_valid &&
           !s.busy && s.controller_mode == 5 &&
           (check_fresh ? fresh(ns(s.header.stamp)) : true);
  }
  chess_robot_twist::Joints joints(const Joint &j) const {
    chess_robot_twist::Joints q;
    auto names = model_->jointNames();
    if (j.name.size() != j.position.size() ||
        j.velocity.size() != j.name.size()) {
      throw he::Error("joint position/velocity/name size mismatch");
    }
    for (size_t i = 0; i < 6; ++i) {
      auto it = std::find(j.name.begin(), j.name.end(), names[i]);
      if (it == j.name.end() ||
          std::count(j.name.begin(), j.name.end(), names[i]) != 1) {
        throw he::Error("missing/duplicate model joint");
      }
      auto k = it - j.name.begin();
      q[i] = j.position[k];
      if (!std::isfinite(q[i]) || !std::isfinite(j.velocity[k]) ||
          std::abs(j.velocity[k]) > c_.speed) {
        throw he::Error("joint feedback is not stationary");
      }
    }
    if (!model_->withinJointBounds(q)) {
      throw he::Error("joint outside model bounds");
    }
    return q;
  }
  void checkParameters() {
    if (!parameters_->wait_for_service(std::chrono::seconds(5))) {
      throw he::Error("arm parameter service unavailable");
    }
    auto f = parameters_->get_parameters({"approach_clearance_m",
                                          "tcp.translation_m", "tcp.rpy_rad",
                                          "urdf_file"});
    wait(f, 5);
    auto p = f.get();
    if (p.size() != 4) {
      throw he::Error("arm parameter response incomplete");
    }
    if (std::abs(p[0].as_double() - c_.clearance) > 1e-9) {
      throw he::Error(
          "approach_clearance_m differs from configured expectation");
    }
    for (int i = 1; i <= 2; ++i) {
      auto v = p[i].as_double_array();
      auto e = i == 1 ? c_.tcp_p : c_.tcp_rpy;
      if (v.size() != 3) {
        throw he::Error("arm TCP parameter malformed");
      }
      for (int j = 0; j < 3; ++j) {
        if (std::abs(v[j] - e[j]) > 1e-9) {
          throw he::Error("arm TCP model differs from configured expectation");
        }
      }
    }
    if (file(p[3].as_string()) != file(c_.urdf)) {
      throw he::Error("arm URDF differs from handeye FK model");
    }
    report_["verified_arm_approach_clearance_m"] = p[0].as_double();
    report_["arm_urdf"] = p[3].as_string();
    persist();
  }
  void initialReady() {
    auto end = Clock::now() + std::chrono::duration<double>(c_.settle_timeout);
    while (Clock::now() < end) {
      checkAbort();
      {
        std::lock_guard<std::mutex> l(mutex_);
        if (!arms_.empty() && !joints_.empty() && stateHealthy(*arms_.back()) &&
            fresh(ns(joints_.back()->header.stamp))) {
          joints(*joints_.back());
          return;
        }
      }
      pause();
    }
    throw he::Error("fresh idle referenced arm/joint feedback unavailable");
  }
  void move(const he::Pose &p, YAML::Node &entry) {
    checkParameters();
    initialReady();
    if (c_.mode == "execute") {
      visionReady(entry);
    }
    checkAbort();
    Move::Goal goal;
    goal.operation = Move::Goal::PREGRASP;
    goal.profile = c_.profile;
    goal.plan_only = c_.mode == "plan";
    goal.target.header.frame_id = c_.base;
    goal.target.header.stamp = now();
    goal.target.pose.position.x = p.p[0];
    goal.target.pose.position.y = p.p[1];
    goal.target.pose.position.z = p.p[2] - c_.clearance;
    goal.target.pose.orientation.x = p.q[0];
    goal.target.pose.orientation.y = p.q[1];
    goal.target.pose.orientation.z = p.q[2];
    goal.target.pose.orientation.w = p.q[3];
    entry["requested_actual_tcp"] = he::matrixYaml(he::poseMatrix(p.p, p.q));
    entry["action_target_z_m"] = goal.target.pose.position.z;
    entry["plan_only"] = goal.plan_only;
    report_["points"].push_back(entry);
    persist();
    auto options = rclcpp_action::Client<Move>::SendGoalOptions();
    options.goal_response_callback = [this](Handle::SharedPtr h) {
      std::lock_guard<std::mutex> l(mutex_);
      owned_ = h;
      if (h && (abort_ || signal_abort)) {
        client_->async_cancel_goal(h);
      }
    };
    if (c_.mode == "execute" &&
        !fresh(entry["vision_preflight_stamp_ns"].as<int64_t>())) {
      throw he::Error("vision preflight expired before execution goal send");
    }
    auto accepted = client_->async_send_goal(goal, options);
    try {
      wait(accepted, 5);
    } catch (...) {
      abort_ = true;
      report_["goal_acceptance_unknown"] = true;
      // Keep spinning for a bounded late response so an accepted owned goal can
      // be canceled.
      try {
        wait(accepted, 3, false);
      } catch (...) {
      }
      throw he::Error("goal response timed out/aborted; late acceptance may be "
                      "unresolved; do not assume physical stop");
    }
    auto h = accepted.get();
    if (!h) {
      throw he::Error("arm action rejected goal; no automatic retry");
    }
    auto result = client_->async_get_result(h);
    wait(result, c_.action_timeout);
    auto wrapped = result.get();
    if (!wrapped.result) {
      throw he::Error("arm result absent");
    }
    auto &r = *wrapped.result;
    entry["action_code"] = r.code;
    entry["action_detail"] = r.detail;
    entry["execution_attempted"] = r.executed;
    report_["points"][report_["points"].size() - 1] = entry;
    persist();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      throw he::Error("arm action terminated without success");
    }
    if (goal.plan_only) {
      if (r.code != Move::Result::PLAN_READY || r.executed) {
        throw he::Error("invalid plan-only result");
      }
    } else {
      if (r.code != Move::Result::COMPLETED || !r.executed ||
          !r.final_tcp_valid || r.final_tcp.header.frame_id != c_.base ||
          !fresh(ns(r.final_tcp.header.stamp))) {
        throw he::Error("action did not confirm fresh real arrival");
      }
      auto target = he::poseMatrix(p.p, p.q);
      auto actual = poseT(r.final_tcp.pose);
      if (he::translationDistance(actual, target) > c_.position_tol ||
          he::rotationDistance(actual, target) > c_.rotation_tol) {
        throw he::Error("action arrival differs from configured actual TCP");
      }
      entry["action_final_tcp"] = he::matrixYaml(actual);
      entry["action_final_stamp_ns"] = ns(r.final_tcp.header.stamp);
    }
    {
      std::lock_guard<std::mutex> l(mutex_);
      owned_.reset();
    }
  }
  template <class D> typename D::value_type closest(const D &d, int64_t stamp) {
    typename D::value_type best;
    int64_t delta = INT64_MAX;
    for (auto &m : d) {
      int64_t dist = std::abs(ns(m->header.stamp) - stamp);
      if (dist < delta) {
        best = m;
        delta = dist;
      }
    }
    if (delta > int64_t(c_.pair * 1e9)) {
      return {};
    }
    return best;
  }
  template <class D> typename D::value_type exact(const D &d, int64_t stamp) {
    for (auto &m : d) {
      if (ns(m->header.stamp) == stamp && m->header.frame_id == c_.optical) {
        return m;
      }
    }
    return {};
  }
  cv::Matx33d validateVision(const Image &im, const Info &info,
                             const Frame &frame) const {
    const auto stamp = ns(im.header.stamp);
    if (!fresh(stamp) || ns(info.header.stamp) != stamp ||
        ns(frame.header.stamp) != stamp || im.header.frame_id != c_.optical ||
        info.header.frame_id != c_.optical ||
        frame.header.frame_id != c_.optical) {
      throw he::Error("vision headers are stale, mismatched or in wrong frame");
    }
    if (!frame.stream_ok || !std::isfinite(frame.device_timestamp_ms) ||
        !std::isfinite(frame.depth_scale_m) || frame.depth_scale_m <= 0) {
      throw he::Error("vision stream health invalid");
    }
    if (im.encoding != "bgr8" || info.width != im.width ||
        info.height != im.height || im.width == 0 || im.height == 0 ||
        im.step < uint64_t(im.width) * 3 ||
        im.data.size() < uint64_t(im.step) * im.height) {
      throw he::Error("image/CameraInfo geometry, encoding or data invalid");
    }
    for (double d : info.d) {
      if (!std::isfinite(d) || std::abs(d) > 1e-12) {
        throw he::Error("rectified images require zero distortion CameraInfo");
      }
    }
    cv::Matx33d K;
    for (int i = 0; i < 9; ++i) {
      K.val[i] = info.k[i];
      if (!std::isfinite(K.val[i])) {
        throw he::Error("CameraInfo K contains nonfinite values");
      }
    }
    if (K(0, 0) <= 0 || K(1, 1) <= 0 ||
        std::abs(K(2, 0)) + std::abs(K(2, 1)) + std::abs(K(2, 2) - 1) > 1e-12) {
      throw he::Error("CameraInfo pinhole matrix invalid");
    }
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(info.p[i * 4 + 3]) ||
          std::abs(info.p[i * 4 + 3]) > 1e-12) {
        throw he::Error(
            "rectified color CameraInfo P must have zero translation");
      }
      for (int k = 0; k < 3; ++k) {
        if (!std::isfinite(info.p[i * 4 + k]) ||
            std::abs(info.p[i * 4 + k] - K(i, k)) > 1e-8) {
          throw he::Error("CameraInfo K/P disagreement");
        }
      }
    }
    return K;
  }
  void visionReady(YAML::Node &entry) {
    const auto end =
        Clock::now() + std::chrono::duration<double>(c_.capture_timeout);
    std::string reason = "no fresh exactly matched image/CameraInfo/FrameInfo";
    while (Clock::now() < end) {
      checkAbort();
      Image::ConstSharedPtr image;
      Info::ConstSharedPtr info;
      Frame::ConstSharedPtr frame;
      {
        std::lock_guard<std::mutex> l(mutex_);
        for (auto it = images_.rbegin(); it != images_.rend(); ++it) {
          const auto stamp = ns((*it)->header.stamp);
          if (!fresh(stamp) || (*it)->header.frame_id != c_.optical) {
            continue;
          }
          auto ci = exact(infos_, stamp);
          auto fi = exact(frames_, stamp);
          if (ci && fi) {
            image = *it;
            info = ci;
            frame = fi;
            break;
          }
        }
      }
      if (image) {
        try {
          validateVision(*image, *info, *frame);
          entry["vision_preflight_stamp_ns"] = ns(image->header.stamp);
          return;
        } catch (const std::exception &e) {
          reason = e.what();
        }
      }
      pause();
    }
    throw he::Error("vision preflight failed before execution goal: " + reason);
  }
  void capture(const he::Pose &p, YAML::Node &entry) {
    auto dir = std::filesystem::path(c_.output) / p.id;
    if (!std::filesystem::create_directory(dir)) {
      throw he::Error("sample directory already exists");
    }
    std::ofstream proof(dir / "stationarity.csv");
    if (!proof) {
      throw he::Error("cannot open stationary proof");
    }
    proof.precision(17);
    proof << "check_ros_ns,arm_stamp_ns,joint_stamp_ns,healthy,generation,arm_"
             "age_s,joint_age_s\n";
    const int64_t after = now().nanoseconds();
    int64_t last = 0, cutoff = 0, stable_end = 0;
    uint64_t generation = 0;
    bool initialized = false, settled = false;
    he::StillWindow window(c_.dwell, c_.gap);
    auto end = Clock::now() + std::chrono::duration<double>(c_.settle_timeout);
    auto target = he::poseMatrix(p.p, p.q);
    while (true) {
      checkAbort();
      if (Clock::now() > end) {
        throw he::Error(settled ? "fresh matched ChArUco capture timed out"
                                : "continuous stationary dwell timed out");
      }
      Arm::ConstSharedPtr a, paired_arm;
      Joint::ConstSharedPtr j, paired_joint;
      Image::ConstSharedPtr im;
      Info::ConstSharedPtr info;
      Frame::ConstSharedPtr frame;
      {
        std::lock_guard<std::mutex> l(mutex_);
        if (arms_.empty() || joints_.empty()) {
          pause();
          continue;
        }
        a = arms_.back();
        j = joints_.back();
        if (!initialized) {
          generation = bad_generation_;
          initialized = true;
        }
        if (generation != bad_generation_) {
          if (settled) {
            throw he::Error(
                "feedback became invalid/moving after stationary dwell");
          }
          generation = bad_generation_;
          window.reset();
          last = 0;
        }
        const int64_t stamp =
            std::min(ns(a->header.stamp), ns(j->header.stamp));
        if (stamp > after && stamp > last) {
          last = stamp;
          bool healthy =
              stateHealthy(*a) && fresh(ns(j->header.stamp)) &&
              std::abs(secs(ns(a->header.stamp) - ns(j->header.stamp))) <=
                  c_.pair;
          try {
            auto q = joints(*j);
            auto tcp = eigenT(model_->tcp(q));
            healthy = healthy &&
                      he::translationDistance(tcp, target) <= c_.position_tol &&
                      he::rotationDistance(tcp, target) <= c_.rotation_tol &&
                      he::translationDistance(poseT(a->tcp.pose), tcp) < 1e-6 &&
                      he::rotationDistance(poseT(a->tcp.pose), tcp) < 1e-6;
          } catch (...) {
            healthy = false;
          }
          proof << now().nanoseconds() << ',' << ns(a->header.stamp) << ','
                << ns(j->header.stamp) << ',' << healthy << ',' << generation
                << ',' << secs(now().nanoseconds() - ns(a->header.stamp)) << ','
                << secs(now().nanoseconds() - ns(j->header.stamp)) << '\n';
          if (!proof) {
            throw he::Error("stationary proof write failed");
          }
          if (window.update(secs(stamp), healthy)) {
            stable_end = stamp;
            if (!settled) {
              settled = true;
              cutoff = stamp;
              end = Clock::now() +
                    std::chrono::duration<double>(c_.capture_timeout);
              entry["settled_stamp_ns"] = cutoff;
            }
          } else if (settled) {
            throw he::Error("stationary proof lost after dwell");
          }
        }
        if (settled && (!stateHealthy(*a) || !fresh(ns(j->header.stamp)))) {
          throw he::Error("feedback expired while waiting for image");
        }
        if (settled) {
          for (auto &candidate : images_) {
            auto t = ns(candidate->header.stamp);
            if (t <= cutoff || t > stable_end || !fresh(t) ||
                candidate->header.frame_id != c_.optical) {
              continue;
            }
            auto ci = exact(infos_, t);
            auto fi = exact(frames_, t);
            auto ar = closest(arms_, t);
            auto jq = closest(joints_, t);
            if (ci && fi && ar && jq) {
              im = candidate;
              info = ci;
              frame = fi;
              paired_arm = ar;
              paired_joint = jq;
              break;
            }
          }
        }
      }
      if (!im) {
        pause();
        continue;
      }
      if (!stateHealthy(*paired_arm) ||
          !fresh(ns(paired_joint->header.stamp))) {
        throw he::Error("paired image/feedback health invalid");
      }
      const auto K = validateVision(*im, *info, *frame);
      auto q = joints(*paired_joint);
      auto flange = eigenT(model_->flange(q));
      auto bgr = cv_bridge::toCvCopy(im, "bgr8")->image;
      // Preserve every candidate before detection. A bad capture ends this run;
      // no silent replacement.
      if (!cv::imwrite((dir / "color.png").string(), bgr)) {
        throw he::Error("image write failed");
      }
      saveRos(dir / "arm_state.cdr", *paired_arm);
      saveRos(dir / "joint_state.cdr", *paired_joint);
      saveRos(dir / "camera_info.cdr", *info);
      saveRos(dir / "frame_info.cdr", *frame);
      YAML::Node raw;
      raw["id"] = p.id;
      raw["role"] = p.role;
      raw["image"] = headerYaml(*im);
      raw["camera_info"] = headerYaml(*info);
      raw["frame_info"] = headerYaml(*frame);
      raw["frame_info"]["color_frame_number"] = frame->color_frame_number;
      raw["frame_info"]["depth_frame_number"] = frame->depth_frame_number;
      raw["frame_info"]["device_timestamp_ms"] = frame->device_timestamp_ms;
      raw["frame_info"]["timestamp_domain"] = frame->timestamp_domain;
      raw["frame_info"]["depth_scale_m"] = frame->depth_scale_m;
      raw["frame_info"]["stream_ok"] = frame->stream_ok;
      raw["frame_info"]["diagnostic"] = frame->diagnostic;
      raw["arm"] = headerYaml(*paired_arm);
      raw["arm"]["busy"] = paired_arm->busy;
      raw["arm"]["feedback_fresh"] = paired_arm->feedback_fresh;
      raw["arm"]["reference_valid"] = paired_arm->reference_valid;
      raw["arm"]["simulated"] = paired_arm->simulated;
      raw["arm"]["controller_mode"] = paired_arm->controller_mode;
      raw["arm"]["diagnostic"] = paired_arm->diagnostic;
      raw["arm"]["tcp"] = he::matrixYaml(poseT(paired_arm->tcp.pose));
      raw["joints"] = headerYaml(*paired_joint);
      raw["joints"]["names"] = paired_joint->name;
      raw["joints"]["positions_rad"] = paired_joint->position;
      raw["joints"]["velocities_rad_s"] = paired_joint->velocity;
      raw["base_T_flange"] = he::matrixYaml(flange);
      raw["units"] = "metres, radians";
      raw["settled_stamp_ns"] = cutoff;
      raw["stationary_proof_end_ns"] = stable_end;
      raw["image_width"] = im->width;
      raw["image_height"] = im->height;
      raw["distortion_model"] = info->distortion_model;
      for (auto x : info->k) {
        raw["K"].push_back(x);
      }
      for (auto x : info->p) {
        raw["P"].push_back(x);
      }
      raw["D"] = info->d;
      writeYaml(dir / "sample.yaml", raw);
      auto detection =
          he::detectBoard(bgr, K, c_.board, c_.min_corners, c_.reprojection_px);
      raw["camera_T_board"] = he::matrixYaml(detection.camera_T_board);
      raw["reprojection_rms_px"] = detection.reprojection_px;
      raw["charuco_ids"] = detection.ids;
      for (auto &corner : detection.corners) {
        YAML::Node v;
        v.push_back(corner.x);
        v.push_back(corner.y);
        raw["charuco_corners_px"].push_back(v);
      }
      writeYaml(dir / "sample.yaml", raw);
      samples_.push_back({flange, detection.camera_T_board, p.id, p.role});
      entry["capture"] = "accepted";
      entry["image_stamp_ns"] = ns(im->header.stamp);
      entry["reprojection_rms_px"] = detection.reprojection_px;
      entry["corner_count"] = detection.ids.size();
      return;
    }
  }
  void cancelOwned() {
    Handle::SharedPtr h;
    {
      std::lock_guard<std::mutex> l(mutex_);
      h = owned_;
    }
    if (!h) {
      report_["owned_goal_cancel_requested"] = false;
      return;
    }
    report_["owned_goal_cancel_requested"] = true;
    report_["physical_stop_confirmed"] = false;
    try {
      auto cancel = client_->async_cancel_goal(h);
      wait(cancel, 3, false);
      report_["cancel_response_code"] = cancel.get()->return_code;
    } catch (const std::exception &e) {
      report_["cancel_error"] = e.what();
    }
    // Cancellation ACK is not physical stop. Consume original feedback after
    // this observation boundary, rather than trusting reported zero velocity.
    const int64_t stop_started = now().nanoseconds();
    report_["stop_observation_started_ns"] = stop_started;
    report_["stop_proof_file"] = "stop_proof.csv";
    std::ofstream proof(c_.output + "/stop_proof.csv");
    if (!proof) {
      report_["stop_confirmation_failure"] = "cannot create stop proof";
      return;
    }
    proof.precision(17);
    proof << "check_ros_ns,arm_stamp_ns,joint_stamp_ns,generation,healthy,"
             "credited,max_position_speed_rad_s,reason";
    for (int i = 1; i <= 6; ++i)
      proof << ",q" << i;
    for (int i = 1; i <= 6; ++i)
      proof << ",v" << i;
    proof << '\n';
    auto row = [&](const Arm::ConstSharedPtr &a, const Joint::ConstSharedPtr &j,
                   uint64_t generation, bool healthy, bool credited,
                   double differential_speed, const std::string &reason) {
      proof << now().nanoseconds() << ',' << (a ? ns(a->header.stamp) : 0)
            << ',' << (j ? ns(j->header.stamp) : 0) << ',' << generation << ','
            << healthy << ',' << credited << ',' << differential_speed << ','
            << reason;
      const auto names = model_->jointNames();
      for (int kind = 0; kind < 2; ++kind) {
        for (const auto &name : names) {
          double value = std::numeric_limits<double>::quiet_NaN();
          if (j) {
            auto it = std::find(j->name.begin(), j->name.end(), name);
            if (it != j->name.end()) {
              const size_t index = it - j->name.begin();
              const auto &values = kind == 0 ? j->position : j->velocity;
              if (index < values.size())
                value = values[index];
            }
          }
          proof << ',' << value;
        }
      }
      proof << '\n';
    };
    he::StillWindow w(c_.dwell, c_.gap);
    int64_t cursor = stop_started, previous_stamp = 0, last_pair_stamp = 0;
    chess_robot_twist::Joints previous_q{};
    bool have_previous = false;
    uint64_t generation;
    {
      std::lock_guard<std::mutex> l(mutex_);
      generation = bad_generation_;
    }
    std::map<std::string, size_t> rejected;
    size_t consumed = 0;
    std::string last_reason = "no new paired feedback";
    auto end = Clock::now() + std::chrono::seconds(5);
    while (Clock::now() < end && rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> l(mutex_);
        if (generation != bad_generation_) {
          generation = bad_generation_;
          w.reset();
          have_previous = false;
          last_pair_stamp = 0;
          // Buffered samples predating an observed anomaly cannot rebuild a
          // supposedly post-anomaly continuous window.
          cursor = now().nanoseconds();
          last_reason = "callback_feedback_anomaly";
          ++rejected[last_reason];
          row(arms_.empty() ? nullptr : arms_.back(),
              joints_.empty() ? nullptr : joints_.back(), generation, false,
              false, std::numeric_limits<double>::quiet_NaN(), last_reason);
        }
        for (const auto &j : joints_) {
          const int64_t joint_stamp = ns(j->header.stamp);
          if (joint_stamp <= cursor)
            continue;
          auto a = closest(arms_, joint_stamp);
          if (!a && fresh(joint_stamp)) {
            // An arm message may still arrive for this joint receipt stamp.
            break;
          }
          cursor = joint_stamp;
          ++consumed;
          bool healthy = true;
          std::string reason = "healthy";
          double differential_speed = 0;
          if (!a || !stateHealthy(*a) || !fresh(joint_stamp) ||
              ns(a->header.stamp) < stop_started ||
              std::abs(secs(ns(a->header.stamp) - joint_stamp)) > c_.pair) {
            healthy = false;
            reason = "state_or_joint_stale_invalid_or_unpaired";
          }
          try {
            const auto q = joints(*j);
            if (!have_previous) {
              healthy = false;
              reason = "position_difference_baseline_required";
            } else {
              const double dt = secs(joint_stamp - previous_stamp);
              if (dt <= 0 || dt > c_.gap) {
                healthy = false;
                reason = "joint_timestamp_or_gap_invalid";
              } else {
                for (size_t i = 0; i < 6; ++i) {
                  differential_speed = std::max(
                      differential_speed, std::abs(q[i] - previous_q[i]) / dt);
                }
                if (differential_speed > c_.speed) {
                  healthy = false;
                  reason = "position_difference_exceeds_stationary_speed";
                }
              }
            }
            previous_q = q;
            previous_stamp = joint_stamp;
            have_previous = true;
          } catch (const std::exception &e) {
            healthy = false;
            have_previous = false;
            reason = "joint_velocity_or_structure_invalid";
            report_["stop_feedback_error"] = e.what();
          }
          const int64_t pair_stamp =
              a ? std::min(ns(a->header.stamp), joint_stamp) : joint_stamp;
          const bool credited = healthy && pair_stamp > last_pair_stamp;
          bool complete = false;
          if (!healthy) {
            w.reset();
            last_pair_stamp = 0;
            ++rejected[reason];
          } else if (credited) {
            last_pair_stamp = pair_stamp;
            complete = w.update(secs(pair_stamp), true);
          }
          last_reason = reason;
          row(a, j, generation, healthy, credited, differential_speed, reason);
          if (!proof) {
            report_["stop_confirmation_failure"] = "stop proof write failed";
            return;
          }
          if (complete) {
            proof.flush();
            if (!proof) {
              report_["stop_confirmation_failure"] = "stop proof flush failed";
              return;
            }
            report_["physical_stop_confirmed"] = true;
            report_["stop_confirmed_stamp_ns"] = pair_stamp;
            break;
          }
        }
        if (report_["physical_stop_confirmed"].as<bool>()) {
          break;
        }
        if (arms_.empty() || joints_.empty() ||
            !fresh(ns(arms_.back()->header.stamp)) ||
            !fresh(ns(joints_.back()->header.stamp))) {
          w.reset();
          have_previous = false;
          last_pair_stamp = 0;
          last_reason = "feedback_missing_or_expired";
        }
      }
      pause();
    }
    report_["stop_feedback_samples_consumed"] = consumed;
    for (const auto &item : rejected) {
      report_["stop_rejection_counts"][item.first] = item.second;
    }
    if (!report_["physical_stop_confirmed"].as<bool>()) {
      report_["stop_confirmation_failure"] =
          "continuous stationary window not established: " + last_reason;
    }
  }
  void run() {
    try {
      if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
        throw he::Error("arm action unavailable");
      }
      checkParameters();
      initialReady();
      report_["status"] = "running";
      persist();
      for (const auto &p : c_.poses) {
        YAML::Node entry;
        entry["id"] = p.id;
        entry["role"] = p.role;
        move(p, entry);
        if (c_.mode == "execute") {
          capture(p, entry);
        }
        entry["status"] = "completed";
        report_["points"][report_["points"].size() - 1] = entry;
        persist();
      }
      if (c_.mode == "plan") {
        report_["status"] =
            "individual_plans_only_not_sequence_or_visibility_certification";
        report_["accepted"] = false;
      } else {
        auto s = he::solve(samples_, c_);
        report_["flange_T_camera"] = he::matrixYaml(s.flange_T_camera);
        report_["transform_semantics"] =
            "p_flange = flange_T_camera * p_camera; base_T_board = "
            "base_T_flange * flange_T_camera * camera_T_board";
        report_["training_count"] = s.training;
        report_["validation_count"] = s.validation;
        report_["training_max_translation_m"] = s.train_max_m;
        report_["training_max_rotation_rad"] = s.train_max_rad;
        report_["validation_max_translation_m"] = s.validation_max_m;
        report_["validation_max_rotation_rad"] = s.validation_max_rad;
        report_["accepted"] = s.accepted;
        report_["status"] =
            s.accepted
                ? "held_out_consistency_passed_physical_accuracy_unverified"
                : "consistency_rejected";
        passed_ = s.accepted;
      }
      persist();
    } catch (const std::exception &e) {
      report_["status"] = "failed";
      report_["accepted"] = false;
      report_["failure"] = e.what();
      try {
        cancelOwned();
        persist();
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "failed to persist failure/stop report");
      }
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
    finished_ = true;
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  int rc = 1;
  try {
    auto node = std::make_shared<HandeyeNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      3);
    executor.add_node(node);
    while (rclcpp::ok() && !node->finished()) {
      if (signal_abort) {
        node->abort();
      }
      executor.spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    rc = node->passed() ? 0 : 2;
    executor.remove_node(node);
  } catch (const std::exception &e) {
    std::cerr << "handeye: " << e.what() << "\n";
  }
  rclcpp::shutdown();
  return rc;
}
