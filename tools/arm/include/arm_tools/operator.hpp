#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace arm_tools {
using Joints = std::array<double, 6>;
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;
struct Feedback {
  Joints q{}, speed{};
  uint64_t mode = 0, controller_stamp = 0;
  Clock::time_point received{};
};
struct Network {
  std::string ip = "127.0.0.1";
  int dashboard_port = 29999, feedback_port = 30004, timeout_ms = 500;
};
struct Settings {
  Network controller;
  double deadline_s = 10, capture_s = 3, max_gap_s = .25;
  double stationary_rad_s = .01, max_span_rad = .003;
  size_t min_frames = 100;
  Joints lower{{-6.28, -3.14, -2.79, -6.28, -6.28, -6.28}};
  Joints upper{{6.28, 3.14, 2.79, 6.28, 6.28, 6.28}};
  int home_velocity_percent = 5, home_acceleration_percent = 5;
  double home_timeout_s = 120, home_joint_tolerance_rad = .015,
         arrival_dwell_s = .3, stop_timeout_s = 3;
  void validate() const;
};
class FeedbackReader {
  struct Impl;
  std::unique_ptr<Impl> impl_;

public:
  explicit FeedbackReader(const Network &);
  ~FeedbackReader();
  Feedback read();
};
struct Capture {
  Joints median{}, span{};
  size_t frames = 0;
};
using Reader = std::function<Feedback()>;
using Query = std::function<std::string(const std::string &)>;

// Only the operator CLI uses these administrative commands. The runtime motion
// node never acquires control, clears errors, or enables the robot
// automatically.
class Administrator {
  Settings settings_;
  Reader read_;
  Query query_;
  std::string call(const std::string &);
  uint64_t mode();
  void stationary();
  void waitMode(const std::function<bool(uint64_t)> &);

public:
  Administrator(Settings, Reader, Query);
  std::string prepare(bool request_tcp, bool apply);
};
Capture collect(const Settings &, Reader,
                const std::filesystem::path &directory,
                const std::function<bool()> &interrupted);
// Candidate YAML and JSON are always saved. Applying also preserves backups and
// replaces the canonical YAML+JSON, rolling back a failed second replacement.
void saveHome(const Capture &, const std::filesystem::path &directory,
              const std::filesystem::path &canonical, bool apply);
std::string returnHome(const Settings &, const Joints &, Reader, Query,
                       bool execute, const std::function<bool()> &interrupted);
} // namespace arm_tools
