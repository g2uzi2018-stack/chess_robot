#pragma once
#include <atomic>
#include <chess_robot_twist/model.hpp>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace chess_robot_arm {
using chess_robot_twist::Joints;
using chess_robot_twist::Transform;
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;
struct Failure : std::runtime_error {
  uint16_t code;
  Failure(uint16_t c, const std::string &s) : std::runtime_error(s), code(c) {}
};
enum Code : uint16_t {
  PLAN_READY = 0,
  COMPLETED = 1,
  INVALID_GOAL = 10,
  BUSY = 11,
  REFERENCE_INVALID = 12,
  FEEDBACK_FAULT = 13,
  PLANNING_FAILED = 14,
  HARDWARE_DISABLED = 15,
  EXECUTION_FAILED = 16,
  CANCELED = 17,
  GRIPPER_FAILED = 18,
  STOP_FAILED = 19
};
enum class Operation : uint8_t { Pregrasp, Descend, Retreat, Sequence, Home };
struct Profile {
  std::string name;
  double speed = 0.25, acceleration = 0.5;
};
struct ExplorationBoundary {
  std::string profile;
  double tcp_radius = .08, joint_radius = .3, twist_limit = pi / 6;
};
struct Config {
  std::string backend = "read_only", ip = "127.0.0.1", urdf;
  int dashboard_port = 29999, feedback_port = 30004, timeout_ms = 500,
      stop_ack_timeout_ms = 1500, enable_ack_timeout_ms = 5000;
  Joints home{{1.3873624514421463, 0.29600795831732624, 1.4554299194273725,
               -0.18064158423931584, -1.5707963267948966,
               -0.06370430298043014}};
  // Pinned official Nova5 ROS2 limits; device configuration must be verified before motion.
  std::string controller_firmware, controller_family = "unverified";
  bool motion_paused = true;
  std::string default_profile = "low";
  Joints velocity{{2.36, 2.36, 2.36, 2.36, 2.36, 2.36}};
  Joints acceleration{{4.72, 4.72, 4.72, 4.72, 4.72, 4.72}};
  Transform flange_tcp = Transform::Identity();
  Eigen::Vector3d workspace_min{-0.8, -0.8, 0.0}, workspace_max{0.8, 0.8, 1.2};
  Eigen::Vector3d grasp_axis{1, 0, 0};
  double clearance = 0.25, downward_tolerance = 0.15;
  bool power_cycle_handoff = false, enable_after_history_restore = false;
  // Only explicit near-Home live or named stopped power-cycle joins use this bound.
  double power_cycle_endpoint_tolerance = 0.001;
  std::string power_cycle_join_file; // Explicit recorded first audit of a known fault power epoch.
  std::string power_cycle_stopped_join_file; // One explicit recorded stopped power-only handoff.
  // One SHA-bound, final-audit fault suffix -> disabled restart at an exact record boundary.
  std::string power_cycle_fault_restart_file, power_cycle_fault_restart_sha256;
  int64_t power_cycle_fault_restart_offset_bytes = -1;
  bool feedback_trace = false; // Bounded in-memory timings; flush only after feedback thread stops.
  bool home_roundtrip_precheck = false; // Opt-in direct-TCP outward + predicted Home return validation.
  double stamp_age = 1.0, stamp_future = 0.05, freshness = 0.25;
  double home_tolerance = 0.03, stationary_speed = 0.01, dwell = 0.3;
  double position_tolerance = 0.003, rotation_tolerance = 0.03,
         joint_tolerance = 0.015;
  double joint_step = 0.025, cartesian_step = 0.002, sample_period = 0.03;
  double planning_timeout = 30.0, gripper_timeout = 15.0,
         arrival_timeout = 10.0, stop_timeout = 4.0;
  double servo_ahead = 50, servo_gain = 500, max_tracking_error = 0.15;
  double servo_ack_timeout = .15;
  double servo_command_time = 0; // 0 uses sample period; explicit horizon is independent of send cadence.
  size_t servo_max_pending = 6;
  size_t max_samples = 20000;
  double cost_duration = 1.0, cost_net = 1.0, cost_total = 1.0,
         cost_energy = 0.01;
  double neutral_band = 5 * pi / 180, twist_hysteresis = 2 * pi / 180,
         duration_hysteresis = 1.0;
  chess_robot_twist::Limits twist_limits;
  chess_robot_twist::TrackerOptions tracker_options;
  std::vector<Profile> profiles{{"low", .708, 1.416}, {"high", 2.36, 4.72},
                                {"conservative", .708, 1.416}};
  std::vector<ExplorationBoundary> exploration_boundaries;
  // Fixture feedback is explicitly labelled and published under
  // /simulation/arm.
  bool test_fixture = false;
  bool enable = false, tcp_verified = false, limits_verified = false,
       execution_verified = false, servo_certified = false;
  // Bounded commissioning uses the real planner/executor, without asserting
  // production TCP/ServoJ certification. It never reports certified=true.
  std::string safety_model_file;
  std::string audit_directory;
  double commissioning_joint_radius = 0.1, commissioning_tcp_radius = 0.03;
  void validate() const;
  bool hardwareAllowed() const;
  const Profile &profile(const std::string &) const;
  Config forProfile(const std::string &) const;
};
struct Feedback {
  Joints q{}, speed{};
  uint64_t mode = 0, controller_stamp = 0;
  Clock::time_point received{};
  int64_t ros_received_ns = 0; // ROS clock sampled at complete frame receipt, never controller time.
};
// SDK kinematics always refer to base/user=0, flange/tool=0. TCP conversion is
// local.
class Kinematics {
public:
  virtual ~Kinematics() = default;
  virtual Transform fk(const Joints &q) = 0;
  virtual Joints ik(const Transform &flange, const Joints &seed) = 0;
};
class Driver : public Kinematics {
public:
  virtual Feedback readFeedback() = 0;
  virtual void servo(const Joints &, double period) = 0;
  virtual void stop() = 0;
  virtual void beginStream() {}
  virtual void checkStream() {}
  virtual void endStream() {}
};
struct Snapshot {
  Feedback feedback;
  bool fresh = false, reference = false;
  chess_robot_twist::Tracker tracker;
  std::string diagnostic;
  Snapshot(chess_robot_twist::Tracker t) : tracker(std::move(t)) {}
};
class State {
  Config config_;
  std::shared_ptr<const chess_robot_twist::Model> model_;
  mutable std::mutex mutex_;
  chess_robot_twist::Tracker tracker_;
  Feedback last_;
  bool have_ = false;
  std::optional<Clock::time_point> stationary_since_;
  std::string diagnostic_;

public:
  State(const Config &, std::shared_ptr<const chess_robot_twist::Model>);
  void update(const Feedback &);
  void invalidate(const std::string &);
  Snapshot snapshot();
  void
  initialize(); // Caller holds task admission lock; no active task permitted.
  std::string restore(const std::vector<std::string> &audit_files, bool no_unobserved_motion);
};
struct Goal {
  Transform target = Transform::Identity();
  Operation operation = Operation::Pregrasp;
  std::string profile = "conservative";
  bool plan_only = true;
};
struct Sample {
  Joints q{};
  double time = 0;
};
struct Segment {
  std::string phase;
  std::vector<Sample> samples;
};
struct Plan {
  std::vector<Segment> segments;
  bool wait_for_gripper = false;
  std::string selection;
};
void validatePose(const Transform &, const Config &, bool downward = true);
void validateCapture(const std::string &frame, double stamp, double now,
                     const Config &);
double jointDistance(const Joints &, const Joints &);
double rotationDistance(const Transform &, const Transform &);
// Incumbent is the checked current-seed branch. Deadbands are anchored to it;
// this pure comparison cannot mutate live history, including during plan-only.
bool preferCandidate(const chess_robot_twist::Metrics &candidate, double duration,
                     const chess_robot_twist::Metrics &incumbent,
                     double incumbent_duration, const Config &);
class Planner {
  Config config_;
  std::shared_ptr<const chess_robot_twist::Model> model_;
  Kinematics &kinematics_;
  std::function<void()> checkpoint_;
  std::function<void(const Joints &)> safety_check_;
  Segment ptp(const std::string &, const Joints &, const Joints &,
              chess_robot_twist::Tracker &, const Profile &);
  Segment vertical(const std::string &, const Joints &, const Transform &,
                   chess_robot_twist::Tracker &, const Profile &);
  void check(const Joints &, chess_robot_twist::Tracker &, bool downward);
  void timePath(Segment &, const Profile &);
  Joints solve(const Transform &, const Joints &);

public:
  Planner(const Config &, std::shared_ptr<const chess_robot_twist::Model>,
          Kinematics &, std::function<void()>,
          std::function<void(const Joints &)> safety_check = {});
  Plan plan(const Goal &, const Snapshot &);
};
// Admission, confirmation and cancellation lifecycle, independent of ROS
// transport.
class TaskGate {
  mutable std::mutex mutex_;
  std::string job_, phase_ = "IDLE", detail_;
  bool busy_ = false, cancel_ = false, confirmation_used_ = false,
       fault_latched_ = false, terminal_committed_ = false;
  std::optional<bool> confirmation_;
  std::optional<Clock::time_point> confirmation_deadline_;

public:
  bool reserve(const std::string &);
  void release();
  void latchFault(const std::string &);
  bool cancel(const std::string &);
  // Linearizes cancellation against the one terminal transition.
  bool commitTerminal();
  bool canceled() const;
  void phase(const std::string &);
  void beginGripperWait(Clock::time_point deadline);
  std::optional<bool> finishGripperWait();
  bool confirm(const std::string &, bool, const std::string &);
  std::optional<bool> confirmation() const;
  struct View {
    bool busy;
    std::string job, phase, detail;
  };
  View view() const;
  bool initialize(State &, std::string &);
  bool restore(State &, const std::vector<std::string> &, bool, std::string &);
};
class Executor {
  Config config_;
  Driver &driver_;
  State &state_;
  TaskGate &gate_;
  std::shared_ptr<const chess_robot_twist::Model> model_;
  bool owned_ = false, attempted_ = false;
  std::function<void(const Joints &)> safety_check_;
  void checkpoint();
  void waitUntil(Clock::time_point);
  void arrival(const Joints &, double timeout);

public:
  Executor(const Config &, Driver &, State &, TaskGate &,
           std::shared_ptr<const chess_robot_twist::Model>,
           std::function<void(const Joints &)> safety_check = {});
  void run(const Plan &);
  void stopOwned();
  bool owned() const { return owned_; }
  bool attempted() const { return attempted_; }
};
} // namespace chess_robot_arm
