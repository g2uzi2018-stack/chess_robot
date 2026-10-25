#include "chess_robot_arm/core.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <cmath>
#include <limits>
#include <thread>

namespace chess_robot_arm {
namespace {
void require(bool value, const std::string &message) {
  if (!value)
    throw Failure(INVALID_GOAL, message);
}
bool positive(double x) { return std::isfinite(x) && x > 0; }
size_t subdivisions(double ratio, size_t maximum) {
  if (!std::isfinite(ratio) || ratio > double(maximum - 1))
    throw Failure(PLANNING_FAILED, "sample budget exceeded");
  return std::max<size_t>(1, static_cast<size_t>(std::ceil(ratio)));
}
double maxSpeed(const Feedback &f) {
  double s = 0;
  for (double v : f.speed)
    s = std::max(s, std::abs(v));
  return s;
}
} // namespace
void Config::validate() const {
  require(backend == "read_only" || backend == "hardware" ||
              backend == "commissioning",
          "backend must be read_only, hardware or commissioning");
  require(!urdf.empty(), "urdf_file is required");
  in_addr address{};
  require(inet_pton(AF_INET, ip.c_str(), &address) == 1,
          "controller.ip must be an IPv4 literal");
  require(!test_fixture || ip == "127.0.0.1",
          "test_fixture requires IPv4 loopback");
  require(dashboard_port > 0 && dashboard_port < 65536 && feedback_port > 0 &&
              feedback_port < 65536 && timeout_ms > 0 && timeout_ms <= 5000 &&
              stop_ack_timeout_ms > 0 && stop_ack_timeout_ms <= 5000,
          "invalid network bounds");
  for (size_t i = 0; i < 6; ++i)
    require(std::isfinite(home[i]) && positive(velocity[i]) &&
                positive(acceleration[i]),
            "invalid Home or positive software caps");
  require(flange_tcp.matrix().allFinite() &&
              std::abs(flange_tcp.linear().determinant() - 1) < 1e-6 &&
              (flange_tcp.linear().transpose() * flange_tcp.linear() -
               Eigen::Matrix3d::Identity())
                      .norm() < 1e-6,
          "invalid TCP transform");
  require(workspace_min.allFinite() && workspace_max.allFinite() &&
              (workspace_min.array() < workspace_max.array()).all(),
          "invalid workspace");
  require(grasp_axis.allFinite() && std::abs(grasp_axis.norm() - 1) < 1e-6,
          "grasp axis must be unit length");
  for (double x : {clearance,
                   downward_tolerance,
                   stamp_age,
                   stamp_future,
                   freshness,
                   home_tolerance,
                   stationary_speed,
                   dwell,
                   position_tolerance,
                   rotation_tolerance,
                   joint_tolerance,
                   joint_step,
                   cartesian_step,
                   sample_period,
                   planning_timeout,
                   gripper_timeout,
                   arrival_timeout,
                   stop_timeout,
                   max_tracking_error,
                   tracker_options.max_joint_step_rad,
                   tracker_options.max_phase_step_rad,
                   twist_limits.net_limit_rad,
                   twist_limits.total_limit_rad})
    require(positive(x), "nonpositive/nonfinite configuration");
  require(home_tolerance <= tracker_options.max_joint_step_rad &&
              tracker_options.max_joint_step_rad < pi &&
              tracker_options.max_phase_step_rad < pi &&
              downward_tolerance < pi / 2 &&
              joint_step <= tracker_options.max_joint_step_rad &&
              sample_period >= 0.004 && sample_period <= 0.1 &&
              max_samples >= 2 && max_samples <= 100000 && servo_ahead >= 20 &&
              servo_ahead <= 100 && servo_gain >= 200 && servo_gain <= 1000 &&
              stop_timeout >
                  (2 * double(timeout_ms) + stop_ack_timeout_ms) / 1000 + dwell,
          "sampling or ServoJ configuration out of bounds");
  for (double weight : {cost_duration, cost_net, cost_total, cost_energy})
    require(positive(weight), "invalid candidate cost weight");
  require(!profiles.empty() && profiles.size() <= 32, "invalid profile count");
  for (size_t i = 0; i < profiles.size(); ++i) {
    require(!profiles[i].name.empty() && profiles[i].name.size() <= 64 &&
                positive(profiles[i].speed) &&
                positive(profiles[i].acceleration),
            "invalid profile");
    for (size_t j = 0; j < i; ++j)
      require(profiles[i].name != profiles[j].name, "duplicate profile");
  }
  if (backend == "commissioning") {
    require(!test_fixture && !safety_model_file.empty(),
            "commissioning requires real geometry guard");
    require(positive(commissioning_joint_radius) &&
                commissioning_joint_radius <= .12 &&
                positive(commissioning_tcp_radius) &&
                commissioning_tcp_radius <= .04 && freshness <= .1 &&
                max_tracking_error <= .01 &&
                twist_limits.net_limit_rad <= pi / 9 &&
                twist_limits.total_limit_rad <= pi / 9,
            "commissioning envelope/feedback/twist limits too wide");
    for (size_t j = 0; j < 6; ++j)
      require(velocity[j] <= .02 && acceleration[j] <= .05,
              "commissioning joint caps too high");
    for (const auto &p : profiles)
      require(p.speed <= .02 && p.acceleration <= .05,
              "commissioning profile too fast");
  }
}
bool Config::hardwareAllowed() const {
  if (backend == "commissioning")
    return enable && !test_fixture && !safety_model_file.empty();
  return backend == "hardware" && enable && tcp_verified && limits_verified &&
         execution_verified && servo_certified;
}
const Profile &Config::profile(const std::string &n) const {
  for (const auto &p : profiles)
    if (p.name == n)
      return p;
  throw Failure(INVALID_GOAL, "unknown motion profile");
}
double jointDistance(const Joints &a, const Joints &b) {
  double d = 0;
  for (size_t i = 0; i < 6; ++i)
    d = std::max(d, std::abs(a[i] - b[i]));
  return d;
}
double rotationDistance(const Transform &a, const Transform &b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}
void validatePose(const Transform &t, const Config &c, bool downward) {
  require(t.matrix().allFinite(), "nonfinite pose");
  require(
      std::abs(t.linear().determinant() - 1) < 1e-6 &&
          (t.linear().transpose() * t.linear() - Eigen::Matrix3d::Identity())
                  .norm() < 1e-6,
      "invalid rotation");
  require((t.translation().array() >= c.workspace_min.array()).all() &&
              (t.translation().array() <= c.workspace_max.array()).all(),
          "TCP outside configured workspace");
  if (downward)
    require((t.linear() * c.grasp_axis).dot(-Eigen::Vector3d::UnitZ()) >=
                std::cos(c.downward_tolerance),
            "grasp axis is not downward");
}
void validateCapture(const std::string &f, double stamp, double now,
                     const Config &c) {
  require(f == "base_link", "target frame must be base_link");
  require(std::isfinite(stamp) && std::isfinite(now) && stamp > 0 &&
              now - stamp <= c.stamp_age && stamp - now <= c.stamp_future,
          "stale, zero or future capture stamp");
}
State::State(const Config &c, std::shared_ptr<const chess_robot_twist::Model> m)
    : config_(c), model_(m), tracker_(m, c.tracker_options) {}
void State::update(const Feedback &f) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (f.mode != 4 && f.mode != 5 && f.mode != 7)
      throw Failure(FEEDBACK_FAULT, "controller not in supported healthy mode");
    if (!model_->withinJointBounds(f.q))
      throw Failure(FEEDBACK_FAULT, "feedback joint bounds");
    for (double v : f.speed)
      if (!std::isfinite(v))
        throw Failure(FEEDBACK_FAULT, "nonfinite feedback speed");
    if (have_ && f.received - last_.received >
                     std::chrono::duration<double>(config_.freshness)) {
      tracker_.invalidate();
      stationary_since_.reset();
      diagnostic_ = "feedback gap; explicit reference required";
    }
    if (have_ && f.controller_stamp <= last_.controller_stamp)
      throw Failure(FEEDBACK_FAULT, "feedback timestamp regression");
    if (tracker_.valid())
      tracker_.advance(f.q);
    if (maxSpeed(f) <= config_.stationary_speed) {
      if (!stationary_since_)
        stationary_since_ = f.received;
    } else
      stationary_since_.reset();
    last_ = f;
    have_ = true;
  } catch (...) {
    tracker_.invalidate();
    stationary_since_.reset();
    diagnostic_ = "invalid/discontinuous feedback";
    throw;
  }
}
void State::invalidate(const std::string &why) {
  std::lock_guard<std::mutex> lock(mutex_);
  tracker_.invalidate();
  stationary_since_.reset();
  diagnostic_ = why;
}
Snapshot State::snapshot() {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool fresh =
      have_ && Clock::now() - last_.received <=
                   std::chrono::duration<double>(config_.freshness);
  if (!fresh) {
    tracker_.invalidate();
    stationary_since_.reset();
    diagnostic_ = "missing/stale feedback";
  }
  Snapshot s(tracker_);
  s.feedback = last_;
  s.fresh = fresh;
  s.reference = tracker_.valid();
  s.diagnostic = diagnostic_;
  return s;
}
void State::initialize() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_ ||
      Clock::now() - last_.received >
          std::chrono::duration<double>(config_.freshness) ||
      !stationary_since_ ||
      Clock::now() - *stationary_since_ <
          std::chrono::duration<double>(config_.dwell))
    throw Failure(FEEDBACK_FAULT, "reference requires fresh stationary dwell");
  tracker_.initializeAtHome(last_.q, config_.home_tolerance);
  diagnostic_ = "explicit Home reference initialized";
}
Planner::Planner(const Config &c,
                 std::shared_ptr<const chess_robot_twist::Model> m,
                 Kinematics &k, std::function<void()> cb,
                 std::function<void(const Joints &)> safety_check)
    : config_(c), model_(m), kinematics_(k), checkpoint_(std::move(cb)),
      safety_check_(std::move(safety_check)) {}
void Planner::check(const Joints &q, chess_robot_twist::Tracker &tracker,
                    bool downward) {
  checkpoint_();
  if (!model_->withinJointBounds(q))
    throw Failure(PLANNING_FAILED, "joint bound violation");
  validatePose(model_->tcp(q), config_, downward);
  tracker.advance(q);
  if (!chess_robot_twist::withinLimits(tracker.metrics(), config_.twist_limits))
    throw Failure(PLANNING_FAILED, "geometric twist limit");
  if (safety_check_)
    safety_check_(q);
}
Joints Planner::solve(const Transform &tcp, const Joints &seed) {
  checkpoint_();
  try {
    const auto flange = tcp * config_.flange_tcp.inverse();
    auto q = kinematics_.ik(flange, seed);
    if (!model_->withinJointBounds(q))
      throw Failure(PLANNING_FAILED, "IK outside joint bounds");
    auto sdk = kinematics_.fk(q) * config_.flange_tcp;
    auto local = model_->tcp(q);
    for (const auto &actual : {sdk, local})
      if ((actual.translation() - tcp.translation()).norm() >
              config_.position_tolerance ||
          rotationDistance(actual, tcp) > config_.rotation_tolerance)
        throw Failure(PLANNING_FAILED, "FK(IK) or SDK/model disagreement");
    return q;
  } catch (const Failure &e) {
    if (e.code == CANCELED || e.code == FEEDBACK_FAULT ||
        e.code == PLANNING_FAILED)
      throw;
    throw Failure(PLANNING_FAILED,
                  std::string("kinematics query: ") + e.what());
  }
}
void Planner::timePath(Segment &s, const Profile &p) {
  // Global conservative dilation checks finite-difference velocity and
  // acceleration, including zero velocity before/after the segment. No URDF
  // zero caps are used.
  double dt = config_.sample_period, factor = 1;
  Joints previous_velocity{};
  for (size_t i = 1; i < s.samples.size(); ++i) {
    for (size_t j = 0; j < 6; ++j) {
      double v = (s.samples[i].q[j] - s.samples[i - 1].q[j]) / dt;
      factor = std::max(factor,
                        std::abs(v) / std::min(p.speed, config_.velocity[j]));
      factor = std::max(factor,
                        std::abs(v - previous_velocity[j]) / dt /
                            std::min(p.acceleration, config_.acceleration[j]));
      previous_velocity[j] = v;
    }
  }
  for (size_t j = 0; j < 6; ++j)
    factor =
        std::max(factor, std::abs(previous_velocity[j]) / dt /
                             std::min(p.acceleration, config_.acceleration[j]));
  // Integer subdivision retains original vertices and gives fixed ServoJ
  // periods.
  size_t sub = subdivisions(factor, config_.max_samples);
  if (sub > config_.max_samples || s.samples.size() > config_.max_samples / sub)
    throw Failure(PLANNING_FAILED, "timed sample budget exceeded");
  std::vector<Sample> out;
  out.reserve((s.samples.size() - 1) * sub + 1);
  out.push_back(s.samples.front());
  for (size_t i = 1; i < s.samples.size(); ++i)
    for (size_t k = 1; k <= sub; ++k) {
      Sample x;
      for (size_t j = 0; j < 6; ++j)
        x.q[j] = s.samples[i - 1].q[j] +
                 (s.samples[i].q[j] - s.samples[i - 1].q[j]) * double(k) / sub;
      x.time = out.size() * dt;
      out.push_back(x);
    }
  s.samples = std::move(out);
}
Segment Planner::ptp(const std::string &phase, const Joints &a, const Joints &b,
                     chess_robot_twist::Tracker &t, const Profile &p) {
  Segment s{phase, {}};
  size_t n = subdivisions(jointDistance(a, b) / config_.joint_step,
                          config_.max_samples);
  if (n >= config_.max_samples)
    throw Failure(PLANNING_FAILED, "PTP sample budget");
  for (size_t i = 0; i <= n; ++i) {
    double u = double(i) / n;
    double blend = u * u * u * (10 + u * (-15 + 6 * u));
    Sample x;
    for (size_t j = 0; j < 6; ++j)
      x.q[j] = a[j] + blend * (b[j] - a[j]);
    s.samples.push_back(x);
  }
  timePath(s, p);
  for (const auto &x : s.samples)
    check(x.q, t, phase != "HOME");
  return s;
}
Segment Planner::vertical(const std::string &phase, const Joints &a,
                          const Transform &target,
                          chess_robot_twist::Tracker &t, const Profile &p) {
  auto start = model_->tcp(a);
  if ((start.translation().head<2>() - target.translation().head<2>()).norm() >
          config_.position_tolerance ||
      rotationDistance(start, target) > config_.rotation_tolerance)
    throw Failure(PLANNING_FAILED, "vertical operation requires aligned "
                                   "current TCP and fixed orientation");
  // Keep measured/model orientation and XY exactly; target deviations are only
  // tolerance checks.
  Transform end = start;
  end.translation().z() = target.translation().z();
  const double dz = end.translation().z() - start.translation().z();
  if ((phase == "DESCEND" && dz > 0) || (phase == "RETREAT" && dz < 0))
    throw Failure(PLANNING_FAILED, "vertical direction mismatch");
  size_t n =
      subdivisions(std::abs(dz) / config_.cartesian_step, config_.max_samples);
  if (n >= config_.max_samples)
    throw Failure(PLANNING_FAILED, "Cartesian sample budget");
  Segment s{phase, {{a, 0}}};
  Joints seed = a;
  for (size_t i = 1; i <= n; ++i) {
    auto desired = start;
    double u = double(i) / n;
    desired.translation().z() += dz * u;
    auto q = solve(desired, seed);
    if (jointDistance(q, seed) > config_.joint_step)
      throw Failure(PLANNING_FAILED, "IK branch jump; reduce Cartesian step");
    s.samples.push_back({q, 0});
    seed = q;
  }
  timePath(s, p);
  for (const auto &x : s.samples) {
    check(x.q, t, true);
    auto actual = model_->tcp(x.q);
    if ((actual.translation().head<2>() - start.translation().head<2>())
                .norm() > config_.position_tolerance ||
        rotationDistance(actual, start) > config_.rotation_tolerance)
      throw Failure(PLANNING_FAILED, "sampled vertical interpolation deviates");
  }
  return s;
}
Plan Planner::plan(const Goal &g, const Snapshot &state) {
  if (config_.backend == "commissioning" && !safety_check_)
    throw Failure(HARDWARE_DISABLED, "commissioning geometry guard missing");
  if (static_cast<uint8_t>(g.operation) > static_cast<uint8_t>(Operation::Home))
    throw Failure(INVALID_GOAL, "unknown operation");
  if (!state.fresh)
    throw Failure(FEEDBACK_FAULT, "fresh measured state required");
  if (!state.reference)
    throw Failure(REFERENCE_INVALID, "explicit Home reference required");
  const auto &p = config_.profile(g.profile);
  auto t = state.tracker;
  const auto start = state.feedback.q;
  if (g.operation == Operation::Home)
    return {{ptp("HOME", start, config_.home, t, p)}, false};
  validatePose(g.target, config_);
  auto pre = g.target;
  pre.translation().z() += config_.clearance;
  validatePose(pre, config_);
  Plan plan;
  if (g.operation == Operation::Pregrasp ||
      g.operation == Operation::Sequence) {
    // Both current and Home seeds are candidates, each checked along its full
    // path.
    std::optional<Segment> best;
    std::optional<chess_robot_twist::Tracker> best_tracker;
    std::string last;
    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto &seed : {start, config_.home}) {
      try {
        auto trial = t;
        auto q = solve(pre, seed);
        auto s = ptp("PREGRASP", start, q, trial, p);
        auto metrics = trial.metrics();
        double cost = config_.cost_duration * s.samples.back().time +
                      config_.cost_net * std::abs(metrics.net) +
                      config_.cost_total * metrics.total +
                      config_.cost_energy * metrics.energy;
        if (cost < best_cost) {
          best_cost = cost;
          best = std::move(s);
          best_tracker = trial;
        }
      } catch (const Failure &e) {
        if (e.code == CANCELED || e.code == FEEDBACK_FAULT)
          throw;
        last = e.what();
      } catch (const chess_robot_twist::Error &e) {
        last = e.what();
      }
    }
    if (!best)
      throw Failure(PLANNING_FAILED, "no checked pregrasp candidate: " + last);
    plan.segments.push_back(std::move(*best));
    t = *best_tracker;
  }
  if (g.operation == Operation::Descend || g.operation == Operation::Sequence) {
    auto a =
        plan.segments.empty() ? start : plan.segments.back().samples.back().q;
    plan.segments.push_back(vertical("DESCEND", a, g.target, t, p));
  }
  if (g.operation == Operation::Retreat || g.operation == Operation::Sequence) {
    auto a =
        plan.segments.empty() ? start : plan.segments.back().samples.back().q;
    plan.segments.push_back(vertical("RETREAT", a, pre, t, p));
  }
  plan.wait_for_gripper = g.operation == Operation::Sequence;
  size_t count = 0;
  for (const auto &s : plan.segments)
    count += s.samples.size();
  if (count > config_.max_samples)
    throw Failure(PLANNING_FAILED, "whole task sample budget");
  return plan;
}
bool TaskGate::reserve(const std::string &id) {
  std::lock_guard<std::mutex> l(mutex_);
  if (busy_ || fault_latched_ || id.empty())
    return false;
  busy_ = true;
  job_ = id;
  phase_ = "PLANNING";
  cancel_ = false;
  terminal_committed_ = false;
  confirmation_.reset();
  confirmation_deadline_.reset();
  confirmation_used_ = false;
  detail_.clear();
  return true;
}
void TaskGate::release() {
  std::lock_guard<std::mutex> l(mutex_);
  busy_ = false;
  job_.clear();
  phase_ = fault_latched_ ? "FAULT" : "IDLE";
  confirmation_.reset();
  confirmation_deadline_.reset();
}
void TaskGate::latchFault(const std::string &why) {
  std::lock_guard<std::mutex> l(mutex_);
  fault_latched_ = true;
  phase_ = "FAULT";
  detail_ = why;
}
bool TaskGate::cancel(const std::string &id) {
  std::lock_guard<std::mutex> l(mutex_);
  if (!busy_ || terminal_committed_ || id != job_)
    return false;
  cancel_ = true;
  return true;
}
bool TaskGate::commitTerminal() {
  std::lock_guard<std::mutex> l(mutex_);
  terminal_committed_ = true;
  return cancel_;
}
bool TaskGate::canceled() const {
  std::lock_guard<std::mutex> l(mutex_);
  return cancel_;
}
void TaskGate::phase(const std::string &p) {
  std::lock_guard<std::mutex> l(mutex_);
  phase_ = p;
}
void TaskGate::beginGripperWait(Clock::time_point deadline) {
  std::lock_guard<std::mutex> l(mutex_);
  phase_ = "WAIT_GRIPPER";
  confirmation_deadline_ = deadline;
  confirmation_.reset();
  confirmation_used_ = false;
}
std::optional<bool> TaskGate::finishGripperWait() {
  std::lock_guard<std::mutex> l(mutex_);
  // Closing the phase and reading its result are one operation. A confirmation
  // already accepted before its deadline remains valid if scheduling wakes
  // late.
  phase_ = "GRIPPER_CHECKED";
  confirmation_deadline_.reset();
  return confirmation_;
}
bool TaskGate::confirm(const std::string &id, bool success,
                       const std::string &detail) {
  std::lock_guard<std::mutex> l(mutex_);
  if (!busy_ || cancel_ || terminal_committed_ || phase_ != "WAIT_GRIPPER" ||
      id != job_ || confirmation_used_ || !confirmation_deadline_ ||
      Clock::now() >= *confirmation_deadline_)
    return false;
  confirmation_ = success;
  confirmation_used_ = true;
  detail_ = detail.substr(0, 1024);
  return true;
}
std::optional<bool> TaskGate::confirmation() const {
  std::lock_guard<std::mutex> l(mutex_);
  return confirmation_;
}
TaskGate::View TaskGate::view() const {
  std::lock_guard<std::mutex> l(mutex_);
  return {busy_, job_, phase_, detail_};
}
bool TaskGate::initialize(State &s, std::string &detail) {
  std::lock_guard<std::mutex> l(mutex_);
  if (busy_ || fault_latched_) {
    detail = fault_latched_
                 ? "Stop fault latched; operator recovery and restart required"
                 : "busy";
    return false;
  }
  try {
    s.initialize();
    detail = "software reference initialized";
    return true;
  } catch (const std::exception &e) {
    detail = e.what();
    return false;
  }
}
Executor::Executor(const Config &c, Driver &d, State &s, TaskGate &g,
                   std::shared_ptr<const chess_robot_twist::Model> m,
                   std::function<void(const Joints &)> safety_check)
    : config_(c), driver_(d), state_(s), gate_(g), model_(m),
      safety_check_(std::move(safety_check)) {}
void Executor::checkpoint() {
  if (gate_.canceled())
    throw Failure(CANCELED, "goal canceled");
  auto s = state_.snapshot();
  if (!s.fresh)
    throw Failure(FEEDBACK_FAULT, "feedback lost during task");
  if (!s.reference)
    throw Failure(REFERENCE_INVALID, "reference lost during task");
  if (!model_->withinJointBounds(s.feedback.q) ||
      !chess_robot_twist::withinLimits(s.tracker.metrics(),
                                       config_.twist_limits))
    throw Failure(EXECUTION_FAILED, "measured path policy violation");
  if (safety_check_) {
    try {
      safety_check_(s.feedback.q);
    } catch (const Failure &e) {
      throw Failure(EXECUTION_FAILED,
                    std::string("measured geometry: ") + e.what());
    }
  }
  for (size_t j = 0; j < 6; ++j)
    if (config_.backend == "commissioning" &&
        std::abs(s.feedback.speed[j]) > config_.velocity[j] * 1.5 + .002)
      throw Failure(EXECUTION_FAILED, "commissioning measured speed cap");
}
void Executor::waitUntil(Clock::time_point time) {
  while (Clock::now() < time) {
    checkpoint();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
void Executor::arrival(const Joints &goal, double timeout) {
  auto deadline = Clock::now() + std::chrono::duration<double>(timeout);
  std::optional<Clock::time_point> since;
  const auto desired = model_->tcp(goal);
  while (Clock::now() < deadline) {
    checkpoint();
    auto s = state_.snapshot();
    auto actual = model_->tcp(s.feedback.q);
    const bool reached =
        jointDistance(s.feedback.q, goal) <= config_.joint_tolerance &&
        maxSpeed(s.feedback) <= config_.stationary_speed &&
        s.feedback.mode == 5 &&
        (actual.translation() - desired.translation()).norm() <=
            config_.position_tolerance &&
        rotationDistance(actual, desired) <= config_.rotation_tolerance;
    if (reached) {
      if (!since)
        since = Clock::now();
      if (Clock::now() - *since >= std::chrono::duration<double>(config_.dwell))
        return;
    } else
      since.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw Failure(EXECUTION_FAILED, "measured arrival deadline exceeded");
}
void Executor::run(const Plan &plan) {
  if (!config_.hardwareAllowed())
    throw Failure(HARDWARE_DISABLED, "hardware prerequisites are false");
  if (config_.backend == "commissioning" && !safety_check_)
    throw Failure(HARDWARE_DISABLED, "commissioning geometry guard missing");
  if (plan.segments.empty() || plan.segments.size() > 3)
    throw Failure(EXECUTION_FAILED, "invalid execution plan");
  for (const auto &segment : plan.segments)
    if (segment.samples.size() < 2 ||
        segment.samples.size() > config_.max_samples)
      throw Failure(EXECUTION_FAILED, "invalid segment sample count");
  // Check the whole exact execution plan before the first actuation call.
  if (safety_check_)
    for (const auto &segment : plan.segments)
      for (const auto &sample : segment.samples)
        safety_check_(sample.q);
  checkpoint();
  {
    auto initial = state_.snapshot();
    if (initial.feedback.mode != 5 ||
        maxSpeed(initial.feedback) > config_.stationary_speed)
      throw Failure(EXECUTION_FAILED,
                    "hardware execution requires idle stationary start");
  }
  for (const auto &segment : plan.segments) {
    if (segment.phase == "RETREAT" && plan.wait_for_gripper) {
      auto deadline =
          Clock::now() +
          std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(config_.gripper_timeout));
      gate_.beginGripperWait(deadline);
      while (!gate_.confirmation() && Clock::now() < deadline) {
        checkpoint();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      auto confirmation = gate_.finishGripperWait();
      if (!confirmation || !*confirmation)
        throw Failure(GRIPPER_FAILED,
                      "gripper failure or confirmation timeout; no retreat");
    }
    gate_.phase(segment.phase);
    auto start = Clock::now();
    if (jointDistance(state_.snapshot().feedback.q, segment.samples.front().q) >
        config_.joint_tolerance)
      throw Failure(EXECUTION_FAILED, "segment start drift");
    for (size_t i = 1; i < segment.samples.size(); ++i) {
      checkpoint();
      const auto &x = segment.samples[i];
      auto due =
          start +
          std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(x.time - config_.sample_period));
      waitUntil(due);
      if (Clock::now() - due >
          std::chrono::duration<double>(config_.sample_period))
        throw Failure(EXECUTION_FAILED, "ServoJ schedule overrun");
      auto measured = state_.snapshot();
      if (jointDistance(measured.feedback.q, segment.samples[i - 1].q) >
          config_.max_tracking_error)
        throw Failure(EXECUTION_FAILED, "measured tracking error");
      if (!model_->withinJointBounds(x.q))
        throw Failure(EXECUTION_FAILED, "execution joint bound");
      checkpoint();
      // Mark ownership before send: an ambiguous ACK may still have started
      // motion.
      owned_ = true;
      attempted_ = true;
      driver_.servo(x.q, config_.sample_period);
    }
    arrival(segment.samples.back().q, config_.arrival_timeout);
    owned_ = false;
  }
}
void Executor::stopOwned() {
  if (!owned_)
    return;
  // Uses the controller's one permitted command session. Any in-flight call
  // has a bounded deadline and is included in the configured stop budget.
  auto deadline =
      Clock::now() + std::chrono::duration<double>(config_.stop_timeout);
  try {
    driver_.stop();
  } catch (const std::exception &e) {
    throw Failure(STOP_FAILED,
                  std::string("Stop acknowledgement failed: ") + e.what());
  }
  std::optional<Clock::time_point> since;
  while (Clock::now() < deadline) {
    auto s = state_.snapshot();
    if (s.fresh && maxSpeed(s.feedback) <= config_.stationary_speed &&
        s.feedback.mode == 5) {
      if (!since)
        since = Clock::now();
      if (Clock::now() - *since >=
          std::chrono::duration<double>(config_.dwell)) {
        owned_ = false;
        return;
      }
    } else
      since.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw Failure(
      STOP_FAILED,
      "Stop could not be verified from fresh idle stationary feedback");
}
} // namespace chess_robot_arm
