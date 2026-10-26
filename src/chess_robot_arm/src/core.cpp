#include "chess_robot_arm/core.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <cmath>
#include <limits>
#include <thread>
#include <fstream>
#include "chess_robot_arm/feedback_frame.hpp"
#include "chess_robot_arm/driver.hpp"

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
              backend == "commissioning" || backend == "exploration",
          "backend must be read_only, hardware or commissioning");
  require(!urdf.empty(), "urdf_file is required");
  require(!enable_after_history_restore || power_cycle_handoff, "explicit power-cycle handoff required before recovery enable");
  in_addr address{};
  require(inet_pton(AF_INET, ip.c_str(), &address) == 1,
          "controller.ip must be an IPv4 literal");
  require(!test_fixture || ip == "127.0.0.1",
          "test_fixture requires IPv4 loopback");
  require(dashboard_port > 0 && dashboard_port < 65536 && feedback_port > 0 &&
              feedback_port < 65536 && timeout_ms > 0 && timeout_ms <= 5000 &&
              stop_ack_timeout_ms > 0 && stop_ack_timeout_ms <= 5000 &&
              enable_ack_timeout_ms>0 && enable_ack_timeout_ms<=10000,
          "invalid network bounds");
  for (size_t i = 0; i < 6; ++i)
    require(std::isfinite(home[i]) && positive(velocity[i]) &&
                positive(acceleration[i]),
            "invalid Home or manufacturer joint limits");
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
  require(std::isfinite(clearance) && clearance >= 0, "invalid approach clearance");
  require(!home_roundtrip_precheck || clearance == 0, "roundtrip precheck requires direct TCP (zero approach clearance)");
  for (double x : {downward_tolerance,
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
                  (2 * double(timeout_ms) + stop_ack_timeout_ms) / 1000 + dwell + servo_ack_timeout,
          "sampling or ServoJ configuration out of bounds");
  for (double weight : {cost_duration, cost_net, cost_total, cost_energy})
    require(positive(weight), "invalid candidate cost weight");
  require(positive(neutral_band) && positive(twist_hysteresis) &&
              twist_hysteresis < neutral_band && positive(duration_hysteresis),
          "invalid selection deadbands");
  require(positive(servo_ack_timeout) && servo_ack_timeout <= .5 &&
              servo_max_pending >= 1 && servo_max_pending <= 8,
          "invalid bounded ServoJ acknowledgement window");
  require(servo_command_time==0 || (std::isfinite(servo_command_time) &&
              servo_command_time>=sample_period && servo_command_time<=.2),
          "invalid ServoJ following horizon");
  require(!profiles.empty() && profiles.size() <= 32, "invalid profile count");
  for (size_t i = 0; i < profiles.size(); ++i) {
    require(!profiles[i].name.empty() && profiles[i].name.size() <= 64 &&
                positive(profiles[i].speed) &&
                positive(profiles[i].acceleration),
            "invalid profile");
    for (size_t j = 0; j < i; ++j)
      require(profiles[i].name != profiles[j].name, "duplicate profile");
  }
  profile(default_profile);
  if (backend == "commissioning" || backend == "exploration") {
    require(!test_fixture && !safety_model_file.empty(),
            "commissioning requires real geometry guard");
    if (backend == "commissioning") require(positive(commissioning_joint_radius) &&
                commissioning_joint_radius <= .12 &&
                positive(commissioning_tcp_radius) &&
                commissioning_tcp_radius <= .04 && freshness <= .1 &&
                twist_limits.net_limit_rad <= pi / 9 &&
                twist_limits.total_limit_rad <= pi / 9,
            "commissioning envelope/feedback/twist limits too wide");
    require(freshness <= .1,
            "commissioning/exploration feedback caps too wide");
    // Speed comes from the selected manufacturer limits/profile, independently
    // of commissioning geometry and measured stop/arrival tolerances.
    if (backend == "exploration") {
      require(!exploration_boundaries.empty() && exploration_boundaries.size() <= 16 &&
                  twist_limits.net_limit_rad <= 2*pi && twist_limits.total_limit_rad <= 2*pi,
              "exploration boundaries/global twist cap required");
      double r=0, j=0, twist=0;
      for (const auto &b : exploration_boundaries) {
        profile(b.profile);
        require(positive(b.tcp_radius) && positive(b.joint_radius) && positive(b.twist_limit) &&
                    b.tcp_radius >= r && b.joint_radius >= j && b.twist_limit >= twist &&
                    b.tcp_radius <= 2 && b.joint_radius <= 2*pi &&
                    b.twist_limit <= std::min(twist_limits.net_limit_rad,twist_limits.total_limit_rad),
                "invalid/non-monotonic exploration boundary");
        r=b.tcp_radius; j=b.joint_radius; twist=b.twist_limit;
      }
    }
  }
}
bool Config::hardwareAllowed() const {
  if (motion_paused || (!test_fixture && (controller_family != "v4" ||
      controller_firmware.rfind("4.", 0) != 0))) return false;
  if (backend == "commissioning" || backend == "exploration")
    return enable && !test_fixture && !safety_model_file.empty();
  return backend == "hardware" && enable && tcp_verified && limits_verified &&
         execution_verified && servo_certified;
}
Config Config::forProfile(const std::string &requested) const {
  const auto &name = requested.empty() ? default_profile : requested;
  auto c = *this;
  if (backend == "exploration") {
    auto it=std::find_if(exploration_boundaries.begin(), exploration_boundaries.end(),
                       [&](const ExplorationBoundary &b){return b.profile==name;});
    if (it==exploration_boundaries.end()) throw Failure(INVALID_GOAL,"profile has no exploration boundary");
    c.commissioning_tcp_radius=it->tcp_radius;
    c.commissioning_joint_radius=it->joint_radius;
    c.twist_limits={it->twist_limit,it->twist_limit};
  }
  return c;
}
const Profile &Config::profile(const std::string &requested) const {
  const auto &n = requested.empty() ? default_profile : requested;
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
      throw Failure(FEEDBACK_FAULT, "controller mode "+std::to_string(f.mode)+" is not a supported healthy mode");
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
void State::restore(const std::vector<std::string> &files, bool no_motion) {
  require(no_motion, "history handoff requires explicit no-unobserved-motion attestation");
  require(!files.empty() && files.size() <= 16, "invalid history file count");
  chess_robot_twist::Tracker trial(model_, config_.tracker_options);
  FeedbackSequence sequence;
  Feedback previous;
  bool have=false;
  size_t count=0;
  std::optional<int64_t> stationary_hold_since;
  Joints stationary_hold_anchor{};
  int64_t last_host=0, recent_stationary_home=0;
  bool recorded_fault_tail=false;
  for (size_t file_index=0;file_index<files.size();++file_index) {
    const auto &path=files[file_index];
    std::ifstream f(path, std::ios::binary);
    require(bool(f), "cannot read history audit");
    bool first=true;
    int64_t previous_host=0;
    for (;;) {
      int64_t host;
      f.read(reinterpret_cast<char *>(&host), sizeof(host));
      if (!f && f.eof() && f.gcount()==0) break;
      require(bool(f), "truncated history timestamp");
      FeedbackFrame bytes{};
      f.read(reinterpret_cast<char *>(bytes.data()), bytes.size());
      require(bool(f), "truncated history frame");
      require(++count <= 2000000, "history replay budget exceeded");
      require(host > 0 && (!previous_host || (host > previous_host &&
                  double(host-previous_host)*1e-9 <= config_.freshness)),
              "history feedback gap/regression inside audit");
      previous_host=host;
      if (!sequence.accept(bytes)) continue;
      // Recorded host UTC is retained as evidence; never reinterpreted as ROS time.
      auto current=decodeFeedbackFrame(bytes,Clock::time_point{},0);
      require(model_->withinJointBounds(current.q),"invalid historical joints");
      if (current.mode==9 && config_.power_cycle_handoff) {
        // Fault encoders are evidence of the last small joint displacement,
        // never evidence of healthy motion or arrival. Only a terminal fault
        // immediately following stationary formal Home is recoverable here.
        require(have && file_index+1==files.size() && recent_stationary_home>0 &&
                    host-recent_stationary_home <= int64_t(config_.freshness*1e9) &&
                    jointDistance(current.q,config_.home)<=config_.home_tolerance,
                "power-cycle recovery requires terminal near-Home fault after recorded stationary Home");
        recorded_fault_tail=true;
      } else {
        require(!recorded_fault_tail,"fault history must be a terminal suffix");
        require(current.mode==4 || current.mode==5 || current.mode==7,"invalid historical mode");
      }
      if (current.mode==5 && maxSpeed(current)<=config_.stationary_speed &&
          jointDistance(current.q,config_.home)<=config_.joint_tolerance)
        recent_stationary_home=host;
      if (!have) {
        require(current.mode==5 && maxSpeed(current)<=config_.stationary_speed,
                "history must begin at stationary known Home");
        trial.initializeAtHome(current.q,config_.home_tolerance);
      } else {
        // ServoJ can hold a stationary position in mode 7 after an ACK fault.
        // It is not accepted as the final endpoint: a separate stopped mode-5
        // audit must join it, with a recorded stationary dwell before the gap.
        const bool recorded_hold = previous.mode==7 && stationary_hold_since &&
            double(last_host-*stationary_hold_since)*1e-9>=config_.dwell &&
            maxSpeed(previous)<=config_.stationary_speed;
        if (first) require((previous.mode==5 || recorded_hold) && current.mode==5 &&
                       jointDistance(previous.q,current.q)<=.001 &&
                       maxSpeed(current)<=config_.stationary_speed,
                       "history file handoff moved or was not stopped");
        trial.advance(current.q);
      }
      require(chess_robot_twist::withinLimits(trial.metrics(),config_.twist_limits),
              "historical twist exceeds current hard limits");
      if (current.mode==7 && maxSpeed(current)<=config_.stationary_speed) {
        if (first || !stationary_hold_since || jointDistance(current.q,stationary_hold_anchor)>.001) {
          stationary_hold_since=host;stationary_hold_anchor=current.q;
        }
      } else stationary_hold_since.reset();
      last_host=host;
      first=false; have=true; previous=current;
    }
    require(!first, "empty history audit");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  require(have && have_ && !tracker_.valid(), "history restore needs invalid reference and real feedback");
  const bool power_handoff=config_.power_cycle_handoff &&
      (last_.mode==4 || last_.mode==5) &&
      jointDistance(last_.q,config_.home)<=config_.home_tolerance &&
      jointDistance(previous.q,config_.home)<=config_.home_tolerance;
  require(Clock::now()-last_.received <= std::chrono::duration<double>(config_.freshness) &&
              (last_.mode==5 || power_handoff) && maxSpeed(last_)<=config_.stationary_speed &&
              stationary_since_ && Clock::now()-*stationary_since_>=std::chrono::duration<double>(config_.dwell),
          "history restore needs fresh stationary idle dwell");
  require(((previous.mode==5 && maxSpeed(previous)<=config_.stationary_speed) ||
               (power_handoff && recorded_fault_tail && previous.mode==9)) &&
              (last_.controller_stamp>previous.controller_stamp || power_handoff) &&
              jointDistance(last_.q,previous.q)<=.001,
          "current feedback does not join historical stopped/fault endpoint");
  trial.advance(last_.q);
  require(chess_robot_twist::withinLimits(trial.metrics(),config_.twist_limits),"restored twist limit");
  tracker_=std::move(trial);
  diagnostic_=power_handoff ? "explicit power-cycle audit handoff; winding retained, controller clock epoch not joined" :
      "explicit audit history restored without zeroing twist";
}
bool preferCandidate(const chess_robot_twist::Metrics &a, double duration,
                     const chess_robot_twist::Metrics &b, double incumbent_duration,
                     const Config &c) {
  // Total first prevents cancellation of opposing local twists from masquerading
  // as neutral. Absolute net breaks total-twist ties. Time cannot buy twist
  // outside the explicit angular deadband, at any profile speed or weight.
  for (auto pair : {std::pair<double, double>{a.total, b.total},
                    {std::abs(a.net), std::abs(b.net)}}) {
    const double av = std::max(0.0, pair.first - c.neutral_band);
    const double bv = std::max(0.0, pair.second - c.neutral_band);
    if (av < bv - c.twist_hysteresis) return true;
    if (bv < av - c.twist_hysteresis) return false;
  }
  // Equivalent twist: require meaningful time savings to leave the incumbent.
  // Energy and legacy angular weights no longer trade against neutrality.
  return duration < incumbent_duration - c.duration_hysteresis;
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
void Planner::timePath(Segment &segment, const Profile &profile) {
  // C1 Hermite interpolation of the checked geometric knots, driven by a
  // quintic scalar law with zero endpoint velocity. Every resulting sample is
  // subsequently checked by the same FK/vertical/twist/geometry gates.
  const auto knots=segment.samples;
  const size_t n=knots.size()-1;
  const double dt=config_.sample_period;
  double duration=2*dt;
  for (size_t j=0;j<6;++j) {
    double distance=0;
    for (size_t i=1;i<=n;++i) distance+=std::abs(knots[i].q[j]-knots[i-1].q[j]);
    duration=std::max(duration,1.875*distance/std::min(profile.speed,config_.velocity[j]));
    duration=std::max(duration,std::sqrt(5.774*distance/std::min(profile.acceleration,config_.acceleration[j])));
  }
  for (int attempt=0;attempt<12;++attempt) {
    size_t steps=subdivisions(duration/dt,config_.max_samples);
    std::vector<Sample> out; out.reserve(steps+1);
    for (size_t k=0;k<=steps;++k) {
      if (k%200==0) checkpoint_();
      double t=double(k)/steps;
      double u=t*t*t*(10+t*(-15+6*t))*n;
      size_t i=std::min(n-1,size_t(u)); double x=u-i;
      Sample q; q.time=k*dt;
      for (size_t j=0;j<6;++j) {
        double a=knots[i].q[j], b=knots[i+1].q[j];
        double ma=i ? (knots[i+1].q[j]-knots[i-1].q[j])/2 : b-a;
        double mb=i+1<n ? (knots[i+2].q[j]-knots[i].q[j])/2 : b-a;
        q.q[j]=(2*x*x*x-3*x*x+1)*a+(x*x*x-2*x*x+x)*ma+
                  (-2*x*x*x+3*x*x)*b+(x*x*x-x*x)*mb;
      }
      out.push_back(q);
    }
    out.front().q=knots.front().q; out.back().q=knots.back().q;
    Joints previous_velocity{}; double dilation=1;
    for (size_t i=1;i<out.size();++i)
      for (size_t j=0;j<6;++j) {
        double v=(out[i].q[j]-out[i-1].q[j])/dt;
        dilation=std::max(dilation,std::abs(v)/std::min(profile.speed,config_.velocity[j]));
        dilation=std::max(dilation,std::sqrt(std::abs(v-previous_velocity[j])/dt/std::min(profile.acceleration,config_.acceleration[j])));
        previous_velocity[j]=v;
      }
    for (size_t j=0;j<6;++j)
      dilation=std::max(dilation,std::sqrt(std::abs(previous_velocity[j])/dt/std::min(profile.acceleration,config_.acceleration[j])));
    if (dilation<=1+1e-10) { segment.samples=std::move(out); return; }
    duration=steps*dt*dilation*1.005;
  }
  throw Failure(PLANNING_FAILED,"smooth time parameterization did not converge");
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
    // Geometric knots lie on the joint-space line. timePath supplies the
    // quintic time law once; blending both here and there wastes speed budget.
    double blend = u;
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
  if ((config_.backend == "commissioning" || config_.backend == "exploration") && !safety_check_)
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
  if (config_.home_roundtrip_precheck && g.operation != Operation::Pregrasp)
    throw Failure(INVALID_GOAL,"roundtrip test mode only accepts direct TCP target or formal Home");
  if (config_.home_roundtrip_precheck &&
      jointDistance(start,config_.home)>config_.joint_tolerance)
    throw Failure(PLANNING_FAILED,"roundtrip outward requires measured formal Home start");
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
    double best_duration = 0, best_return_duration = 0;
    chess_robot_twist::Metrics best_metrics;
    int candidate_index=0, selected_index=-1;
    for (const auto &seed : {start, config_.home}) {
      const int index=candidate_index++;
      try {
        auto trial = t;
        auto q = solve(pre, seed);
        auto s = ptp("PREGRASP", start, q, trial, p);
        auto metrics = trial.metrics();
        double duration = s.samples.back().time;
        double return_duration = 0;
        if (config_.home_roundtrip_precheck) {
          // trial already contains this exact outward path's winding history.
          // Test the predicted target -> unchanged formal Home before any send.
          // This copy never mutates the live State/Tracker or sends motion.
          auto return_tracker = trial;
          try {
            const auto back = ptp("HOME", q, config_.home, return_tracker, p);
            return_duration = back.samples.back().time;
          } catch (const Failure &e) {
            if (e.code==CANCELED || e.code==FEEDBACK_FAULT) throw;
            throw Failure(PLANNING_FAILED,std::string("predicted Home return rejected: ")+e.what());
          }
        }
        plan.selection += " seed="+std::to_string(index)+" total="+std::to_string(metrics.total)+
            " net="+std::to_string(metrics.net)+" seconds="+std::to_string(duration);
        if (!best || preferCandidate(metrics, duration, best_metrics,
                                     best_duration, config_)) {
          best_metrics = metrics;
          selected_index=index;
          best_duration = duration;
          best_return_duration = return_duration;
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
    plan.selection += " selected="+std::to_string(selected_index);
    if (config_.home_roundtrip_precheck)
      plan.selection += " home_return_checked=true home_return_duration_s="+std::to_string(best_return_duration);
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
bool TaskGate::restore(State &state, const std::vector<std::string> &files,
                       bool attested, std::string &detail) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (busy_ || fault_latched_) { detail="restore requires idle non-faulted admission"; return false; }
  try { state.restore(files,attested); detail="audit history restored; twist not zeroed"; return true; }
  catch (const std::exception &e) { detail=e.what(); return false; }
}

Executor::Executor(const Config &c, Driver &d, State &s, TaskGate &g,
                   std::shared_ptr<const chess_robot_twist::Model> m,
                   std::function<void(const Joints &)> safety_check)
    : config_(c), driver_(d), state_(s), gate_(g), model_(m),
      safety_check_(std::move(safety_check)) {}
void Executor::checkpoint() {
  driver_.checkStream();
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
  // No debug-profile-derived instantaneous velocity trip. Freshness, mode,
  // continuous winding, joint/geometry and tracking checks remain authoritative.
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
  if ((config_.backend == "commissioning" || config_.backend == "exploration") && !safety_check_)
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
    double previous_io_s = 0;
    if (jointDistance(state_.snapshot().feedback.q, segment.samples.front().q) >
        config_.joint_tolerance)
      throw Failure(EXECUTION_FAILED, "segment start drift");
    driver_.beginStream();
    for (size_t i = 1; i < segment.samples.size(); ++i) {
      checkpoint();
      const auto &x = segment.samples[i];
      auto due =
          start +
          std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(x.time - config_.sample_period));
      waitUntil(due);
      const double late_s = std::chrono::duration<double>(Clock::now() - due).count();
      if (Clock::now() - due >
          std::chrono::duration<double>(config_.sample_period))
        throw Failure(EXECUTION_FAILED, "ServoJ schedule overrun: sample=" +
                      std::to_string(i) + " late_s=" + std::to_string(late_s) +
                      " previous_io_s=" + std::to_string(previous_io_s));
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
      const auto io_start = Clock::now();
      driver_.servo(x.q, config_.sample_period);
      previous_io_s = std::chrono::duration<double>(Clock::now() - io_start).count();
    }
    driver_.endStream();
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
