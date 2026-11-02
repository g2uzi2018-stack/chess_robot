#include "chess_robot_arm/core.hpp"
#include "chess_robot_arm/checkpoint.hpp"
#include "chess_robot_arm/driver.hpp"
#include "chess_robot_arm/geometry_guard.hpp"
#include "chess_robot_arm/net.hpp"
#include <chess_robot_interfaces/action/move_arm.hpp>
#include <chess_robot_interfaces/action/move_joints.hpp>
#include <chess_robot_interfaces/msg/arm_state.hpp>
#include <chess_robot_interfaces/srv/confirm_grasp.hpp>
#include <chess_robot_interfaces/srv/initialize_reference.hpp>
#include <chess_robot_interfaces/srv/restore_reference.hpp>
#include <chess_robot_interfaces/srv/restore_checkpoint.hpp>
#include <map>
#include <condition_variable>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <thread>

namespace chess_robot_arm {
using Move = chess_robot_interfaces::action::MoveArm;
using JointMove = chess_robot_interfaces::action::MoveJoints;
using JointHandle = rclcpp_action::ServerGoalHandle<JointMove>;
using Handle = rclcpp_action::ServerGoalHandle<Move>;
namespace {
std::string idString(const rclcpp_action::GoalUUID &id) {
  std::ostringstream s;
  for (auto x : id)
    s << std::hex << std::setw(2) << std::setfill('0') << int(x);
  return s.str();
}
geometry_msgs::msg::PoseStamped poseMessage(const Transform &t,
                                            const rclcpp::Time &stamp) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "base_link";
  p.header.stamp = stamp;
  p.pose.position.x = t.translation().x();
  p.pose.position.y = t.translation().y();
  p.pose.position.z = t.translation().z();
  Eigen::Quaterniond q(t.linear());
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  return p;
}
Transform poseTransform(const geometry_msgs::msg::Pose &p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y,
                       p.orientation.z);
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) ||
      std::abs(q.norm() - 1) > 1e-6)
    throw Failure(INVALID_GOAL,
                  "quaternion must be finite, nonzero and normalized");
  Transform t = Transform::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return t;
}
} // namespace
class ArmNode final : public rclcpp::Node {
  Config config_;
  std::shared_ptr<const chess_robot_twist::Model> model_;
  std::unique_ptr<V4Driver> driver_;
  std::unique_ptr<State> state_;
  std::unique_ptr<CheckpointStore> checkpoint_store_;
  Clock::time_point checkpoint_saved_{}, checkpoint_attempt_{};
  std::atomic<bool> checkpoint_save_due_{true};
  std::string checkpoint_status_;
  bool checkpoint_enable_pending_=false;
  bool checkpoint_had_reference_=false;
  Clock::time_point checkpoint_enable_deadline_{};
  std::string checkpoint_enable_status_;
  bool checkpoint_auto_inhibited_=false;
  bool checkpoint_motion_observed_=false;
  std::unique_ptr<GeometryGuard> geometry_guard_;
  std::map<std::string,std::shared_ptr<GeometryGuard>> exploration_guards_;
  std::function<void(const Joints &)> safetyCheck(const std::string &profile="") {
    if (config_.backend=="exploration") {
      auto guard=exploration_guards_.at(profile.empty() ? config_.default_profile : profile);
      return [guard](const Joints &q) { guard->check(q); };
    }
    if (!geometry_guard_)
      return {};
    return [this](const Joints &q) { geometry_guard_->check(q); };
  }
  TaskGate gate_;
  std::atomic<bool> shutdown_{false}, feedback_shutdown_{false};
  std::thread feedback_thread_, worker_thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::function<void()> pending_, terminal_;
  rclcpp_action::Server<JointMove>::SharedPtr joint_action_;
  rclcpp_action::Server<Move>::SharedPtr action_;
  rclcpp::Service<chess_robot_interfaces::srv::ConfirmGrasp>::SharedPtr
      confirm_;
  rclcpp::Service<chess_robot_interfaces::srv::InitializeReference>::SharedPtr
      initialize_;
  rclcpp::Service<chess_robot_interfaces::srv::RestoreReference>::SharedPtr restore_;
  rclcpp::Service<chess_robot_interfaces::srv::RestoreCheckpoint>::SharedPtr restore_checkpoint_;
  rclcpp::Publisher<chess_robot_interfaces::msg::ArmState>::SharedPtr
      state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::TimerBase::SharedPtr timer_, publish_timer_;
  double state_publish_hz_=20.;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_lock_;
  std::string endpoint(const std::string &suffix) const {
    return (config_.test_fixture ? "/simulation/arm/" : "/arm/") + suffix;
  }
  template <class T> T param(const std::string &n, const T &v) {
    return declare_parameter<T>(n, v);
  }
  std::vector<double> array(const std::string &name,
                            const std::vector<double> &defaults, size_t size) {
    auto a = param(name, defaults);
    if (a.size() != size)
      throw Failure(INVALID_GOAL, name + " size mismatch");
    for (double x : a)
      if (!std::isfinite(x))
        throw Failure(INVALID_GOAL, name + " nonfinite");
    return a;
  }
  void configure() {
    auto &c = config_;
    state_publish_hz_=param("feedback.publish_hz",20.);
    if(!std::isfinite(state_publish_hz_) || state_publish_hz_<1. || state_publish_hz_>125.)
      throw Failure(INVALID_GOAL,"feedback.publish_hz must be finite and in [1,125]; ROS publication only");
    c.backend = param("backend", c.backend);
    c.ip = param("controller.ip", c.ip);
    c.test_fixture = param("test_fixture", false);
    c.controller_family = param("controller.protocol_family", c.controller_family);
    c.controller_firmware = param("controller.firmware_version", c.controller_firmware);
    c.motion_paused = param("motion.paused", true);
    c.default_profile = param("profiles.default", c.default_profile);
    c.urdf = param("urdf_file", std::string(""));
    c.dashboard_port = param("controller.dashboard_port", c.dashboard_port);
    c.feedback_port = param("controller.feedback_port", c.feedback_port);
    c.timeout_ms = param("controller.timeout_ms", c.timeout_ms);
    c.enable_ack_timeout_ms = param("controller.enable_ack_timeout_ms", c.enable_ack_timeout_ms);
    c.stop_ack_timeout_ms =
        param("controller.stop_ack_timeout_ms", c.stop_ack_timeout_ms);
    auto h = array("home.joint_positions_rad",
                   std::vector<double>(c.home.begin(), c.home.end()), 6);
    std::copy(h.begin(), h.end(), c.home.begin());
    auto reference = array("reference.joint_positions_rad",
                   std::vector<double>(c.twist_reference.begin(), c.twist_reference.end()), 6);
    std::copy(reference.begin(), reference.end(), c.twist_reference.begin());
    auto names = param("home.joint_names",
                       std::vector<std::string>{"joint1", "joint2", "joint3",
                                                "joint4", "joint5", "joint6"});
    if (names != std::vector<std::string>{"joint1", "joint2", "joint3",
                                          "joint4", "joint5", "joint6"})
      throw Failure(INVALID_GOAL, "unexpected Home joint order");
    if (!param("home.taught", true))
      throw Failure(INVALID_GOAL, "taught Home required");
    auto v =
        array("limits.velocity_rad_s",
              std::vector<double>(c.velocity.begin(), c.velocity.end()), 6);
    std::copy(v.begin(), v.end(), c.velocity.begin());
    auto a = array(
        "limits.acceleration_rad_s2",
        std::vector<double>(c.acceleration.begin(), c.acceleration.end()), 6);
    std::copy(a.begin(), a.end(), c.acceleration.begin());
    auto xyz = array("tcp.translation_m", {0, 0, 0.2}, 3);
    auto rpy = array("tcp.rpy_rad", {pi / 2, -pi / 2, 0}, 3);
    c.flange_tcp.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    c.flange_tcp.linear() =
        (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    auto lo = array("workspace.min_m", {-0.8, -0.8, 0}, 3),
         hi = array("workspace.max_m", {0.8, 0.8, 1.2}, 3),
         axis = array("grasp_axis", {1, 0, 0}, 3);
    for (size_t i = 0; i < 3; ++i) {
      c.workspace_min[i] = lo[i];
      c.workspace_max[i] = hi[i];
      c.grasp_axis[i] = axis[i];
    }
#define PARAM(field, name) c.field = param(name, c.field)
    PARAM(clearance, "approach_clearance_m");
    PARAM(power_cycle_handoff, "reference.power_cycle_handoff");
    PARAM(power_cycle_endpoint_tolerance, "reference.power_cycle_endpoint_tolerance_rad");
    PARAM(power_cycle_join_file, "reference.power_cycle_join_file");
    PARAM(power_cycle_stopped_join_file, "reference.power_cycle_stopped_join_file");
    PARAM(power_cycle_fault_restart_file, "reference.power_cycle_fault_restart_file");
    PARAM(power_cycle_fault_restart_offset_bytes, "reference.power_cycle_fault_restart_offset_bytes");
    PARAM(power_cycle_fault_restart_sha256, "reference.power_cycle_fault_restart_sha256");
    PARAM(feedback_trace, "diagnostics.feedback_trace");
    PARAM(enable_after_history_restore, "controller.enable_after_history_restore");
    PARAM(home_roundtrip_precheck, "testing.home_roundtrip_precheck");
    PARAM(downward_tolerance, "tolerances.downward_rad");
    PARAM(stamp_age, "capture.max_age_s");
    PARAM(stamp_future, "capture.future_tolerance_s");
    PARAM(freshness, "feedback.freshness_s");
    PARAM(home_tolerance, "reference.home_tolerance_rad");
    PARAM(stationary_speed, "tolerances.stationary_rad_s");
    PARAM(dwell, "tolerances.dwell_s");
    PARAM(position_tolerance, "tolerances.position_m");
    PARAM(rotation_tolerance, "tolerances.rotation_rad");
    PARAM(joint_tolerance, "tolerances.joint_rad");
    PARAM(joint_step, "sampling.joint_step_rad");
    PARAM(cartesian_step, "sampling.cartesian_step_m");
    PARAM(sample_period, "sampling.period_s");
    PARAM(planning_timeout, "planning_timeout_s");
    PARAM(gripper_timeout, "gripper_timeout_s");
    PARAM(arrival_timeout, "arrival_timeout_s");
    PARAM(stop_timeout, "stop_timeout_s");
    PARAM(servo_ahead, "servo.aheadtime");
    PARAM(servo_gain, "servo.gain");
    PARAM(servo_ack_timeout, "servo.ack_timeout_s");
    PARAM(servo_command_time, "servo.command_time_s");
    PARAM(max_tracking_error, "tolerances.tracking_rad");
    PARAM(cost_duration, "twist.cost_duration");
    PARAM(cost_net, "twist.cost_net");
    PARAM(cost_total, "twist.cost_total");
    PARAM(cost_energy, "twist.cost_energy");
    PARAM(neutral_band, "twist.neutral_band_rad");
    PARAM(twist_hysteresis, "twist.switch_hysteresis_rad");
    PARAM(duration_hysteresis, "twist.duration_hysteresis_s");
    PARAM(twist_limits.net_limit_rad, "twist.net_limit_rad");
    PARAM(twist_limits.total_limit_rad, "twist.total_limit_rad");
    PARAM(tracker_options.max_joint_step_rad, "reference.max_joint_step_rad");
    PARAM(tracker_options.max_phase_step_rad, "reference.max_phase_step_rad");
    PARAM(enable, "hardware.enable");
    PARAM(tcp_verified, "hardware.tcp_verified");
    PARAM(limits_verified, "hardware.limits_verified");
    PARAM(execution_verified, "hardware.execution_profile_verified");
    PARAM(servo_certified, "hardware.servoj_certified");
    PARAM(safety_model_file, "safety.model_file");
    PARAM(audit_directory, "audit.directory");
    PARAM(checkpoint_file, "checkpoint.file");
    PARAM(checkpoint_period, "checkpoint.period_s");
    PARAM(checkpoint_dwell, "checkpoint.stopped_dwell_s");
    PARAM(checkpoint_endpoint_tolerance, "checkpoint.endpoint_tolerance_rad");
    PARAM(checkpoint_auto_restore, "checkpoint.auto_restore");
    PARAM(checkpoint_no_unobserved_motion, "checkpoint.no_unobserved_motion");
    PARAM(checkpoint_enable_after_restore, "checkpoint.enable_after_restore");
    PARAM(commissioning_joint_radius, "commissioning.joint_radius_rad");
    PARAM(commissioning_tcp_radius, "commissioning.tcp_radius_m");
#undef PARAM
    auto audit_bytes=param<int64_t>("audit.max_file_bytes",64*1024*1024);
    auto audit_keep=param<int64_t>("audit.keep_files",16);
    if(audit_bytes<1448 || audit_bytes>1024LL*1024*1024 || audit_keep<1 || audit_keep>128)
      throw Failure(INVALID_GOAL,"invalid audit rotation bounds");
    c.audit_max_bytes=size_t(audit_bytes);c.audit_keep_files=size_t(audit_keep);
    int64_t count = param<int64_t>("sampling.max_samples", 20000);
    if (count < 2 || count > 100000)
      throw Failure(INVALID_GOAL, "sample budget out of range");
    c.max_samples = static_cast<size_t>(count);
    int64_t window=param<int64_t>("servo.max_pending",6);
    if (window<1 || window>8) throw Failure(INVALID_GOAL,"invalid ServoJ window");
    c.servo_max_pending=size_t(window);
    auto profile_names =
        param("profiles.names", std::vector<std::string>{"low", "high", "conservative"});
    c.profiles.clear();
    for (const auto &n : profile_names) {
      const double speed = n=="high" ? 2.36 : .708;
      const double acceleration = n=="high" ? 4.72 : 1.416;
      c.profiles.push_back({n, param("profiles." + n + ".speed_rad_s", speed),
          param("profiles." + n + ".acceleration_rad_s2", acceleration)});
    }
    if (c.backend=="exploration") {
      auto stages=param("exploration.stages",std::vector<std::string>{});
      for (const auto &name : stages)
        c.exploration_boundaries.push_back({name,
          param("exploration."+name+".tcp_radius_m",.08),
          param("exploration."+name+".joint_radius_rad",.3),
          param("exploration."+name+".twist_limit_rad",pi/6)});
      if (!c.exploration_boundaries.empty()) {
        c.commissioning_tcp_radius=c.exploration_boundaries.back().tcp_radius;
        c.commissioning_joint_radius=c.exploration_boundaries.back().joint_radius;
      }
    }
    c.validate();
  }
  Goal decode(const Move::Goal &g) {
    if (g.operation > Move::Goal::HOME)
      throw Failure(INVALID_GOAL, "unknown operation");
    Goal goal;
    goal.operation = static_cast<Operation>(g.operation);
    goal.profile = g.profile.empty() ? config_.default_profile : g.profile;
    goal.plan_only = g.plan_only;
    config_.profile(g.profile);
    config_.forProfile(g.profile);
    if (goal.operation != Operation::Home) {
      validateCapture(g.target.header.frame_id,
                      rclcpp::Time(g.target.header.stamp).seconds(),
                      now().seconds(), config_);
      goal.target = poseTransform(g.target.pose);
      validatePose(goal.target, config_);
    }
    return goal;
  }
  Goal decode(const JointMove::Goal &g) {
    validateCapture(g.header.frame_id, rclcpp::Time(g.header.stamp).seconds(), now().seconds(), config_);
    Goal goal;
    goal.operation=Operation::Joint; goal.positions=g.positions; goal.expected_start=g.expected_start;
    goal.profile=g.profile.empty() ? config_.default_profile : g.profile;
    goal.plan_only=g.plan_only;
    config_.profile(goal.profile); config_.forProfile(goal.profile);
    for(size_t i=0;i<6;++i)
      if(!std::isfinite(g.positions[i]) || !std::isfinite(g.expected_start[i]))
        throw Failure(INVALID_GOAL,"nonfinite joint target");
    return goal;
  }
  void feedbackLoop() {
    while (!feedback_shutdown_) {
      try {
        const auto f=driver_->readFeedback();
        const auto begin=Clock::now();
        try { state_->update(f); }
        catch (...) {driver_->noteFeedbackDispatch(begin,Clock::now());throw;}
        driver_->noteFeedbackDispatch(begin,Clock::now());
      } catch (const QueryFailure &e) {
        state_->invalidate(std::string(e.what()) +
                           " partial=" + e.partial_reply);
      } catch (const std::exception &e) {
        state_->invalidate(e.what());
      }
      // Bounded backoff also avoids a reconnect spin on refused localhost
      // sockets.
      if (!state_->snapshot().fresh) {
        const auto begin=Clock::now();
        for (int i = 0; i < 10 && !feedback_shutdown_; ++i)
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        driver_->noteFeedbackBackoff(begin,Clock::now());
      }
    }
  }
  template<class Result> void fillFinal(Result &r) {
    auto s = state_->snapshot();
    r.final_tcp_valid = s.fresh;
    if (s.fresh)
      r.final_tcp = poseMessage(model_->tcp(s.feedback.q), rclcpp::Time(s.feedback.ros_received_ns, get_clock()->get_clock_type()));
  }
  trajectory_msgs::msg::JointTrajectory trajectory(const Plan &plan) {
    trajectory_msgs::msg::JointTrajectory t;
    t.header.frame_id = "base_link";
    t.header.stamp = now();
    t.joint_names.assign(model_->jointNames().begin(),
                         model_->jointNames().end());
    double offset = 0;
    // Gripper waiting time is not a timed trajectory segment. This trajectory
    // is informational; GRASP_SEQUENCE execution always enforces the
    // confirmation gate.
    for (const auto &s : plan.segments) {
      for (size_t i = 0; i < s.samples.size(); ++i) {
        if (!t.points.empty() && i == 0)
          continue;
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions.assign(s.samples[i].q.begin(), s.samples[i].q.end());
        p.time_from_start =
            rclcpp::Duration::from_seconds(offset + s.samples[i].time);
        t.points.push_back(p);
      }
      offset += s.samples.back().time;
    }
    return t;
  }
  template<class Action> void process(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> &h) {
    auto result = std::make_shared<typename Action::Result>();
    bool success = false;
    const auto effective=config_.forProfile(h->get_goal()->profile);
    auto safety=safetyCheck(h->get_goal()->profile);
    Executor executor(effective, *driver_, *state_, gate_, model_, safety);
    try {
      if (shutdown_ || gate_.canceled())
        throw Failure(CANCELED, "canceled before planning");
      const auto goal = decode(*h->get_goal());
      auto planning_deadline = Clock::now() + std::chrono::duration<double>(
                                                  config_.planning_timeout);
      Planner planner(
          effective, model_, *driver_,
          [this, planning_deadline]() {
            if (Clock::now() > planning_deadline)
              throw Failure(PLANNING_FAILED, "planning deadline exceeded");
            if (shutdown_ || gate_.canceled())
              throw Failure(CANCELED, "planning canceled");
            auto s = state_->snapshot();
            if (!s.fresh)
              throw Failure(FEEDBACK_FAULT, "feedback lost during planning");
            if (!s.reference)
              throw Failure(REFERENCE_INVALID,
                            "reference lost during planning");
          },
          safety);
      auto start = state_->snapshot();
      auto plan = planner.plan(goal, start);
      result->trajectory = trajectory(plan);
      if (gate_.canceled() || shutdown_)
        throw Failure(CANCELED, "canceled after planning");
      if (goal.plan_only) {
        result->code = PLAN_READY;
        result->detail =
            "PLAN_READY: sampled geometric plan only; movement not executed";
        result->detail += plan.selection;
      } else {
        // Reject stale plans if the arm moved during SDK planning.
        if (jointDistance(state_->snapshot().feedback.q, start.feedback.q) >
            config_.joint_tolerance)
          throw Failure(EXECUTION_FAILED, "state moved during planning");
        // Durable write-ahead marker: stale clean save must not survive a motion.
        checkpoint_save_due_=true;
        if (checkpoint_store_) checkpoint_store_->markMotionPending();
        executor.run(plan);
        result->code = COMPLETED;
        result->executed = true;
        result->detail = "measured arrival verified";
        result->detail += plan.selection;
      }
      fillFinal(*result);
      success = true;
    } catch (const std::exception &e) {
      auto failure = dynamic_cast<const Failure *>(&e);
      result->code =
          failure ? failure->code
                  : (executor.attempted() ? EXECUTION_FAILED : PLANNING_FAILED);
      result->detail = e.what();
      // executed means motion was attempted, including ambiguous send/ACK
      // outcomes.
      result->executed = executor.attempted();
      try {
        executor.stopOwned();
      } catch (const std::exception &stop) {
        result->code = STOP_FAILED;
        result->detail += "; ";
        result->detail += stop.what();
        gate_.latchFault(result->detail);
      }
      try {
        fillFinal(*result);
      } catch (...) {
        result->final_tcp_valid = false;
      }
    }
    // ROS terminal transitions are dispatched by the same mutually exclusive
    // callback group as cancellation. The framework has applied CANCELING by
    // then; the worker never races the cancel callback's return boundary.
    std::lock_guard<std::mutex> lock(queue_mutex_);
    terminal_ = [this,h,result,success] { finishResult(h,result,success); };
  }
  template<class Action> void finishResult(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> &h,
      const std::shared_ptr<typename Action::Result> &result, bool success) {
    const bool canceled=gate_.commitTerminal();
    try {
      if(h->is_canceling()) {
        if(success) {result->code=CANCELED;result->detail="canceled at completion boundary";}
        h->canceled(result);
      } else if(success && !canceled && !shutdown_) h->succeed(result);
      else {
        if(success) {result->code=CANCELED;result->detail="shutdown before result dispatch";}
        h->abort(result);
      }
    } catch(const std::exception &e) {
      gate_.latchFault(std::string("ROS terminal transition failed: ")+e.what());
      RCLCPP_ERROR(get_logger(),"ROS terminal transition failed: %s",e.what());
    }
    gate_.release();
  }
  void finishPending() {
    std::function<void()> terminal;
    {std::lock_guard<std::mutex> lock(queue_mutex_); terminal=std::move(terminal_); terminal_={};}
    if(terminal) terminal();
  }
  void workerLoop() {
    for (;;) {
      std::function<void()> work;
      {
        std::unique_lock<std::mutex> l(queue_mutex_);
        queue_cv_.wait(l, [this] { return shutdown_ || pending_; });
        if (!pending_ && shutdown_)
          break;
        work = std::move(pending_); pending_={};
      }
      try {
        work();
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "action terminal transition failed: %s",
                     e.what());
        gate_.release();
      }
    }
  }
  void publish() {
    auto s = state_->snapshot();
    auto v = gate_.view();
    chess_robot_interfaces::msg::ArmState msg;
    msg.header.stamp = rclcpp::Time(s.feedback.ros_received_ns, get_clock()->get_clock_type());
    msg.header.frame_id = "base_link";
    msg.backend = config_.backend;
    msg.phase = v.phase;
    msg.job_id = v.job;
    msg.busy = v.busy;
    msg.feedback_fresh = s.fresh;
    msg.reference_valid = s.reference;
    msg.hardware_certified = config_.backend == "hardware" &&
                             config_.hardwareAllowed() && !config_.test_fixture;
    msg.simulated = config_.test_fixture;
    msg.controller_mode = s.feedback.mode;
    msg.diagnostic = s.diagnostic + (v.detail.empty() ? "" : "; " + v.detail) +
        (checkpoint_status_.empty() ? "" : "; checkpoint="+
          (v.busy ? std::string("task active; wait for stopped save") : checkpoint_status_))+(checkpoint_enable_status_.empty() ? "" : "; actuator="+checkpoint_enable_status_)+driver_->auditStatus();
    if (s.fresh) {
      msg.tcp = poseMessage(model_->tcp(s.feedback.q), rclcpp::Time(s.feedback.ros_received_ns, get_clock()->get_clock_type()));
      sensor_msgs::msg::JointState j;
      j.header = msg.header;
      j.name.assign(model_->jointNames().begin(), model_->jointNames().end());
      j.position.assign(s.feedback.q.begin(), s.feedback.q.end());
      j.velocity.assign(s.feedback.speed.begin(), s.feedback.speed.end());
      joints_pub_->publish(j);
    }
    if (s.reference) {
      auto m = s.tracker.metrics();
      msg.signed_twist_rad = m.net;
      msg.total_twist_rad = m.total;
      msg.twist_energy_rad2_per_m = m.energy;
    }
    state_pub_->publish(msg);
    std::shared_ptr<Handle> h;
    std::shared_ptr<JointHandle> jh;
    {std::lock_guard<std::mutex> l(queue_mutex_);h=active_.lock();jh=joint_active_.lock();}
    publishActionFeedback<Move>(h,msg,s.fresh);
    publishActionFeedback<JointMove>(jh,msg,s.fresh);
  }
  template<class Action> void publishActionFeedback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> &h,
      const chess_robot_interfaces::msg::ArmState &msg,bool fresh) {
    if(!h || !h->is_active()) return;
    auto f=std::make_shared<typename Action::Feedback>();
    f->phase=msg.phase;f->job_id=msg.job_id;f->tcp=msg.tcp;f->tcp_valid=fresh;
    f->reference_valid=msg.reference_valid;f->signed_twist_rad=msg.signed_twist_rad;f->total_twist_rad=msg.total_twist_rad;
    h->publish_feedback(f);
  }
  std::weak_ptr<Handle> active_;
  std::weak_ptr<JointHandle> joint_active_;
  bool enableRecoveredHistory(bool attested,double stopped_dwell,std::string &detail) {
    // Legacy opt-in remains explicit; never automatically retry an actuator attempt.
    checkpoint_auto_inhibited_=true;
    if(!gate_.reserve("checkpoint-enable")) {
      checkpoint_enable_status_="enable skipped: task admission busy/faulted; history restored";
      detail+="; "+checkpoint_enable_status_;return true;
    }
    std::unique_ptr<TaskGate,std::function<void(TaskGate*)>> admission(&gate_,[](TaskGate *g){g->release();});
    const auto before=state_->snapshot();
    std::string failure;
    try {
      state_->validateRecovery(before.reference_generation,false,0);
      if(before.feedback.mode==4) {
        driver_->enableAfterHistoryRestore(attested,[this,generation=before.reference_generation,stopped_dwell]{
          state_->validateRecovery(generation,true,stopped_dwell);
        });
      }
    } catch(const std::exception &e) {failure=e.what();}
    try {
      state_->confirmRecoveryEndpoint(before.reference_generation,before.feedback,Clock::now(),config_.freshness);
    } catch(const std::exception &e) {
      checkpoint_enable_status_="enable/confirmation failed: "+failure+"; "+e.what();
      detail="history committed but subsequent feedback confirmation invalidated it; "+checkpoint_enable_status_;
      return false;
    }
    if(!failure.empty()) {
      checkpoint_enable_status_="enable failed; history retained by independent fresh stopped endpoint: "+failure;
    } else if(state_->snapshot().feedback.mode==5) {
      checkpoint_enable_status_="enabled; independently confirmed; no motion sent";
    } else {
      checkpoint_enable_pending_=true;
      checkpoint_enable_deadline_=Clock::now()+std::chrono::milliseconds(config_.enable_ack_timeout_ms);
      checkpoint_enable_status_="EnableRobot acknowledged; awaiting fresh mode5; history valid";
    }
    detail+="; "+checkpoint_enable_status_;
    return true;
  }
  bool recoverCheckpoint(bool attested,std::string &detail) {
    if(!checkpoint_store_) {detail="checkpoint storage is disabled";return false;}
    try {
      auto cp=checkpoint_store_->load();
      if(!gate_.restoreCheckpoint(*state_,cp,attested,detail)) return false;
    } catch(const std::exception &e) {detail=std::string("history load/restore failed: ")+e.what();return false;}
    checkpoint_save_due_=true;
    checkpoint_had_reference_=true;
    checkpoint_enable_status_="not requested; history alone grants no motion readiness";
    const bool valid=!config_.checkpoint_enable_after_restore || enableRecoveredHistory(attested,config_.checkpoint_dwell,detail);
    checkpoint_status_=detail;return valid;
  }
  void checkpointTick() {
    auto s=state_->snapshot();
    if(checkpoint_enable_pending_ && s.fresh && s.reference && s.feedback.mode==5) {
      checkpoint_enable_pending_=false;checkpoint_enable_status_="enabled; fresh mode5 confirmed; no motion sent";
    }
    const auto now=Clock::now();
    if(checkpoint_enable_pending_ && (!s.reference || now>=checkpoint_enable_deadline_)) {
      checkpoint_enable_pending_=false;
      checkpoint_enable_status_=s.reference ? "enable confirmation timed out; stopped history remains valid; no automatic retry" : "enable confirmation lost; history invalid";
    }
    if(!checkpoint_store_)return;
    auto v=gate_.view();
    if(v.busy)return;
    const bool measured_motion=s.fresh && (s.feedback.mode==7 ||
        std::any_of(s.feedback.speed.begin(),s.feedback.speed.end(),[this](double x){return std::abs(x)>config_.stationary_speed;}));
    if(checkpoint_had_reference_ && !s.reference) checkpoint_auto_inhibited_=true;
    checkpoint_had_reference_=s.reference;
    if(measured_motion) {
      checkpoint_save_due_=true;
      if(!checkpoint_motion_observed_) {
        try {checkpoint_store_->markMotionPending();checkpoint_motion_observed_=true;}
        catch(const std::exception &e) {
          checkpoint_auto_inhibited_=true;state_->invalidate("observed motion could not be durably marked");
          checkpoint_status_=std::string("cannot mark observed motion: ")+e.what();return;
        }
      }
      checkpoint_status_="observed motion; wait for stopped save";return;
    }
    if(!measured_motion)checkpoint_motion_observed_=false;
    if(!s.reference && config_.checkpoint_auto_restore && !checkpoint_auto_inhibited_ &&
        now-checkpoint_attempt_>=std::chrono::seconds(1)) {
      checkpoint_attempt_=now;std::string detail;
      if(!recoverCheckpoint(config_.checkpoint_no_unobserved_motion,detail) && detail!=checkpoint_status_) {
        checkpoint_status_=detail;RCLCPP_WARN(get_logger(),"checkpoint recovery: %s",detail.c_str());
      }
      return;
    }
    if(s.reference && (checkpoint_save_due_ || now-checkpoint_saved_>=std::chrono::duration<double>(config_.checkpoint_period))) {
      // Failed dwell/admission is expected while stopping; no persistent write yet.
      std::optional<Snapshot> stopped;
      try {stopped=gate_.checkpoint(*state_);} catch(const std::exception &) {
        if(checkpoint_save_due_)checkpoint_status_="waiting for fresh stopped dwell";
        return;
      }
      try {
        gate_.persistCheckpoint(*state_,[this](const Snapshot &current){
          checkpoint_store_->save(current);checkpoint_saved_=Clock::now();checkpoint_save_due_=false;
        });
        checkpoint_status_="durable stopped save ready";
      } catch(const std::exception &e) {
        checkpoint_status_=std::string("save failed: ")+e.what();
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"checkpoint save: %s",e.what());
      }
    }
  }

public:
  ArmNode() : Node("chess_robot_arm") {
    configure();
    model_ = std::make_shared<chess_robot_twist::Model>(
        config_.urdf, "base_link", "Link6", config_.flange_tcp, config_.twist_reference);
    if (!model_->withinJointBounds(config_.twist_reference))
      throw Failure(INVALID_GOAL, "history reference outside URDF bounds");
    if (!model_->withinJointBounds(config_.home))
      throw Failure(INVALID_GOAL, "Home outside URDF bounds");
    if (!config_.safety_model_file.empty())
      geometry_guard_ = std::make_unique<GeometryGuard>(config_, model_);
    if (config_.backend=="exploration")
      for (const auto &b : config_.exploration_boundaries)
        exploration_guards_[b.profile]=std::make_shared<GeometryGuard>(config_.forProfile(b.profile),model_);
    driver_ = std::make_unique<V4Driver>(config_, TransportFactory{},
        [this] { return get_clock()->now().nanoseconds(); });
    state_ = std::make_unique<State>(config_, model_);
    if(!config_.checkpoint_file.empty())checkpoint_store_=std::make_unique<CheckpointStore>(config_);
    state_pub_ = create_publisher<chess_robot_interfaces::msg::ArmState>(
        endpoint("state"), 10);
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        endpoint("joint_states"), 10);
    action_ = rclcpp_action::create_server<Move>(
        this, endpoint("move"),
        [this](const rclcpp_action::GoalUUID &id,
               std::shared_ptr<const Move::Goal> g) {
          try {
            decode(*g);
            if (!g->plan_only && !config_.hardwareAllowed())
              return rclcpp_action::GoalResponse::REJECT;
            if (shutdown_ || !gate_.reserve(idString(id)))
              return rclcpp_action::GoalResponse::REJECT;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "goal rejected: %s", e.what());
            return rclcpp_action::GoalResponse::REJECT;
          }
        },
        [this](std::shared_ptr<Handle> h) {
          return gate_.cancel(idString(h->get_goal_id()))
                     ? rclcpp_action::CancelResponse::ACCEPT
                     : rclcpp_action::CancelResponse::REJECT;
        },
        [this](std::shared_ptr<Handle> h) {
          {
            std::lock_guard<std::mutex> l(queue_mutex_);
            pending_ = [this,h] {process<Move>(h);};
            active_ = h;
          }
          queue_cv_.notify_one();
        });
    joint_action_ = rclcpp_action::create_server<JointMove>(
      this, endpoint("move_joints"),
      [this](const rclcpp_action::GoalUUID &id,std::shared_ptr<const JointMove::Goal> g) {
        try {
          decode(*g);
          if(!g->plan_only && !config_.hardwareAllowed()) return rclcpp_action::GoalResponse::REJECT;
          if(shutdown_ || !gate_.reserve(idString(id))) return rclcpp_action::GoalResponse::REJECT;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } catch(const std::exception &e) {
          RCLCPP_WARN(get_logger(),"joint goal rejected: %s",e.what());
          return rclcpp_action::GoalResponse::REJECT;
        }
      },
      [this](std::shared_ptr<JointHandle> h) {
        return gate_.cancel(idString(h->get_goal_id())) ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
      },
      [this](std::shared_ptr<JointHandle> h) {
        {std::lock_guard<std::mutex> l(queue_mutex_);pending_=[this,h]{process<JointMove>(h);};joint_active_=h;}
        queue_cv_.notify_one();
      });
    confirm_ = create_service<chess_robot_interfaces::srv::ConfirmGrasp>(
        endpoint("confirm_grasp"),
        [this](
            const std::shared_ptr<
                chess_robot_interfaces::srv::ConfirmGrasp::Request>
                req,
            std::shared_ptr<chess_robot_interfaces::srv::ConfirmGrasp::Response>
                res) {
          res->accepted = gate_.confirm(req->job_id, req->success, req->detail);
          res->detail = res->accepted
                            ? "confirmation accepted"
                            : "wrong job, phase, duplicate or canceled";
        });
    initialize_ =
        create_service<chess_robot_interfaces::srv::InitializeReference>(
            endpoint("initialize_reference"),
            [this](
                const std::shared_ptr<
                    chess_robot_interfaces::srv::InitializeReference::Request>,
                std::shared_ptr<
                    chess_robot_interfaces::srv::InitializeReference::Response>
                    res) {
              res->success = gate_.initialize(*state_, res->detail);
            });
    restore_=create_service<chess_robot_interfaces::srv::RestoreReference>(
        endpoint("restore_reference"),
        [this](const std::shared_ptr<chess_robot_interfaces::srv::RestoreReference::Request> req,
               std::shared_ptr<chess_robot_interfaces::srv::RestoreReference::Response> res) {
          res->success=gate_.restore(*state_,req->audit_files,req->no_unobserved_motion,res->detail);
          if (res->success && config_.enable_after_history_restore)
            res->success=enableRecoveredHistory(false,config_.dwell,res->detail);
        });
    restore_checkpoint_=create_service<chess_robot_interfaces::srv::RestoreCheckpoint>(
        endpoint("restore_checkpoint"),[this](
            const std::shared_ptr<chess_robot_interfaces::srv::RestoreCheckpoint::Request> req,
            std::shared_ptr<chess_robot_interfaces::srv::RestoreCheckpoint::Response> res) {
          res->success=recoverCheckpoint(req->no_unobserved_motion,res->detail);
          if(res->success && !config_.checkpoint_enable_after_restore)checkpoint_auto_inhibited_=false;
        });
    parameter_lock_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &) {
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = false;
          r.reason = "restart required for validated immutable configuration";
          return r;
        });
    // Keep checkpoint/admission maintenance at its existing 20Hz cadence.
    // Faster telemetry never renews the underlying feedback timestamp and
    // never changes controller sampling, trajectory timing or safety ages.
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{finishPending();checkpointTick();});
    publish_timer_=create_wall_timer(std::chrono::nanoseconds(
        static_cast<int64_t>(std::llround(1e9/state_publish_hz_))),[this]{publish();});
    feedback_thread_ = std::thread([this] { feedbackLoop(); });
    worker_thread_ = std::thread([this] { workerLoop(); });
    RCLCPP_INFO(get_logger(),
                "backend=%s, actuation gate=%s; checkpoint recovery=%s; no automatic motion",
                config_.backend.c_str(),
                config_.hardwareAllowed()
                    ? ((config_.backend == "commissioning" || config_.backend == "exploration")
                           ? "bounded commissioning, uncertified"
                           : "production attestations")
                    : "disabled",config_.checkpoint_auto_restore ? "opted in" : "manual");
  }
  ~ArmNode() override {
    shutdown_ = true;
    auto v = gate_.view();
    if (v.busy)
      gate_.cancel(v.job);
    queue_cv_.notify_all();
    if (worker_thread_.joinable())
      worker_thread_.join();
    finishPending();
    feedback_shutdown_ = true;
    if (feedback_thread_.joinable())
      feedback_thread_.join();
  }
};
} // namespace chess_robot_arm
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<chess_robot_arm::ArmNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    fprintf(stderr, "arm startup failure: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
