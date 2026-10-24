#include "chess_robot_arm/core.hpp"
#include "chess_robot_arm/driver.hpp"
#include "chess_robot_arm/net.hpp"
#include <chess_robot_interfaces/action/move_arm.hpp>
#include <chess_robot_interfaces/msg/arm_state.hpp>
#include <chess_robot_interfaces/srv/confirm_grasp.hpp>
#include <chess_robot_interfaces/srv/initialize_reference.hpp>
#include <condition_variable>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <thread>

namespace chess_robot_arm {
using Move = chess_robot_interfaces::action::MoveArm;
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
  TaskGate gate_;
  std::atomic<bool> shutdown_{false}, feedback_shutdown_{false};
  std::thread feedback_thread_, worker_thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::shared_ptr<Handle> pending_;
  struct Terminal {
    std::shared_ptr<Handle> handle;
    std::shared_ptr<Move::Result> result;
    bool success;
  };
  std::optional<Terminal> terminal_;
  rclcpp_action::Server<Move>::SharedPtr action_;
  rclcpp::Service<chess_robot_interfaces::srv::ConfirmGrasp>::SharedPtr
      confirm_;
  rclcpp::Service<chess_robot_interfaces::srv::InitializeReference>::SharedPtr
      initialize_;
  rclcpp::Publisher<chess_robot_interfaces::msg::ArmState>::SharedPtr
      state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
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
    c.backend = param("backend", c.backend);
    c.ip = param("controller.ip", c.ip);
    c.test_fixture = param("test_fixture", false);
    c.urdf = param("urdf_file", std::string(""));
    c.dashboard_port = param("controller.dashboard_port", c.dashboard_port);
    c.feedback_port = param("controller.feedback_port", c.feedback_port);
    c.timeout_ms = param("controller.timeout_ms", c.timeout_ms);
    auto h = array("home.joint_positions_rad",
                   std::vector<double>(c.home.begin(), c.home.end()), 6);
    std::copy(h.begin(), h.end(), c.home.begin());
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
    PARAM(max_tracking_error, "tolerances.tracking_rad");
    PARAM(cost_duration, "twist.cost_duration");
    PARAM(cost_net, "twist.cost_net");
    PARAM(cost_total, "twist.cost_total");
    PARAM(cost_energy, "twist.cost_energy");
    PARAM(twist_limits.net_limit_rad, "twist.net_limit_rad");
    PARAM(twist_limits.total_limit_rad, "twist.total_limit_rad");
    PARAM(tracker_options.max_joint_step_rad, "reference.max_joint_step_rad");
    PARAM(tracker_options.max_phase_step_rad, "reference.max_phase_step_rad");
    PARAM(enable, "hardware.enable");
    PARAM(tcp_verified, "hardware.tcp_verified");
    PARAM(limits_verified, "hardware.limits_verified");
    PARAM(execution_verified, "hardware.execution_profile_verified");
    PARAM(servo_certified, "hardware.servoj_certified");
#undef PARAM
    int64_t count = param<int64_t>("sampling.max_samples", 20000);
    if (count < 2 || count > 100000)
      throw Failure(INVALID_GOAL, "sample budget out of range");
    c.max_samples = static_cast<size_t>(count);
    auto profile_names =
        param("profiles.names", std::vector<std::string>{"conservative"});
    c.profiles.clear();
    for (const auto &n : profile_names)
      c.profiles.push_back(
          {n, param("profiles." + n + ".speed_rad_s", 0.25),
           param("profiles." + n + ".acceleration_rad_s2", 0.5)});
    c.validate();
  }
  Goal decode(const Move::Goal &g) {
    if (g.operation > Move::Goal::HOME)
      throw Failure(INVALID_GOAL, "unknown operation");
    Goal goal;
    goal.operation = static_cast<Operation>(g.operation);
    goal.profile = g.profile;
    goal.plan_only = g.plan_only;
    config_.profile(g.profile);
    if (goal.operation != Operation::Home) {
      validateCapture(g.target.header.frame_id,
                      rclcpp::Time(g.target.header.stamp).seconds(),
                      now().seconds(), config_);
      goal.target = poseTransform(g.target.pose);
      validatePose(goal.target, config_);
    }
    return goal;
  }
  void feedbackLoop() {
    while (!feedback_shutdown_) {
      try {
        state_->update(driver_->readFeedback());
      } catch (const QueryFailure &e) {
        state_->invalidate(std::string(e.what()) +
                           " partial=" + e.partial_reply);
      } catch (const std::exception &e) {
        state_->invalidate(e.what());
      }
      // Bounded backoff also avoids a reconnect spin on refused localhost
      // sockets.
      if (!state_->snapshot().fresh)
        for (int i = 0; i < 10 && !feedback_shutdown_; ++i)
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  void fillFinal(Move::Result &r) {
    auto s = state_->snapshot();
    r.final_tcp_valid = s.fresh;
    if (s.fresh)
      r.final_tcp = poseMessage(model_->tcp(s.feedback.q), now());
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
  void process(const std::shared_ptr<Handle> &h) {
    auto result = std::make_shared<Move::Result>();
    bool success = false;
    Executor executor(config_, *driver_, *state_, gate_, model_);
    try {
      if (shutdown_ || gate_.canceled())
        throw Failure(CANCELED, "canceled before planning");
      const auto goal = decode(*h->get_goal());
      auto planning_deadline = Clock::now() + std::chrono::duration<double>(
                                                  config_.planning_timeout);
      Planner planner(config_, model_, *driver_, [this, planning_deadline]() {
        if (Clock::now() > planning_deadline)
          throw Failure(PLANNING_FAILED, "planning deadline exceeded");
        if (shutdown_ || gate_.canceled())
          throw Failure(CANCELED, "planning canceled");
        auto s = state_->snapshot();
        if (!s.fresh)
          throw Failure(FEEDBACK_FAULT, "feedback lost during planning");
        if (!s.reference)
          throw Failure(REFERENCE_INVALID, "reference lost during planning");
      });
      auto start = state_->snapshot();
      auto plan = planner.plan(goal, start);
      result->trajectory = trajectory(plan);
      if (gate_.canceled() || shutdown_)
        throw Failure(CANCELED, "canceled after planning");
      if (goal.plan_only) {
        result->code = PLAN_READY;
        result->detail =
            "PLAN_READY: sampled geometric plan only; movement not executed";
      } else {
        // Reject stale plans if the arm moved during SDK planning.
        if (jointDistance(state_->snapshot().feedback.q, start.feedback.q) >
            config_.joint_tolerance)
          throw Failure(EXECUTION_FAILED, "state moved during planning");
        executor.run(plan);
        result->code = COMPLETED;
        result->executed = true;
        result->detail = "measured arrival verified";
      }
      fillFinal(*result);
      success = true;
    } catch (const std::exception &e) {
      auto failure = dynamic_cast<const Failure *>(&e);
      result->code = failure ? failure->code : PLANNING_FAILED;
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
    terminal_ = Terminal{h, result, success};
  }
  void finishPending() {
    std::optional<Terminal> terminal;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      terminal = std::move(terminal_);
      terminal_.reset();
    }
    if (!terminal)
      return;
    const bool canceled = gate_.commitTerminal();
    auto &h = terminal->handle;
    auto &result = terminal->result;
    try {
      if (h->is_canceling()) {
        if (terminal->success) {
          result->code = CANCELED;
          result->detail = "canceled at completion boundary";
        }
        h->canceled(result);
      } else if (terminal->success && !canceled && !shutdown_) {
        h->succeed(result);
      } else {
        if (terminal->success) {
          result->code = CANCELED;
          result->detail = "shutdown before result dispatch";
        }
        h->abort(result);
      }
    } catch (const std::exception &e) {
      gate_.latchFault(std::string("ROS terminal transition failed: ") +
                       e.what());
      RCLCPP_ERROR(get_logger(), "ROS terminal transition failed: %s",
                   e.what());
    }
    gate_.release();
  }
  void workerLoop() {
    for (;;) {
      std::shared_ptr<Handle> h;
      {
        std::unique_lock<std::mutex> l(queue_mutex_);
        queue_cv_.wait(l, [this] { return shutdown_ || pending_; });
        if (!pending_ && shutdown_)
          break;
        h = std::move(pending_);
      }
      try {
        process(h);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "action terminal transition failed: %s",
                     e.what());
        gate_.release();
      }
    }
  }
  void publish() {
    finishPending();
    auto s = state_->snapshot();
    auto v = gate_.view();
    chess_robot_interfaces::msg::ArmState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.backend = config_.backend;
    msg.phase = v.phase;
    msg.job_id = v.job;
    msg.busy = v.busy;
    msg.feedback_fresh = s.fresh;
    msg.reference_valid = s.reference;
    msg.hardware_certified = config_.hardwareAllowed() && !config_.test_fixture;
    msg.simulated = config_.test_fixture;
    msg.controller_mode = s.feedback.mode;
    msg.diagnostic = s.diagnostic + (v.detail.empty() ? "" : "; " + v.detail);
    if (s.fresh) {
      msg.tcp = poseMessage(model_->tcp(s.feedback.q), now());
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
    {
      std::lock_guard<std::mutex> l(queue_mutex_);
      h = active_.lock();
    }
    if (h && h->is_active()) {
      auto f = std::make_shared<Move::Feedback>();
      f->phase = v.phase;
      f->job_id = v.job;
      f->tcp = msg.tcp;
      f->tcp_valid = s.fresh;
      f->reference_valid = s.reference;
      f->signed_twist_rad = msg.signed_twist_rad;
      f->total_twist_rad = msg.total_twist_rad;
      h->publish_feedback(f);
    }
  }
  std::weak_ptr<Handle> active_;

public:
  ArmNode() : Node("chess_robot_arm") {
    configure();
    model_ = std::make_shared<chess_robot_twist::Model>(
        config_.urdf, "base_link", "Link6", config_.flange_tcp, config_.home);
    if (!model_->withinJointBounds(config_.home))
      throw Failure(INVALID_GOAL, "Home outside URDF bounds");
    driver_ = std::make_unique<V4Driver>(config_);
    state_ = std::make_unique<State>(config_, model_);
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
            pending_ = h;
            active_ = h;
          }
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
    parameter_lock_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &) {
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = false;
          r.reason = "restart required for validated immutable configuration";
          return r;
        });
    timer_ =
        create_wall_timer(std::chrono::milliseconds(50), [this] { publish(); });
    feedback_thread_ = std::thread([this] { feedbackLoop(); });
    worker_thread_ = std::thread([this] { workerLoop(); });
    RCLCPP_INFO(
        get_logger(),
        "backend=%s, actuation prerequisites=%s; no automatic enable/reference",
        config_.backend.c_str(),
        config_.hardwareAllowed() ? "verified by configuration" : "false");
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
