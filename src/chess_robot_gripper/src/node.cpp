#include "chess_robot_gripper/core.hpp"
#include <chess_robot_interfaces/action/move_gripper.hpp>
#include <chess_robot_interfaces/msg/gripper_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace cg = chess_robot_gripper;
using Action = chess_robot_interfaces::action::MoveGripper;
using State = chess_robot_interfaces::msg::GripperState;
using Handle = rclcpp_action::ServerGoalHandle<Action>;

class GripperNode final : public rclcpp::Node {
public:
  GripperNode() : Node("gripper_node") {
    auto backend = declare_parameter<std::string>("backend", "mock");
    simulated_ = backend == "mock";
    enabled_ = declare_parameter<bool>("enable_commands", false);
    max_force_ = declare_parameter<int>("max_force_percent", 30);
    tolerance_ = declare_parameter<int>("position_tolerance_raw", 5);
    freshness_ = declare_parameter<double>("feedback_max_age_sec", 1.0);
    auto period = declare_parameter<int>("poll_period_ms", 100);
    auto slave = declare_parameter<int>("slave_id", 1);
    auto timeout = declare_parameter<int>("io_timeout_ms", 150);
    auto host = declare_parameter<std::string>("host", "");
    auto port = declare_parameter<int>("port", 60000);
    auto contact = declare_parameter<int>("mock_contact_raw", -1);
    if (max_force_ < 20 || max_force_ > 100 || tolerance_ < 0 || tolerance_ > 50 ||
        !std::isfinite(freshness_) || freshness_ < .1 || freshness_ > 5 ||
        period < 20 || period > 1000 || slave < 1 || slave > 247)
      throw std::invalid_argument("invalid gripper configuration");
    period_ = std::chrono::milliseconds(period);
    if (backend != "mock" && backend != "tcp_rtu")
      throw std::invalid_argument("backend must be mock or tcp_rtu (raw RTU, NOT Modbus TCP)");
    driver_ = std::make_unique<cg::Driver>(simulated_ ? cg::mock_transport(contact) :
      cg::tcp_transport(host, port, timeout), uint8_t(slave));
    // Parameters are startup-only: avoid a displayed parameter differing from the active driver.
    parameter_guard_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>&) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false; result.reason = "gripper parameters require restart"; return result;
    });
    publisher_ = create_publisher<State>("/gripper/state", 10);
    server_ = rclcpp_action::create_server<Action>(this, "/gripper/command",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
        auto c = command(*goal);
        auto why = cg::validate(c, max_force_);
        auto s = state();
        if (!enabled_ || fault_ || !why.empty() || !s.feedback_valid ||
            s.initialization == 2 || (s.initialization == 1 && s.grip_status == 0) ||
            (c.operation == 1 && s.initialization != 1)) {
          RCLCPP_WARN(get_logger(), "Goal rejected: %s", !why.empty() ? why.c_str() :
            "commands disabled, fault latched, stale feedback, uninitialized or device moving");
          return rclcpp_action::GoalResponse::REJECT;
        }
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<Handle>) {
        // No documented independent AG95 stop register: rejecting cancel is explicit.
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](std::shared_ptr<Handle> handle) {
        { std::lock_guard<std::mutex> lock(mutex_); pending_ = std::move(handle); }
        wake_.notify_one();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { publisher_->publish(state()); });
    worker_ = std::thread([this] { loop(); });
    RCLCPP_INFO(get_logger(), "AG95 backend=%s commands=%s; no auto-initialize, no automatic write retries",
      backend.c_str(), enabled_ ? "enabled" : "disabled");
  }
  ~GripperNode() override {
    stopping_ = true; wake_.notify_all();
    if (worker_.joinable()) worker_.join();
  }
private:
  static cg::Command command(const Action::Goal &g) {
    return {g.operation, g.position_raw, g.force_percent, g.require_contact, g.timeout_sec};
  }
  State state() {
    std::lock_guard<std::mutex> lock(mutex_);
    State s;
    s.header.stamp = now();
    s.simulated = simulated_; s.connected = connected_; s.busy = busy_;
    s.commands_enabled = enabled_ && !fault_;
    auto age = sample_.sampled == cg::Clock::time_point{} ? -1. :
      std::chrono::duration<double>(cg::Clock::now() - sample_.sampled).count();
    s.feedback_age_sec = age;
    s.feedback_valid = connected_ && age >= 0 && age <= freshness_;
    s.initialization = s.feedback_valid ? sample_.initialization : 255;
    s.grip_status = s.feedback_valid ? sample_.status : 255;
    s.position_raw = s.feedback_valid ? sample_.position : 65535;
    s.object_detected = s.feedback_valid && s.initialization == 1 && s.grip_status == 2;
    s.object_dropped = s.feedback_valid && s.initialization == 1 && s.grip_status == 3;
    s.detail = detail_;
    if (fault_) s.detail = "FAULT LATCHED; inspect device before restart. " + s.detail;
    return s;
  }
  cg::Sample poll() {
    auto s = driver_->poll();
    if (std::chrono::duration<double>(cg::Clock::now() - s.sampled).count() > freshness_)
      throw std::runtime_error("complete feedback poll exceeded freshness deadline");
    std::lock_guard<std::mutex> lock(mutex_);
    sample_ = s; connected_ = true;
    return s;
  }
  void detail(const std::string &text) { std::lock_guard<std::mutex> lock(mutex_); detail_ = text; }
  void disconnect(const std::string &text) {
    std::lock_guard<std::mutex> lock(mutex_); connected_ = false; detail_ = text;
  }
  void pause() {
    std::unique_lock<std::mutex> lock(mutex_);
    wake_.wait_for(lock, period_, [this] { return stopping_ || bool(pending_); });
  }
  void execute(const std::shared_ptr<Handle> &handle) {
    auto result = std::make_shared<Action::Result>();
    auto c = command(*handle->get_goal());
    auto outcome = cg::Outcome::Pending;
    std::string explanation;
    try {
      auto before = poll();
      if (before.initialization == 2 || (before.initialization == 1 && before.status == 0))
        throw std::runtime_error("device began moving before command; no write issued");
      if (c.operation == 1 && before.initialization != 1) outcome = cg::Outcome::NotInitialized;
      else if (c.operation == 0 && before.initialization == 1) {
        outcome = cg::Outcome::Initialized;
        explanation = "already initialized; fresh feedback confirmed, no write issued";
      } else {
        cg::Completion completion(c, before, tolerance_);
        auto deadline = cg::Clock::now() + std::chrono::duration<double>(c.timeout);
        result->command_attempted = true; // Includes ambiguous write failures.
        if (c.operation == 0) driver_->initialize(); else driver_->move(c.position, c.force);
        detail("command acknowledged; awaiting device feedback");
        while (!stopping_ && rclcpp::ok() && cg::Clock::now() < deadline) {
          auto sample = poll();
          auto feedback = std::make_shared<Action::Feedback>(); feedback->state = state();
          handle->publish_feedback(feedback);
          if (cg::Clock::now() >= deadline) break;
          outcome = completion.observe(sample);
          if (outcome != cg::Outcome::Pending) break;
          pause();
        }
        if (outcome == cg::Outcome::Pending)
          outcome = stopping_ || !rclcpp::ok() ? cg::Outcome::Shutdown : cg::Outcome::Timeout;
      }
    } catch (const std::exception &e) {
      outcome = cg::Outcome::CommunicationError; explanation = e.what(); disconnect(explanation);
    }
    bool success = outcome == cg::Outcome::Initialized || outcome == cg::Outcome::Reached || outcome == cg::Outcome::Grasped;
    bool uncertain = outcome == cg::Outcome::Timeout || outcome == cg::Outcome::CommunicationError ||
      outcome == cg::Outcome::Shutdown;
    if (uncertain && result->command_attempted) fault_ = true;
    if (explanation.empty()) explanation = success ? "confirmed by fresh device feedback" :
      "command did not meet completion condition; no stop/open/retry was sent";
    if (uncertain && result->command_attempted)
      explanation += "; device may still move or hold an object; inspect before restarting";
    detail(explanation);
    result->code = uint16_t(outcome); result->detail = explanation;
    result->simulated = simulated_; result->final_state = state();
    result->final_state.busy = false;
    try { if (success) handle->succeed(result); else handle->abort(result); }
    catch (const std::exception &e) { RCLCPP_WARN(get_logger(), "result delivery failed: %s", e.what()); }
    busy_ = false;
  }
  void loop() {
    while (!stopping_ && rclcpp::ok()) {
      std::shared_ptr<Handle> handle;
      { std::lock_guard<std::mutex> lock(mutex_); handle.swap(pending_); }
      if (handle) execute(handle);
      else {
        try { poll(); }
        catch (const std::exception &e) { disconnect(e.what()); }
        pause();
      }
    }
    std::shared_ptr<Handle> abandoned;
    { std::lock_guard<std::mutex> lock(mutex_); abandoned.swap(pending_); }
    if (abandoned) {
      auto result = std::make_shared<Action::Result>();
      result->code = uint16_t(cg::Outcome::Shutdown); result->simulated = simulated_;
      result->detail = "node shutdown before write; no command issued"; result->final_state = state();
      try { abandoned->abort(result); } catch (...) {}
    }
  }
  std::unique_ptr<cg::Driver> driver_;
  bool simulated_ = true, enabled_ = false;
  std::atomic<bool> stopping_{false}, busy_{false}, fault_{false};
  int max_force_ = 30, tolerance_ = 5;
  double freshness_ = 1.;
  std::chrono::milliseconds period_{100};
  std::mutex mutex_;
  std::condition_variable wake_;
  cg::Sample sample_;
  bool connected_ = false;
  std::string detail_ = "awaiting first feedback";
  std::shared_ptr<Handle> pending_;
  std::thread worker_;
  rclcpp::Publisher<State>::SharedPtr publisher_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<GripperNode>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("gripper_node"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
