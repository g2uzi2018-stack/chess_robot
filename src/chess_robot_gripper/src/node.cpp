// AG95 ROS 节点：接收夹爪任务、发布状态，由单个工作线程串行操作设备。
// 启动只读反馈；初始化和开合都需要显式的 Action 请求。

#include "chess_robot_gripper/core.hpp"
#include <chess_robot_interface/action/move_gripper.hpp>
#include <chess_robot_interface/msg/gripper_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace cg = chess_robot_gripper;
using Action = chess_robot_interface::action::MoveGripper;
using State = chess_robot_interface::msg::GripperState;
using Handle = rclcpp_action::ServerGoalHandle<Action>;

class GripperNode final : public rclcpp::Node {
public:
  // 读取启动参数，建立 ROS 接口，再启动后台轮询；此处不发送设备动作。
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

    // 力上限须在 20～100%，开度容差在 0～50 原始单位。
    // 反馈有效期须为有限的 0.1～5 s，轮询间隔 20～1000 ms，从站地址 1～247。
    if (max_force_ < 20 || max_force_ > 100 || tolerance_ < 0 || tolerance_ > 50 ||
        !std::isfinite(freshness_) || freshness_ < .1 || freshness_ > 5 ||
        period < 20 || period > 1000 || slave < 1 || slave > 247)
      throw std::invalid_argument("invalid gripper configuration");
    period_ = std::chrono::milliseconds(period);
    // 仅支持模拟设备或 TCP 透传 RTU 帧；不支持标准 Modbus TCP 报文。
    if (backend != "mock" && backend != "tcp_rtu")
      throw std::invalid_argument("backend must be mock or tcp_rtu (raw RTU, NOT Modbus TCP)");
    // 创建通信对象；真实 TCP 连接在第一次读写时才建立。
    driver_ = std::make_unique<cg::Driver>(simulated_ ? cg::mock_transport(contact) :
      cg::tcp_transport(host, port, timeout), uint8_t(slave));

    // 参数只在启动时读取，运行中修改一律拒绝，避免显示值与驱动实际值不同。
    parameter_guard_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>&) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "gripper parameters require restart";
      return result;
    });
    publisher_ = create_publisher<State>("/gripper/state", 10);
    server_ = rclcpp_action::create_server<Action>(this, "/gripper/command",
      // 收到目标：检查参数与设备状态，占用唯一任务名额。
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
        auto c = command(*goal);
        auto why = cg::validate(c, max_force_);  // 空字符串表示请求参数有效。
        auto s = state();
        // 禁止命令、故障已锁定、参数无效或反馈失效时拒绝。
        // 正在初始化、已初始化但仍在移动，或未初始化却请求开合时，也拒绝。
        if (!enabled_ || fault_ || !why.empty() || !s.feedback_valid ||
            s.initialization == 2 || (s.initialization == 1 && s.grip_status == 0) ||
            (c.operation == 1 && s.initialization != 1)) {
          RCLCPP_WARN(get_logger(), "Goal rejected: %s", !why.empty() ? why.c_str() :
            "commands disabled, fault latched, stale feedback, uninitialized or device moving");
          return rclcpp_action::GoalResponse::REJECT;
        }
        bool expected = false;
        // 只有空闲时才能原子地改为忙碌；已有任务则拒绝，不另排队。
        if (!busy_.compare_exchange_strong(expected, true))
          return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<Handle>) {
        // 当前驱动未实现可确认的独立停止指令，因此明确拒绝取消请求。
        return rclcpp_action::CancelResponse::REJECT;
      },
      // 目标已接受：交给工作线程，ROS 回调本身不做设备读写。
      [this](std::shared_ptr<Handle> handle) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pending_ = std::move(handle);
        }
        wake_.notify_one();
      });
    // 状态固定每 100 ms 发布一次；设备读取间隔另由 poll_period_ms 决定。
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { publisher_->publish(state()); });
    worker_ = std::thread([this] { loop(); });
    RCLCPP_INFO(get_logger(), "AG95 backend=%s commands=%s; no auto-initialize, no automatic write retries",
      backend.c_str(), enabled_ ? "enabled" : "disabled");
  }

  // 通知工作线程退出并等待结束；不向夹爪发送停止或张开命令。
  ~GripperNode() override {
    stopping_ = true;
    wake_.notify_all();
    // 工作线程仍可等待时，等它完成退出处理。
    if (worker_.joinable())
      worker_.join();
  }

private:
  // 把 ROS 目标转换为驱动使用的命令结构，不做单位换算。
  static cg::Command command(const Action::Goal &g) {
    return {g.operation, g.position_raw, g.force_percent, g.require_contact, g.timeout_sec};
  }

  // 用缓存反馈组装 ROS 状态；不在这里访问设备。
  State state() {
    std::lock_guard<std::mutex> lock(mutex_);
    State s;
    s.header.stamp = now();  // 发布时刻；设备反馈的年龄单独计算。
    s.simulated = simulated_;
    s.connected = connected_;
    s.busy = busy_;
    s.commands_enabled = enabled_ && !fault_;
    // 尚未完成首次读取时年龄为 -1；之后用单调时钟计算，避免系统校时影响。
    auto age = sample_.sampled == cg::Clock::time_point{} ? -1. :
      std::chrono::duration<double>(cg::Clock::now() - sample_.sampled).count();
    s.feedback_age_sec = age;
    // 最近读取成功、样本未过期且开度有效，才对外提供有效反馈。
    s.feedback_valid = connected_ && age >= 0 && age <= freshness_ && sample_.position <= 1000;
    s.initialization = s.feedback_valid ? sample_.initialization : 255;
    s.grip_status = s.feedback_valid ? sample_.status : 255;
    s.position_raw = s.feedback_valid ? sample_.position : 65535;
    s.object_detected = s.feedback_valid && s.initialization == 1 && s.grip_status == 2;
    s.object_dropped = s.feedback_valid && s.initialization == 1 && s.grip_status == 3;
    s.detail = detail_;
    // 发出命令后结果不确定时，保留故障锁定提示。
    if (fault_)
      s.detail = "FAULT LATCHED; inspect device before restart. " + s.detail;
    return s;
  }

  // 顺序读取一组设备反馈，检查耗时，再更新共享缓存。
  cg::Sample poll() {
    auto s = driver_->poll();
    // 一轮读取本身已超过有效期时，整组反馈作废。
    if (std::chrono::duration<double>(cg::Clock::now() - s.sampled).count() > freshness_)
      throw std::runtime_error("complete feedback poll exceeded freshness deadline");
    std::lock_guard<std::mutex> lock(mutex_);
    sample_ = s;
    connected_ = true;
    return s;
  }

  // 更新对外状态中的说明文字。
  void detail(const std::string &text) {
    std::lock_guard<std::mutex> lock(mutex_);
    detail_ = text;
  }

  // 标记本次反馈读取失败，并记录原因；后续仍可继续只读轮询。
  void disconnect(const std::string &text) {
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;
    detail_ = text;
  }

  // 等待下一轮读取；收到任务或退出通知时提前醒来。
  void pause() {
    std::unique_lock<std::mutex> lock(mutex_);
    wake_.wait_for(lock, period_, [this] { return stopping_ || bool(pending_); });
  }

  // 执行一个已接受的任务：先读状态，再按需写命令，最后靠反馈判断完成。
  void execute(const std::shared_ptr<Handle> &handle) {
    auto result = std::make_shared<Action::Result>();
    auto c = command(*handle->get_goal());
    auto outcome = cg::Outcome::Pending;
    std::string explanation;
    try {
      auto before = poll();
      // 接受目标后设备又开始初始化或移动时，终止任务，不发送写命令。
      if (before.initialization == 2 || (before.initialization == 1 && before.status == 0))
        throw std::runtime_error("device began moving before command; no write issued");
      // 执行前已变成未初始化状态时，开合任务直接失败。
      if (c.operation == 1 && before.initialization != 1)
        outcome = cg::Outcome::NotInitialized;
      // 已初始化时，请求初始化只做一次新鲜读确认，不重复回零。
      else if (c.operation == 0 && before.initialization == 1) {
        outcome = cg::Outcome::Initialized;
        explanation = "already initialized; fresh feedback confirmed, no write issued";
      } else {
        // 保存命令前状态，用于识别新动作，避免把旧的夹持状态当成本次完成。
        cg::Completion completion(c, before, tolerance_);
        // 任务计时从发命令前开始，覆盖写入与后续等待，不包含上面的预读。
        auto deadline = cg::Clock::now() + std::chrono::duration<double>(c.timeout);
        result->command_attempted = true;  // 后续异常时，也可能已有写命令到达设备。
        // 初始化写回零请求；开合先写力设定，再写目标开度。
        if (c.operation == 0)
          driver_->initialize();
        else
          driver_->move(c.position, c.force);
        detail("command acknowledged; awaiting device feedback");

        // 节点仍运行且任务未超时时，持续读取并发布任务进度。
        while (!stopping_ && rclcpp::ok() && cg::Clock::now() < deadline) {
          auto sample = poll();
          auto feedback = std::make_shared<Action::Feedback>();
          feedback->state = state();
          handle->publish_feedback(feedback);
          // 读取期间已超过任务截止时间时，不再把这轮反馈判为完成。
          if (cg::Clock::now() >= deadline)
            break;
          outcome = completion.observe(sample);  // 连续两次相同判定才确认结果。
          // 已得到成功或明确失败结果时，结束等待。
          if (outcome != cg::Outcome::Pending)
            break;
          pause();
        }
        // 仍未得到明确结果时，区分节点关闭与任务超时。
        if (outcome == cg::Outcome::Pending)
          outcome = stopping_ || !rclcpp::ok() ? cg::Outcome::Shutdown : cg::Outcome::Timeout;
      }
    } catch (const std::exception &e) {
      outcome = cg::Outcome::CommunicationError;
      explanation = e.what();
      disconnect(explanation);
    }
    // 初始化完成、到位和夹持成功属于成功；其余结果均中止 Action。
    bool success = outcome == cg::Outcome::Initialized || outcome == cg::Outcome::Reached || outcome == cg::Outcome::Grasped;
    bool uncertain = outcome == cg::Outcome::Timeout || outcome == cg::Outcome::CommunicationError ||
      outcome == cg::Outcome::Shutdown;
    // 可能发过写命令，却因超时、通信异常或退出失去结果时，锁定后续命令。
    if (uncertain && result->command_attempted)
      fault_ = true;
    // 没有具体原因时，按成功或失败补充说明。
    if (explanation.empty())
      explanation = success ? "confirmed by fresh device feedback" :
        "command did not meet completion condition; no stop/open/retry was sent";
    // 同样的结果不确定条件下，提醒设备可能仍在运动或夹持。
    if (uncertain && result->command_attempted)
      explanation += "; device may still move or hold an object; inspect before restarting";
    detail(explanation);
    result->code = uint16_t(outcome);
    result->detail = explanation;
    result->simulated = simulated_;
    result->final_state = state();
    result->final_state.busy = false;
    try {
      // 根据设备反馈得出的最终结果，通知客户端成功或失败。
      if (success)
        handle->succeed(result);
      else
        handle->abort(result);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "result delivery failed: %s", e.what());
    }
    busy_ = false;
  }

  // 唯一设备工作线程：有任务就执行，空闲时只读状态，不自动初始化。
  void loop() {
    while (!stopping_ && rclcpp::ok()) {
      std::shared_ptr<Handle> handle;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        handle.swap(pending_);
      }
      // 已有待执行任务时，接管并执行；否则更新空闲状态。
      if (handle)
        execute(handle);
      else {
        try {
          poll();
        } catch (const std::exception &e) {
          disconnect(e.what());
        }
        pause();
      }
    }
    std::shared_ptr<Handle> abandoned;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      abandoned.swap(pending_);
    }
    // 退出时仍有已接受但未开始的任务，返回关闭结果，不向设备写入。
    if (abandoned) {
      auto result = std::make_shared<Action::Result>();
      result->code = uint16_t(cg::Outcome::Shutdown);
      result->simulated = simulated_;
      result->detail = "node shutdown before write; no command issued";
      result->final_state = state();
      try {
        abandoned->abort(result);
      } catch (...) {}
    }
  }

  std::unique_ptr<cg::Driver> driver_;
  bool simulated_ = true, enabled_ = false;
  // stopping_ 控制线程退出，busy_ 限制单任务，fault_ 锁定结果不确定后的命令。
  std::atomic<bool> stopping_{false}, busy_{false}, fault_{false};
  int max_force_ = 30, tolerance_ = 5;
  double freshness_ = 1.;
  std::chrono::milliseconds period_{100};
  // ROS 回调与工作线程共享以下缓存，通过 mutex_ 保护。
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

// 初始化 ROS，运行节点；构造或运行异常时记录原因并以失败码退出。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GripperNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("gripper_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
