#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chess_robot_interfaces/action/move_gripper.hpp>
#include <chess_robot_interfaces/action/move_joints.hpp>
#include <chess_robot_interfaces/msg/arm_state.hpp>
#include <chess_robot_interfaces/msg/gripper_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <array>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <map>
#include <random>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = net::ip::tcp;
using Clock = std::chrono::steady_clock;
using GripperState = chess_robot_interfaces::msg::GripperState;
using GripperAction = chess_robot_interfaces::action::MoveGripper;
using GripperHandle = rclcpp_action::ClientGoalHandle<GripperAction>;
using ArmState = chess_robot_interfaces::msg::ArmState;
using JointState = sensor_msgs::msg::JointState;
using MoveJoints = chess_robot_interfaces::action::MoveJoints;
using ArmHandle = rclcpp_action::ClientGoalHandle<MoveJoints>;
using Request = http::request<http::string_body>;
using Response = http::response<http::string_body>;

static std::string quote(const std::string &s) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else out << c;
  }
  return out.str() + '"';
}

static const char *boolean(bool v) { return v ? "true" : "false"; }

static std::string nonce() {
  std::random_device rd;
  std::ostringstream s;
  for (int i = 0; i < 4; ++i) s << std::hex << std::setw(8) << std::setfill('0') << rd();
  return s.str();
}

class Panel final : public rclcpp::Node {
public:
  Panel() : Node("panel_node"), server_id_(nonce()) {
    bind_ = declare_parameter<std::string>("bind_address", "127.0.0.1");
    port_ = declare_parameter<int>("port", 8089);
    enabled_ = declare_parameter<bool>("enable_control", false);
    enable_arm_control_ = declare_parameter<bool>("enable_arm_control", false);
    hardware_ = declare_parameter<bool>("allow_hardware", false);
    arm_profile_ = declare_parameter<std::string>("arm_profile", "roundtrip_low");
    arm_max_delta_deg_ = declare_parameter<double>("arm_max_delta_deg", 10.0);
    arm_timeout_ = declare_parameter<double>("arm_timeout_sec", 10.0);
    force_ = declare_parameter<int>("default_force_percent", 20);
    max_force_ = declare_parameter<int>("max_force_percent", 30);
    open_ = declare_parameter<int>("open_position_raw", 1000);
    close_ = declare_parameter<int>("close_position_raw", 0);
    step_ = declare_parameter<int>("step_raw", 20);
    timeout_ = declare_parameter<double>("timeout_sec", 5.0);
    freshness_ = declare_parameter<double>("feedback_max_age_sec", 1.0);
    web_root_ = ament_index_cpp::get_package_share_directory("chess_robot_panel") + "/web/";

    if (port_ < 1 || port_ > 65535 || max_force_ < 20 || max_force_ > 100 ||
        force_ < 20 || force_ > max_force_ || open_ < 0 || open_ > 1000 ||
        close_ < 0 || close_ > 1000 || step_ < 1 || step_ > 1000 ||
        !std::isfinite(timeout_) || timeout_ < .5 || timeout_ > 60 ||
        !std::isfinite(freshness_) || freshness_ < .1 || freshness_ > 5 ||
        !std::isfinite(arm_max_delta_deg_) || arm_max_delta_deg_ <= 0.0 || arm_max_delta_deg_ > 10.0 ||
        !std::isfinite(arm_timeout_) || arm_timeout_ < .5 || arm_timeout_ > 60 ||
        arm_profile_.empty())
      throw std::invalid_argument("invalid panel startup parameters");

    parameter_guard_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>&) {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = false; r.reason = "panel parameters require restart"; return r;
    });

    client_ = rclcpp_action::create_client<GripperAction>(this, "/gripper/command");
    arm_client_ = rclcpp_action::create_client<MoveJoints>(this, "/arm/move_joints");

    subscription_ = create_subscription<GripperState>("/gripper/state", rclcpp::QoS(1), [this](GripperState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = *msg; received_ = true; received_at_ = Clock::now();
      const auto &stamp = msg->header.stamp;
      const double published = double(stamp.sec) + double(stamp.nanosec) * 1e-9;
      const double delay = now().seconds() - published;
      message_delay_ = published > 0 && std::isfinite(delay) && delay >= -.25 ? std::max(0., delay) : -1.;
    });

    arm_state_sub_ = create_subscription<ArmState>("/arm/state", rclcpp::QoS(20), [this](ArmState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      receive_arm(*msg);
    });

    arm_joints_sub_ = create_subscription<JointState>("/arm/joint_states", rclcpp::QoS(20), [this](JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      receive_joints(*msg);
    });

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_ && phase_ != "unknown" && Clock::now() > deadline_) {
        phase_ = "unknown";
        detail_ = "结果未知；不会自动重试，夹爪未必已停止。请检查实物及驱动。";
      }
      if (arm_active_ && arm_phase_ != "unknown" && Clock::now() > arm_deadline_) {
        arm_phase_ = "unknown";
        arm_detail_ = "结果未知；不会自动重试，机械臂未必已停止。请检查实物及驱动。";
      }
    });
  }

  Response serve(const Request &req) {
    const auto host = std::string(req[http::field::host]);
    const auto authority = [this](std::string address) {
      if (address.find(':') != std::string::npos) address = "[" + address + "]";
      return address + (port_ == 80 ? "" : ":" + std::to_string(port_));
    };
    if (host != authority(bind_) && host != authority("127.0.0.1") &&
        host != authority("localhost") && host != authority("::1"))
      return error(http::status::forbidden, "请求主机不在面板允许列表");
    const auto target = std::string(req.target());
    if (req.method() == http::verb::get && target == "/api/state") {
      std::lock_guard<std::mutex> lock(mutex_);
      return response(http::status::ok, snapshot());
    }
    if (req.method() == http::verb::get && target == "/api/arm/state") {
      std::lock_guard<std::mutex> lock(mutex_);
      return response(http::status::ok, arm_snapshot());
    }
    if (req.method() == http::verb::post &&
        (target == "/api/gripper/move" || target == "/api/gripper/initialize")) {
      // No CORS; a custom header plus JSON disallows ambient cross-origin form commands.
      if (req[http::field::content_type] != "application/json" || req["X-Panel-Request"] != "2")
        return error(http::status::forbidden, "控制界面已更新，请刷新页面；旧版开合按钮已停用");
      if (!req["Sec-Fetch-Site"].empty() && req["Sec-Fetch-Site"] != "same-origin" &&
          req["Sec-Fetch-Site"] != "none")
        return error(http::status::forbidden, "拒绝跨站控制请求");
      if (!req[http::field::origin].empty() &&
          req[http::field::origin] != "http://" + std::string(req[http::field::host]))
        return error(http::status::forbidden, "请求来源不匹配");
      return command(req.body(), target == "/api/gripper/initialize");
    }
    if (req.method() == http::verb::post && target == "/api/arm/joint") {
      if (req[http::field::content_type] != "application/json" || req["X-Panel-Request"] != "2")
        return error(http::status::forbidden, "控制界面已更新，请刷新页面");
      if (!req["Sec-Fetch-Site"].empty() && req["Sec-Fetch-Site"] != "same-origin" &&
          req["Sec-Fetch-Site"] != "none")
        return error(http::status::forbidden, "拒绝跨站控制请求");
      if (!req[http::field::origin].empty() &&
          req[http::field::origin] != "http://" + std::string(req[http::field::host]))
        return error(http::status::forbidden, "请求来源不匹配");
      return arm_command(req.body());
    }
    if (req.method() == http::verb::get) {
      std::string file, mime;
      if (target == "/" || target == "/index.html") { file = "index.html"; mime = "text/html; charset=utf-8"; }
      else if (target == "/app.js") { file = "app.js"; mime = "text/javascript; charset=utf-8"; }
      else if (target == "/style.css") { file = "style.css"; mime = "text/css; charset=utf-8"; }
      else return error(http::status::not_found, "资源不存在");
      std::ifstream in(web_root_ + file, std::ios::binary);
      if (!in) return error(http::status::not_found, "网页文件未安装");
      return response(http::status::ok, std::string(std::istreambuf_iterator<char>(in), {}), mime);
    }
    return error(http::status::method_not_allowed, "不支持的请求");
  }

  const std::string &bind() const { return bind_; }
  int port() const { return port_; }

private:
  Response response(http::status status, const std::string &body,
                    const std::string &mime = "application/json; charset=utf-8") const {
    Response res(status, 11);
    res.set(http::field::content_type, mime);
    res.set(http::field::cache_control, "no-store");
    res.set("X-Content-Type-Options", "nosniff");
    res.set("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'");
    res.keep_alive(false); res.body() = body; res.prepare_payload(); return res;
  }

  Response error(http::status status, const std::string &why) const {
    return response(status, "{\"error\":" + quote(why) + "}");
  }

  double elapsed() const {
    return received_ ? std::chrono::duration<double>(Clock::now() - received_at_).count() : -1.;
  }

  double age() const {
    if (!received_ || message_delay_ < 0 || !std::isfinite(state_.feedback_age_sec) || state_.feedback_age_sec < 0) return -1.;
    return state_.feedback_age_sec + message_delay_ + elapsed();
  }

  bool fresh() const { return received_ && state_.feedback_valid && age() >= 0 && age() <= freshness_; }

  bool parse_joints(const JointState &msg, std::array<double,6> &output_positions, std::array<double,6> &output_velocities) {
    if (msg.header.frame_id != "base_link") return false;
    if (msg.name.size() != 6 || msg.position.size() != 6 || msg.velocity.size() != 6) {
      return false;
    }
    std::array<bool, 6> found{};
    std::array<double, 6> positions{};
    std::array<double, 6> velocities{};
    for (size_t i = 0; i < 6; ++i) {
      const std::string &name = msg.name[i];
      int idx = -1;
      if (name == "joint1") idx = 0;
      else if (name == "joint2") idx = 1;
      else if (name == "joint3") idx = 2;
      else if (name == "joint4") idx = 3;
      else if (name == "joint5") idx = 4;
      else if (name == "joint6") idx = 5;
      else return false;

      if (found[idx]) return false;
      found[idx] = true;

      const double p = msg.position[i];
      const double v = msg.velocity[i];
      if (!std::isfinite(p) || !std::isfinite(v)) return false;

      positions[idx] = p;
      velocities[idx] = v;
    }
    for (int i = 0; i < 6; ++i) {
      if (!found[i]) return false;
    }
    output_positions = positions;
    output_velocities = velocities;
    return true;
  }

  struct ArmFrame { ArmState message; Clock::time_point received; };
  struct JointFrame {
    builtin_interfaces::msg::Time stamp;
    std::array<double,6> positions{}, velocities{};
    Clock::time_point received;
    bool valid = false;
  };
  static int64_t stamp_key(const builtin_interfaces::msg::Time &stamp) {
    return int64_t(stamp.sec)*1000000000LL+stamp.nanosec;
  }
  bool source_stamp_valid(const builtin_interfaces::msg::Time &stamp) {
    if(stamp.sec<0 || stamp.nanosec>=1000000000u || stamp_key(stamp)<=0) return false;
    const double age=now().seconds()-(double(stamp.sec)+double(stamp.nanosec)*1e-9);
    return std::isfinite(age) && age>=0. && age<=1.;
  }
  void invalidate_pair() {
    // Recovery requires a new complete pair; a safe message on one channel
    // cannot revive positions acquired before an unsafe observation.
    have_arm_pair_=false;joints_valid_=false;recovery_required_=true;
    recovery_source_floor_=now().nanoseconds();
    pending_arms_.clear();pending_joints_.clear();
  }
  bool arm_payload_safe(const ArmState &message) const {
    return message.header.frame_id=="base_link" && message.feedback_fresh &&
      message.reference_valid && message.controller_mode==5 && !message.busy;
  }
  static bool velocities_stationary(const std::array<double,6> &speeds) {
    for(double speed:speeds) if(!std::isfinite(speed) || std::abs(speed)>.002) return false;
    return true;
  }
  void pair_frame(int64_t key) {
    auto arm=pending_arms_.find(key);auto joints=pending_joints_.find(key);
    if(arm==pending_arms_.end() || joints==pending_joints_.end() || key<=recovery_source_floor_ || !joints->second.valid ||
       !arm_payload_safe(arm->second.message) || !velocities_stationary(joints->second.velocities) ||
       (have_arm_pair_ && key<=stamp_key(arm_stamp_))) return;
    arm_state_=arm->second.message;arm_stamp_=arm_state_.header.stamp;
    joint_stamp_=joints->second.stamp;
    joint_positions_=joints->second.positions;joint_velocities_=joints->second.velocities;
    arm_received_at_=arm->second.received;joints_received_at_=joints->second.received;
    joints_valid_=true;have_arm_pair_=true;recovery_required_=false;
  }
  void receive_arm(const ArmState &message) {
    const bool had_arm=arm_received_;arm_received_=true;
    const auto key=stamp_key(message.header.stamp);
    latest_arm_timing_valid_=source_stamp_valid(message.header.stamp) && key>latest_arm_key_;
    if(!latest_arm_timing_valid_) { invalidate_pair();return; }
    const bool identity_changed=had_arm && (message.simulated!=latest_arm_state_.simulated || message.backend!=latest_arm_state_.backend);
    latest_arm_key_=key;latest_arm_state_=message;
    if(identity_changed) invalidate_pair();
    if(!arm_payload_safe(message)) { invalidate_pair();return; }
    pending_arms_[key]={message,Clock::now()};pair_frame(key);
    while(pending_arms_.size()>32) pending_arms_.erase(pending_arms_.begin());
  }
  void receive_joints(const JointState &message) {
    joints_received_=true;
    const auto key=stamp_key(message.header.stamp);
    latest_joints_timing_valid_=source_stamp_valid(message.header.stamp) && key>latest_joint_key_;
    if(!latest_joints_timing_valid_) { invalidate_pair();return; }
    latest_joint_key_=key;
    JointFrame frame;frame.stamp=message.header.stamp;frame.received=Clock::now();
    frame.valid=parse_joints(message,frame.positions,frame.velocities);
    latest_joints_valid_=frame.valid;latest_joint_velocities_=frame.velocities;
    if(!frame.valid || !velocities_stationary(frame.velocities)) { invalidate_pair();return; }
    pending_joints_[key]=frame;pair_frame(key);
    while(pending_joints_.size()>32) pending_joints_.erase(pending_joints_.begin());
  }
  bool pair_lag_valid() const {
    // Keep the last genuine exact pair across callback ordering, but fail
    // closed when either stream advances without a pair for more than 100ms.
    return have_arm_pair_ && latest_arm_timing_valid_ && latest_joints_timing_valid_ && latest_joints_valid_ &&
      std::max(latest_arm_key_,latest_joint_key_)-stamp_key(arm_stamp_)<=100000000LL;
  }
  bool arm_stamps_match() const {
    return arm_received_ && joints_received_ && joints_valid_ && pair_lag_valid() &&
           arm_state_.header.frame_id == "base_link" &&
           arm_stamp_.sec == joint_stamp_.sec &&
           arm_stamp_.nanosec == joint_stamp_.nanosec;
  }

  double arm_feedback_age() const {
    if (!arm_received_ || !joints_received_ || !joints_valid_ || !arm_stamps_match()) return -1.0;
    const double published = double(arm_stamp_.sec) + double(arm_stamp_.nanosec) * 1e-9;
    if (published <= 0.0) return -1.0;
    const double current_time = const_cast<Panel*>(this)->now().seconds();
    const double age = current_time - published;
    if (!std::isfinite(age) || age < 0.0) return -1.0;
    const double arm_elapsed = std::chrono::duration<double>(Clock::now() - arm_received_at_).count();
    const double joints_elapsed = std::chrono::duration<double>(Clock::now() - joints_received_at_).count();
    if (arm_elapsed > 1.0 || joints_elapsed > 1.0) return -1.0;
    return std::max(0.0, age);
  }

  bool arm_fresh() const {
    const double age = arm_feedback_age();
    return age >= 0.0 && age <= 1.0;
  }

  bool arm_stationary() const {
    if (!joints_valid_ || !latest_joints_valid_) return false;
    for (int i = 0; i < 6; ++i) {
      if (std::abs(joint_velocities_[i]) > 0.002 || std::abs(latest_joint_velocities_[i]) > 0.002) return false;
    }
    return true;
  }

  std::string arm_blocked() const {
    if (!enabled_) return "面板控制未启用";
    if (!enable_arm_control_) return "面板机械臂控制未启用";
    if (active_) return "夹爪指令执行中，禁止操作机械臂";
    if (received_ && (state_.busy || state_.initialization == 2 || (state_.initialization == 1 && state_.grip_status == 0))) return "夹爪忙碌，禁止操作机械臂";
    if (!arm_received_ || !joints_received_) return "等待机械臂状态及关节反馈";
    if (!latest_arm_timing_valid_ || !latest_joints_timing_valid_) return "机械臂最新反馈时间戳无效或回退";
    if (!latest_joints_valid_) return "机械臂关节数据无效(须含joint1..joint6)";
    if (latest_arm_state_.header.frame_id != "base_link") return "机械臂坐标系无效";
    if (!latest_arm_state_.feedback_fresh) return "机械臂底层反馈未刷新";
    if (!latest_arm_state_.reference_valid) return "机械臂参考原点无效";
    if (!latest_arm_state_.simulated && !hardware_) return "当前面板仅允许模拟设备";
    if (latest_arm_state_.controller_mode != 5) return "机械臂控制器尚未就绪或未停稳";
    if (latest_arm_state_.busy) return "机械臂忙碌";
    for(double speed:latest_joint_velocities_)
      if(std::abs(speed)>.002) return "机械臂未静止(关节速度>0.002rad/s)";
    if (latest_arm_state_.simulated != arm_state_.simulated || latest_arm_state_.backend != arm_state_.backend) return "机械臂数据源已变化，等待同步反馈";
    if (!pair_lag_valid()) return "机械臂状态与关节反馈尚未同步";
    if (!joints_valid_) return "机械臂关节数据无效(须含joint1..joint6)";
    if (arm_stamp_.sec != joint_stamp_.sec || arm_stamp_.nanosec != joint_stamp_.nanosec)
      return "机械臂状态与关节时间戳不匹配";
    const double age = arm_feedback_age();
    if (age < 0.0) return "机械臂时间戳无效或接收超时";
    if (age > 1.0) return "机械臂反馈过期(>1s)";
    if (!arm_state_.feedback_fresh) return "机械臂底层反馈未刷新";
    if (!arm_state_.reference_valid) return "机械臂参考原点无效";
    if (!arm_state_.simulated && !hardware_) return "当前面板仅允许模拟设备";
      if (arm_state_.controller_mode != 5) return "机械臂控制器尚未就绪或未停稳";
    if (!arm_stationary()) return "机械臂未静止(关节速度>0.002rad/s)";
    if (arm_active_) return arm_phase_ == "unknown" ? "上一机械臂指令结果未知，请检查设备" : "机械臂指令执行中";
    if (arm_state_.busy) return "机械臂忙碌";
    if (!arm_client_->action_server_is_ready()) return "机械臂 MoveJoints Action 服务未就绪";
    return "";
  }

  std::string blocked(bool initialize) const {
    if (!enabled_) return "面板控制未启用";
    if (arm_active_) return "机械臂指令执行中，禁止操作夹爪";
    if (arm_received_ && (recovery_required_ || !latest_arm_timing_valid_)) return "机械臂反馈无效，等待新的完整安全反馈";
    if (arm_received_ && (arm_state_.busy || latest_arm_state_.busy)) return "机械臂忙碌，禁止操作夹爪";
    if (!received_) return "等待夹爪状态";
    if (!state_.connected || !fresh()) return "夹爪断线或反馈过期";
    if (!state_.simulated && !hardware_) return "当前面板仅允许模拟设备";
    if (!state_.commands_enabled) return "夹爪驱动禁止命令，请检查配置或故障状态";
    if (active_) return phase_ == "unknown" ? "上一指令结果未知，请检查设备" : "指令执行中";
    if (state_.busy || state_.initialization == 2 ||
        (state_.initialization == 1 && state_.grip_status == 0)) return "夹爪忙碌";
    if (!client_->action_server_is_ready()) return "夹爪 Action 服务未就绪";
    if (!initialize && state_.initialization != 1) return "请先初始化夹爪";
    return "";
  }

  std::string command_json() const {
    return "{\"request_id\":" + quote(request_id_) + ",\"phase\":" + quote(phase_) +
      ",\"detail\":" + quote(detail_) + ",\"code\":" + (code_ < 0 ? "null" : std::to_string(code_)) +
      ",\"command_attempted\":" + boolean(attempted_) + "}";
  }

  std::string arm_command_json() const {
    std::ostringstream o;
    o << "{\"request_id\":" << quote(arm_request_id_)
      << ",\"phase\":" << quote(arm_phase_)
      << ",\"detail\":" << quote(arm_detail_)
      << ",\"code\":" << (arm_code_ < 0 ? "null" : std::to_string(arm_code_))
      << ",\"command_attempted\":" << boolean(arm_attempted_)
      << ",\"joint_index\":" << (arm_joint_index_ < 0 ? "null" : std::to_string(arm_joint_index_))
      << ",\"target_deg\":" << (arm_joint_index_ < 0 ? "null" : std::to_string(arm_target_deg_))
      << "}";
    return o.str();
  }

  std::string arm_json() const {
    std::ostringstream o;
    const bool can_move = arm_blocked().empty();
    const double age = arm_feedback_age();
    const bool stamps_match = arm_stamps_match();
    const bool fresh = arm_fresh();
    const bool stationary = arm_stationary();

    o << "{\"received\":" << boolean(arm_received_ && joints_received_)
      << ",\"fresh\":" << boolean(fresh)
      << ",\"matched\":" << boolean(stamps_match)
      << ",\"joints_valid\":" << boolean(joints_valid_ && latest_joints_valid_)
      << ",\"simulated\":" << boolean(latest_arm_state_.simulated)
      << ",\"reference_valid\":" << boolean(have_arm_pair_ && arm_state_.reference_valid && latest_arm_state_.reference_valid)
      << ",\"feedback_fresh\":" << boolean(have_arm_pair_ && arm_state_.feedback_fresh && latest_arm_state_.feedback_fresh)
      << ",\"controller_mode\":" << latest_arm_state_.controller_mode
      << ",\"idle_stationary\":" << boolean(stationary)
      << ",\"feedback_age_sec\":" << age
      << ",\"busy\":" << boolean(arm_state_.busy || latest_arm_state_.busy)
      << ",\"can_move\":" << boolean(can_move)
      << ",\"action_ready\":" << boolean(arm_client_->action_server_is_ready())
      << ",\"blocked_reason\":" << quote(arm_blocked())
      << ",\"header_stamp\":{\"sec\":" << arm_stamp_.sec << ",\"nanosec\":" << arm_stamp_.nanosec << "}"
      << ",\"joint_names\":[\"joint1\",\"joint2\",\"joint3\",\"joint4\",\"joint5\",\"joint6\"]"
      << ",\"positions_rad\":[";
    for (int i = 0; i < 6; ++i) {
      if (i > 0) o << ",";
      if (joints_valid_ && latest_joints_valid_) o << std::fixed << std::setprecision(6) << joint_positions_[i];
      else o << "0.0";
    }
    o << "],\"positions_deg\":[";
    for (int i = 0; i < 6; ++i) {
      if (i > 0) o << ",";
      if (joints_valid_ && latest_joints_valid_) o << std::fixed << std::setprecision(3) << (joint_positions_[i] * 180.0 / M_PI);
      else o << "0.0";
    }
    o << "],\"velocities_rad_s\":[";
    for (int i = 0; i < 6; ++i) {
      if (i > 0) o << ",";
      if (joints_valid_ && latest_joints_valid_) o << std::fixed << std::setprecision(6) << joint_velocities_[i];
      else o << "0.0";
    }
    o << "],\"detail\":" << quote(latest_arm_state_.diagnostic)
      << "}";
    return o.str();
  }

  std::string arm_config_json() const {
    std::ostringstream o;
    o << "{\"profile\":" << quote(arm_profile_)
      << ",\"max_delta_deg\":" << arm_max_delta_deg_
      << ",\"timeout_sec\":" << arm_timeout_
      << "}";
    return o.str();
  }

  std::string arm_snapshot() const {
    std::ostringstream o;
    o << "{\"server_id\":" << quote(server_id_)
      << ",\"enable_arm_control\":" << boolean(enable_arm_control_)
      << ",\"allow_hardware\":" << boolean(hardware_)
      << ",\"can_move\":" << boolean(arm_blocked().empty())
      << ",\"blocked_reason\":" << quote(arm_blocked())
      << ",\"arm\":" << arm_json()
      << ",\"arm_config\":" << arm_config_json()
      << ",\"arm_command\":" << arm_command_json()
      << "}";
    return o.str();
  }

  std::string snapshot() const {
    std::ostringstream o;
    o << "{\"server_id\":" << quote(server_id_) << ",\"control_enabled\":" << boolean(enabled_)
      << ",\"enable_arm_control\":" << boolean(enable_arm_control_)
      << ",\"allow_hardware\":" << boolean(hardware_)
      << ",\"action_ready\":" << boolean(client_->action_server_is_ready())
      << ",\"can_initialize\":" << boolean(blocked(true).empty())
      << ",\"can_move\":" << boolean(blocked(false).empty())
      << ",\"blocked_reason\":" << quote(blocked(false))
      << ",\"state\":{\"received\":" << boolean(received_) << ",\"fresh\":" << boolean(fresh())
      << ",\"simulated\":" << boolean(state_.simulated) << ",\"connected\":" << boolean(state_.connected)
      << ",\"feedback_valid\":" << boolean(state_.feedback_valid) << ",\"feedback_age_sec\":" << age()
      << ",\"busy\":" << boolean(state_.busy) << ",\"commands_enabled\":" << boolean(state_.commands_enabled)
      << ",\"initialization\":" << int(state_.initialization) << ",\"grip_status\":" << int(state_.grip_status)
      << ",\"position_raw\":" << (fresh() ? state_.position_raw : 65535)
      << ",\"object_detected\":" << boolean(fresh() && state_.object_detected)
      << ",\"object_dropped\":" << boolean(fresh() && state_.object_dropped)
      << ",\"detail\":" << quote(state_.detail) << "},\"config\":{\"default_force_percent\":" << force_
      << ",\"max_force_percent\":" << max_force_ << ",\"open_position_raw\":" << open_
      << ",\"close_position_raw\":" << close_ << ",\"step_raw\":" << step_
      << ",\"timeout_sec\":" << timeout_ << "},\"command\":" << command_json()
      << ",\"arm\":" << arm_json()
      << ",\"arm_config\":" << arm_config_json()
      << ",\"arm_command\":" << arm_command_json() << "}";
    return o.str();
  }

  Response command(const std::string &body, bool initialize) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string id, signature;
    int position = 0, force = force_;
    try {
      boost::property_tree::ptree tree;
      std::istringstream stream(body);
      boost::property_tree::read_json(stream, tree);
      // Reject duplicate keys, unknown fields and nested values. Values undergo range checks below.
      std::unordered_map<std::string, bool> seen;
      for (const auto &entry : tree) {
        const auto &k = entry.first;
        if (!seen.emplace(k, true).second || !entry.second.empty() ||
            (k != "request_id" && k != "server_id" &&
             (initialize || (k != "position_raw" && k != "force_percent"))))
          throw std::invalid_argument("字段重复、未知或不是标量");
      }
      id = tree.get<std::string>("request_id");
      if (!std::regex_match(id, std::regex("[A-Za-z0-9_-]{8,80}"))) throw std::invalid_argument("无效 request_id");
      if (tree.get<std::string>("server_id") != server_id_)
        return error(http::status::conflict, "面板服务已重启，请刷新状态后重新操作");
      if (!initialize) {
        auto integer = [&tree](const std::string &key) {
          const auto v = tree.get<std::string>(key);
          if (!std::regex_match(v, std::regex("[0-9]{1,4}"))) throw std::invalid_argument("需要非负整数");
          return std::stoi(v);
        };
        position = integer("position_raw"); force = integer("force_percent");
        if (position > 1000 || force < 20 || force > max_force_) throw std::invalid_argument("开度或力设定超出范围");
      }
      signature = (initialize ? "I" : "M") + std::to_string(position) + ":" + std::to_string(force);
    } catch (const std::exception &e) { return error(http::status::bad_request, e.what()); }
    const auto old = requests_.find(id);
    if (old != requests_.end()) {
      if (old->second != signature) return error(http::status::conflict, "同一请求 ID 的参数不同");
      return response(http::status::ok, "{\"request_id\":" + quote(id) + ",\"phase\":\"duplicate\",\"detail\":\"重复请求，未再次发送\"}");
    }
    const auto why = blocked(initialize);
    if (!why.empty()) return error(http::status::conflict, why);
    // Bound the lifetime request ledger without evicting IDs and permitting later replay.
    if (requests_.size() >= 4096) return error(http::status::service_unavailable, "会话请求已满，请在设备空闲时重启面板");
    requests_[id] = signature;
    request_id_ = id; active_ = true; phase_ = "sending"; code_ = -1; attempted_ = false;
    detail_ = "请求已登记，等待夹爪接受";
    deadline_ = Clock::now() + std::chrono::milliseconds(int((timeout_ + 3.) * 1000));
    GripperAction::Goal goal;
    goal.operation = initialize ? GripperAction::Goal::INITIALIZE : GripperAction::Goal::MOVE;
    goal.position_raw = uint16_t(position); goal.force_percent = uint8_t(force);
    goal.require_contact = false; goal.timeout_sec = timeout_;
    auto options = rclcpp_action::Client<GripperAction>::SendGoalOptions();
    options.goal_response_callback = [this, id](GripperHandle::SharedPtr handle) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (id != request_id_) return;
      if (!handle) { phase_ = "rejected"; active_ = false; detail_ = "夹爪驱动拒绝指令，未接受执行"; }
      else if (phase_ != "unknown") { phase_ = "accepted"; detail_ = "夹爪已接受，等待设备反馈"; }
    };
    options.result_callback = [this, id](const GripperHandle::WrappedResult &result) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (id != request_id_) return;
      if (!result.result) { phase_ = "unknown"; detail_ = "未收到有效结果，请检查设备"; return; }
      const auto &r = *result.result;
      code_ = r.code; attempted_ = r.command_attempted;
      const bool uncertain = attempted_ && (code_ == 14 || code_ == 15 || code_ == 16);
      const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED && code_ <= 2;
      phase_ = uncertain ? "unknown" : (success ? "succeeded" : "failed");
      active_ = uncertain;
      if (code_ == 12) detail_ = "驱动检测到接触，未到目标开度；请目视判断是否夹住。";
      else if (code_ == 1) detail_ = "已到指定开度；是否夹住激光笔请目视确认。";
      else if (code_ == 0) detail_ = "初始化已确认。";
      else detail_ = r.detail;
      if (uncertain) detail_ = "执行结果不确定，夹爪未必已停止；请检查实物及驱动。 " + detail_;
    };
    try { client_->async_send_goal(goal, options); }
    catch (const std::exception &e) { phase_ = "unknown"; detail_ = std::string("发送结果未知，不重试：") + e.what(); }
    RCLCPP_INFO(get_logger(), "manual request=%s operation=%u position=%u force=%u require_contact=false",
      id.c_str(), goal.operation, goal.position_raw, goal.force_percent);
    return response(http::status::accepted, command_json());
  }

  Response arm_command(const std::string &body) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string id, signature;
    int joint_index = -1;
    double target_deg = 0.0;
    try {
      boost::property_tree::ptree tree;
      std::istringstream stream(body);
      boost::property_tree::read_json(stream, tree);
      std::unordered_map<std::string, bool> seen;
      for (const auto &entry : tree) {
        const auto &k = entry.first;
        if (!seen.emplace(k, true).second || !entry.second.empty() ||
            (k != "request_id" && k != "server_id" && k != "joint_index" && k != "target_deg"))
          throw std::invalid_argument("字段重复、未知或不是标量");
      }
      if (!tree.get_optional<std::string>("request_id") ||
          !tree.get_optional<std::string>("server_id") ||
          !tree.get_optional<std::string>("joint_index") ||
          !tree.get_optional<std::string>("target_deg"))
        throw std::invalid_argument("缺少必要字段");

      id = tree.get<std::string>("request_id");
      if (!std::regex_match(id, std::regex("[A-Za-z0-9_-]{8,80}")))
        throw std::invalid_argument("无效 request_id");
      if (tree.get<std::string>("server_id") != server_id_)
        return error(http::status::conflict, "面板服务已重启，请刷新状态后重新操作");

      const std::string ji_str = tree.get<std::string>("joint_index");
      if (!std::regex_match(ji_str, std::regex("^[0-5]$")))
        throw std::invalid_argument("joint_index 必须为 0 到 5 的整数");
      joint_index = std::stoi(ji_str);

      const std::string td_str = tree.get<std::string>("target_deg");
      if (!std::regex_match(td_str, std::regex("^-?[0-9]+(\\.[0-9]+)?([eE][+-]?[0-9]+)?$")))
        throw std::invalid_argument("target_deg 必须为有限数值");
      target_deg = std::stod(td_str);
      if (!std::isfinite(target_deg))
        throw std::invalid_argument("target_deg 必须为有限数值");

      std::ostringstream sig;
    sig << "J" << joint_index << ":" << std::setprecision(17) << target_deg;
      signature = sig.str();
    } catch (const std::exception &e) {
      return error(http::status::bad_request, e.what());
    }

    const auto old = requests_.find(id);
    if (old != requests_.end()) {
      if (old->second != signature) return error(http::status::conflict, "同一请求 ID 的参数不同");
      return response(http::status::ok, "{\"request_id\":" + quote(id) + ",\"phase\":\"duplicate\",\"detail\":\"重复请求，未再次发送\"}");
    }

    const auto why = arm_blocked();
    if (!why.empty()) return error(http::status::conflict, why);

    const double measured_rad = joint_positions_[joint_index];
    const double measured_deg = measured_rad * 180.0 / M_PI;
    const double delta_deg = std::abs(target_deg - measured_deg);
    if (delta_deg > arm_max_delta_deg_) {
      std::ostringstream msg;
      msg << "单次角度变动 " << std::fixed << std::setprecision(2) << delta_deg
          << "° 超过最大限制 " << arm_max_delta_deg_ << "°";
      return error(http::status::bad_request, msg.str());
    }

    if (requests_.size() >= 4096)
      return error(http::status::service_unavailable, "会话请求已满，请在设备空闲时重启面板");

    requests_[id] = signature;
    arm_request_id_ = id;
    arm_active_ = true;
    arm_phase_ = "sending";
    arm_code_ = -1;
    arm_attempted_ = false;
    arm_detail_ = "请求已登记，等待机械臂接受";
    arm_joint_index_ = joint_index;
    arm_target_deg_ = target_deg;
    arm_deadline_ = Clock::now() + std::chrono::milliseconds(int((arm_timeout_ + 3.) * 1000));

    MoveJoints::Goal goal;
    goal.header.frame_id = "base_link";
    goal.header.stamp = joint_stamp_;
    goal.profile = arm_profile_;
    goal.plan_only = false;
    for (size_t i = 0; i < 6; ++i) {
      goal.expected_start[i] = joint_positions_[i];
      goal.positions[i] = joint_positions_[i];
    }
    goal.positions[joint_index] = target_deg * M_PI / 180.0;

    auto options = rclcpp_action::Client<MoveJoints>::SendGoalOptions();
    options.goal_response_callback = [this, id](ArmHandle::SharedPtr handle) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (id != arm_request_id_) return;
      if (!handle) {
        arm_phase_ = "rejected"; arm_active_ = false;
        arm_detail_ = "机械臂驱动拒绝指令，未接受执行";
      } else if (arm_phase_ != "unknown") {
        arm_phase_ = "accepted";
        arm_detail_ = "机械臂已接受，等待执行结果";
      }
    };
    options.result_callback = [this, id](const ArmHandle::WrappedResult &result) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (id != arm_request_id_) return;
      if (!result.result) {
        arm_phase_ = "unknown";
        arm_detail_ = "未收到有效机械臂结果，请检查设备";
        arm_active_ = true;
        return;
      }
      const auto &r = *result.result;
      arm_code_ = r.code;
      arm_attempted_ = r.executed;
      if (arm_phase_ == "unknown") {
        arm_detail_ = "收到迟到结果，仍保持锁定；请检查设备后在空闲时重启面板。 " + r.detail;
        return;
      }
      const bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED && r.code == 1);
      const bool uncertain = !success && (r.code == 19 || (r.executed && (r.code == 13 || r.code == 16)));
      arm_phase_ = uncertain ? "unknown" : (success ? "succeeded" : "failed");
      arm_active_ = uncertain;
      if (success) {
        arm_detail_ = r.detail.empty() ? "关节运动完成。" : r.detail;
      } else {
        arm_detail_ = r.detail.empty() ? ("机械臂执行失败，返回码: " + std::to_string(r.code)) : r.detail;
        if (uncertain) {
          arm_detail_ = "执行结果不确定，机械臂未必已停止；请检查实物及驱动。 " + arm_detail_;
        }
      }
    };

    try { arm_client_->async_send_goal(goal, options); }
    catch (const std::exception &e) {
      arm_phase_ = "unknown";
      arm_detail_ = std::string("发送结果未知，不重试：") + e.what();
    }

    RCLCPP_INFO(get_logger(), "arm joint request=%s joint=%d target_deg=%.2f profile=%s",
      id.c_str(), joint_index, target_deg, arm_profile_.c_str());

    return response(http::status::accepted, arm_command_json());
  }

  std::string bind_, web_root_, server_id_, request_id_, phase_ = "idle", detail_;
  std::string arm_request_id_, arm_phase_ = "idle", arm_detail_, arm_profile_ = "roundtrip_low";
  int port_, force_, max_force_, open_, close_, step_, code_ = -1;
  int arm_code_ = -1, arm_joint_index_ = -1;
  double arm_target_deg_ = 0.0, arm_max_delta_deg_ = 10.0, arm_timeout_ = 10.0;
  bool enabled_, enable_arm_control_ = false, hardware_, received_ = false, active_ = false, attempted_ = false;
  bool arm_received_ = false, joints_received_ = false, joints_valid_ = false, arm_active_ = false, arm_attempted_ = false;
  double timeout_, freshness_, message_delay_ = -1.;
  GripperState state_;
  ArmState arm_state_;
  // Raw safety observations are independent of the retained coherent pair.
  ArmState latest_arm_state_;
  bool recovery_required_=false,have_arm_pair_=false,latest_arm_timing_valid_=false,latest_joints_timing_valid_=false,latest_joints_valid_=false;
  int64_t latest_arm_key_=0,latest_joint_key_=0,recovery_source_floor_=0;
  std::array<double,6> latest_joint_velocities_{};
  std::map<int64_t,ArmFrame> pending_arms_;
  std::map<int64_t,JointFrame> pending_joints_;
  builtin_interfaces::msg::Time arm_stamp_, joint_stamp_;
  std::array<double, 6> joint_positions_{};
  std::array<double, 6> joint_velocities_{};
  Clock::time_point received_at_, deadline_;
  Clock::time_point arm_received_at_, joints_received_at_, arm_deadline_;
  std::mutex mutex_;
  std::unordered_map<std::string, std::string> requests_;
  rclcpp_action::Client<GripperAction>::SharedPtr client_;
  rclcpp_action::Client<MoveJoints>::SharedPtr arm_client_;
  rclcpp::Subscription<GripperState>::SharedPtr subscription_;
  rclcpp::Subscription<ArmState>::SharedPtr arm_state_sub_;
  rclcpp::Subscription<JointState>::SharedPtr arm_joints_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};

class Session : public std::enable_shared_from_this<Session> {
public:
  Session(tcp::socket socket, Panel &panel) : stream_(std::move(socket)), panel_(panel) { parser_.body_limit(8192); }
  void run() {
    stream_.expires_after(std::chrono::seconds(5));
    http::async_read(stream_, buffer_, parser_, [self = shared_from_this()](beast::error_code ec, std::size_t) {
      if (ec) return;
      try { self->response_ = self->panel_.serve(self->parser_.get()); }
      catch (...) { self->response_ = Response(http::status::internal_server_error, 11); self->response_.prepare_payload(); }
      self->stream_.expires_after(std::chrono::seconds(5));
      http::async_write(self->stream_, self->response_, [self](beast::error_code, std::size_t) {
        beast::error_code ignored;
        self->stream_.socket().shutdown(tcp::socket::shutdown_both, ignored);
      });
    });
  }
private:
  beast::tcp_stream stream_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
  Response response_;
  Panel &panel_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto panel = std::make_shared<Panel>();
    net::io_context io(1);
    tcp::acceptor acceptor(io, {net::ip::make_address(panel->bind()), uint16_t(panel->port())});
    std::function<void()> accept;
    accept = [&] {
      acceptor.async_accept([&](beast::error_code ec, tcp::socket socket) {
        if (!ec) std::make_shared<Session>(std::move(socket), *panel)->run();
        if (acceptor.is_open()) accept();
      });
    };
    accept();
    std::thread web([&] { io.run(); });
    RCLCPP_INFO(panel->get_logger(), "Panel http://%s:%d; manual control ready",
      panel->bind().c_str(), panel->port());
    try { rclcpp::spin(panel); }
    catch (...) { io.stop(); web.join(); throw; }
    io.stop(); web.join();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("panel_node"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
