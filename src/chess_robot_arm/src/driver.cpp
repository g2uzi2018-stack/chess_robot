#include "chess_robot_arm/driver.hpp"
#include "chess_robot_arm/dashboard_reply.hpp"
#include "chess_robot_arm/net.hpp"
#include <filesystem>
#include <algorithm>
#include <iomanip>
#include <locale>
#include <sstream>
#include "Dashboard.h"
#include "FeedbackData.h"

namespace chess_robot_arm {
namespace {
// Reuse the official encoder without connecting its synchronous transport.
class OfficialCommand final : public Dobot::CDashboard {
public:
  std::string SendRecvMsg(std::string &s) override { return s; }
  std::string SendRecvMsg(char *s) override { return s; }
};
Dobot::CJointPoint sdkJoints(const Joints &q) {
  for (double x : q) if (!std::isfinite(x)) throw Failure(INVALID_GOAL, "nonfinite SDK joint");
  return {q[0]*180/pi,q[1]*180/pi,q[2]*180/pi,q[3]*180/pi,q[4]*180/pi,q[5]*180/pi};
}
struct ControllerRejected : Failure {
  explicit ControllerRejected(const std::string &detail)
      : Failure(EXECUTION_FAILED, detail) {}
};
class TcpTransport final : public CommandTransport {
  Socket socket_;
  int timeout_;
  int stop_timeout_, enable_timeout_;

public:
  explicit TcpTransport(const Config &c)
      : socket_(c.ip, c.dashboard_port, c.timeout_ms), timeout_(c.timeout_ms),
        stop_timeout_(c.stop_ack_timeout_ms), enable_timeout_(c.enable_ack_timeout_ms) {}
  std::string query(const std::string &c) override {
    return socket_.query(c, c == "Stop()" ? stop_timeout_ : c == "EnableRobot()" ? enable_timeout_ : timeout_);
  }
  bool streaming() const override { return true; }
  void send(const std::string &c) override {
    socket_.send_command(c, c == "Stop()" ? stop_timeout_ : timeout_);
  }
  std::string receive(int timeout_ms) override { return socket_.receive_reply(timeout_ms); }
};
std::string number(double x) {
  if (!std::isfinite(x))
    throw Failure(INVALID_GOAL, "nonfinite SDK argument");
  std::ostringstream s;
  s.imbue(std::locale::classic());
  // The target controller rejects exponent notation (e.g. a near-zero Ry)
  // with -30005. Keep all wire numbers in decimal notation, including tiny
  // joint values and servo periods. Nine decimal places are below SDK units.
  s << std::fixed << std::setprecision(9) << x;
  return s.str();
}
std::string list(const Joints &q, double scale) {
  std::string s;
  for (size_t i = 0; i < 6; ++i) {
    if (i)
      s += ",";
    s += number(q[i] * scale);
  }
  return s;
}
std::array<double, 6> six(const std::string &payload) {
  std::istringstream s(payload);
  s.imbue(std::locale::classic());
  std::array<double, 6> out{};
  for (size_t i = 0; i < 6; ++i) {
    if (!(s >> out[i]) || !std::isfinite(out[i]))
      throw Failure(PLANNING_FAILED, "invalid SDK six-value payload");
    if (i < 5) {
      char comma;
      if (!(s >> comma) || comma != ',')
        throw Failure(PLANNING_FAILED, "invalid SDK payload delimiter");
    }
  }
  s >> std::ws;
  if (!s.eof())
    throw Failure(PLANNING_FAILED, "extra SDK payload data");
  return out;
}
} // namespace
Feedback decodeFeedbackFrame(const FeedbackFrame &b, Clock::time_point received, int64_t ros_received) {
  validate_feedback(b);
  Feedback f;
  f.received = received;
  f.ros_received_ns = ros_received;
  static_assert(sizeof(Dobot::CFeedbackData)==1440);
  static_assert(offsetof(Dobot::CFeedbackData,RobotMode)==24);
  static_assert(offsetof(Dobot::CFeedbackData,TimeStamp)==32);
  static_assert(offsetof(Dobot::CFeedbackData,TestValue)==48);
  static_assert(offsetof(Dobot::CFeedbackData,QActual)==432);
  static_assert(offsetof(Dobot::CFeedbackData,QDActual)==480);
  Dobot::CFeedbackData data;
  std::memcpy(&data,b.data(),sizeof(data));
  f.controller_stamp = data.TimeStamp;
  f.mode = data.RobotMode;
  for (size_t i=0;i<6;++i) {
    f.q[i]=data.QActual[i]*pi/180;
    f.speed[i]=data.QDActual[i]*pi/180;
  }
  return f;
}
Transform sdkPose(const std::array<double, 6> &a) {
  for (double x : a)
    if (!std::isfinite(x))
      throw Failure(PLANNING_FAILED, "nonfinite SDK pose");
  Transform t = Transform::Identity();
  t.translation() = Eigen::Vector3d(a[0], a[1], a[2]) * 0.001;
  // Manufacturer Rx,Ry,Rz degrees: Rz * Ry * Rx (extrinsic fixed XYZ).
  t.linear() = (Eigen::AngleAxisd(a[5] * pi / 180, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(a[4] * pi / 180, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(a[3] * pi / 180, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  return t;
}
std::array<double, 6> toSdkPose(const Transform &t) {
  if (!t.matrix().allFinite())
    throw Failure(INVALID_GOAL, "nonfinite SDK transform");
  // Euler branches are pose representations only; joint turns are never
  // wrapped.
  auto zyx = t.linear().eulerAngles(2, 1, 0);
  return {{t.translation().x() * 1000, t.translation().y() * 1000,
           t.translation().z() * 1000, zyx[2] * 180 / pi, zyx[1] * 180 / pi,
           zyx[0] * 180 / pi}};
}
V4Driver::V4Driver(const Config &c, TransportFactory factory, std::function<int64_t()> ros_clock)
    : config_(c), transport_(std::move(factory)), ros_clock_(std::move(ros_clock)) {
  if (!transport_)
    transport_ = [c]() { return std::make_unique<TcpTransport>(c); };
  if (c.feedback_trace) feedback_timings_.reserve(262144);
  if (!c.audit_directory.empty()) {
    try {
    audit_directory_=std::make_unique<AuditDirectory>(c.audit_directory);
    command_audit_=std::make_unique<RotatingAudit>(c.audit_directory+"/commands.csv",c.audit_max_bytes,c.audit_keep_files,"host_utc_ns,latency_s,command,reply_or_error\n");
    feedback_audit_=std::make_unique<RotatingAudit>(c.audit_directory+"/feedback.bin",c.audit_max_bytes,c.audit_keep_files);
    send_audit_=std::make_unique<RotatingAudit>(c.audit_directory+"/servo_send.csv",c.audit_max_bytes,c.audit_keep_files,"host_utc_ns,send_duration_s,pending_count\n");
    } catch(const Failure &e) {
      if(e.code==BUSY)throw;
      audit_init_error_=e.what();
    } catch(const std::exception &e) {audit_init_error_=e.what();}
  }
}
V4Driver::~V4Driver() {
  try { closeStream(true); } catch (...) {}
  try { flushFeedbackTimings(); } catch (...) {}
}
std::string V4Driver::auditStatus() {
  std::string status=audit_init_error_.empty()?"":"; diagnostic audit unavailable: "+audit_init_error_;
  for(auto *log:{command_audit_.get(),feedback_audit_.get(),send_audit_.get()}) {
    if(log && (log->dropped() || !log->error().empty()))
      status+="; diagnostic audit dropped="+std::to_string(log->dropped())+" error="+log->error();
  }
  return status;
}
void V4Driver::saveFeedbackTiming(const FeedbackTiming &row) {
  if (!config_.feedback_trace) return;
  if (feedback_timings_.size()<262144) feedback_timings_.push_back(row);
  else ++timing_dropped_; // Never allocate or flush from the receive path.
}
void V4Driver::noteFeedbackDispatch(Clock::time_point begin,Clock::time_point end) {
  if (!config_.feedback_trace || feedback_timings_.empty()) return;
  auto &r=feedback_timings_.back();
  r.dispatch_begin=std::chrono::duration_cast<std::chrono::nanoseconds>(begin.time_since_epoch()).count();
  r.dispatch_end=std::chrono::duration_cast<std::chrono::nanoseconds>(end.time_since_epoch()).count();
}
void V4Driver::noteFeedbackBackoff(Clock::time_point begin,Clock::time_point end) {
  if (!config_.feedback_trace || feedback_timings_.empty()) return;
  auto &r=feedback_timings_.back();
  r.backoff_begin=std::chrono::duration_cast<std::chrono::nanoseconds>(begin.time_since_epoch()).count();
  r.backoff_end=std::chrono::duration_cast<std::chrono::nanoseconds>(end.time_since_epoch()).count();
}
void V4Driver::flushFeedbackTimings() {
  if (!config_.feedback_trace) return;
  std::ofstream out(config_.audit_directory+"/feedback_timing.csv");
  out << "frame,host_utc_ns,read_begin_steady_ns,complete_steady_ns,audit_begin_steady_ns,audit_end_steady_ns,decode_end_steady_ns,dispatch_begin_steady_ns,dispatch_end_steady_ns,backoff_begin_steady_ns,backoff_end_steady_ns,controller_stamp,mode,status\n";
  for (const auto &r:feedback_timings_)
    out<<r.frame<<','<<r.host<<','<<r.begin<<','<<r.complete<<','<<r.audit_begin<<','<<r.audit_end<<','<<r.decode_end<<','<<r.dispatch_begin<<','<<r.dispatch_end<<','<<r.backoff_begin<<','<<r.backoff_end<<','<<r.controller<<','<<r.mode<<','<<r.status<<'\n';
  std::ofstream meta(config_.audit_directory+"/feedback_timing_meta.txt");
  meta<<"capacity=262144\nrows="<<feedback_timings_.size()<<"\ndropped="<<timing_dropped_<<"\nflush=after_feedback_thread_join\n";
}
void V4Driver::audit(const std::string &command, const std::string &value,
                     Clock::time_point start) {
  if (!command_audit_) return;
  auto escape = [](const std::string &s) {
    std::string out;
    for (char c : s) { out += c; if (c == '"') out += '"'; }
    return out;
  };
  std::ostringstream row;row << utc_ns() << "," << std::setprecision(9)
      << std::chrono::duration<double>(Clock::now() - start).count()
      << ",\"" << escape(command) << "\",\"" << escape(value) << "\"\n";
  command_audit_->write(row.str());
}
std::string V4Driver::call(const std::string &command,const std::function<void()> &before_send) {
  // Reuse a single ordered stream: per-command connection setup caused
  // intermittent query timeouts in the real burst test. The real controller
  // rejects a second command connection from this host, so Stop must use the
  // same stream. An in-flight call can delay Stop by its bounded I/O deadline;
  // callers must budget that latency. Never retry an ambiguous command.
  auto query = [&](CommandTransport &transport) {
    const auto start = Clock::now();
    auto audit = [&](const std::string &value) {
      if (!command_audit_)
        return;
      auto escape = [](const std::string &s) {
        std::string out;
        for (char c : s) {
          out += c;
          if (c == '\"')
            out += '\"';
        }
        return out;
      };
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
      std::ostringstream row;row << ns << "," << std::setprecision(9)
          << std::chrono::duration<double>(Clock::now() - start).count()
          << ",\"" << escape(command) << "\",\"" << escape(value) << "\"\n";
      command_audit_->write(row.str());
    };
    std::string raw;
    try {
      raw = transport.query(command);
    } catch (const QueryFailure &e) {
      audit(std::string(e.what()) + "; partial=" + e.partial_reply);
      throw Failure(EXECUTION_FAILED,
                    std::string(e.what()) +
                        "; partial reply: " + e.partial_reply);
    }
    audit(raw);
    auto reply = parse_dashboard_reply(raw, command);
    if (reply.error != 0)
      throw ControllerRejected("controller error " +
                               std::to_string(reply.error) + " for " + command);
    return reply.payload;
  };
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (streaming_) throw Failure(BUSY, "SDK query during active ServoJ stream");
  if (session_fault_ && command != "Stop()")
    throw Failure(EXECUTION_FAILED, "ambiguous session locked; Stop only, no automatic reconnect");
  if (!command_transport_)
    command_transport_ = transport_();
  // A guard rejection occurs before any bytes are sent and must not mark the
  // ordered socket ambiguous. No reference lock is held during network waits.
  if(before_send)before_send();
  try {
    return query(*command_transport_);
  } catch (const ControllerRejected &) {
    // A matching, well-formed negative reply does not corrupt the stream.
    // Keep its exclusive session available for the next IK seed or Stop.
    throw;
  } catch (...) {
    // Keep the exclusive session for one Stop attempt; reconnecting immediately
    // can be refused as occupied, and may replay ambiguous work.
    session_fault_ = true;
    throw;
  }
}
Transform V4Driver::fk(const Joints &q) {
  OfficialCommand sdk;
  return sdkPose(six(call(sdk.PositiveKin(sdkJoints(q), 0, 0))));
}
Joints V4Driver::ik(const Transform &flange, const Joints &seed) {
  const auto a = toSdkPose(flange);
  OfficialCommand sdk;
  Dobot::CDescartesPoint pose{a[0],a[1],a[2],a[3],a[4],a[5]};
  auto values = six(call(sdk.InverseKin(pose, "useJointNear=1",
      "jointNear={" + list(seed,180/pi) + "}", "user=0", "tool=0")));
  for (auto &v : values)
    v *= pi / 180;
  return values;
}
Feedback V4Driver::readFeedback() {
  FeedbackTiming timing;
  try {
    if (!feedback_socket_)
      feedback_socket_ = std::make_unique<Socket>(
          config_.ip, config_.feedback_port, config_.timeout_ms);
    // Overall deadline also covers duplicates. Duplicate packets do not refresh
    // host freshness.
    // Feedback gets its own requested freshness window. Dashboard, ServoJ ACK
    // and Stop deadlines remain independent; a duplicate never restarts this.
    const auto receive_window = std::max(
        std::chrono::milliseconds(config_.timeout_ms),
        std::chrono::ceil<std::chrono::milliseconds>(
            std::chrono::duration<double>(config_.freshness)));
    const auto deadline = Clock::now() + receive_window;
    while (Clock::now() < deadline) {
      FeedbackFrame b{};
      timing=FeedbackTiming{};if (config_.feedback_trace) timing.begin=steady_ns();
      auto remaining =
          std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now())
              .count();
      feedback_socket_->exact(
          b.data(), b.size(),
          static_cast<int>(std::max<int64_t>(1, remaining)));
      const auto received = Clock::now();
      const auto ros_received = ros_clock_ ? ros_clock_() : 0;
      const auto host_received=utc_ns();
      if (config_.feedback_trace) {
        timing.frame=++timing_frames_;timing.host=host_received;
        timing.complete=std::chrono::duration_cast<std::chrono::nanoseconds>(received.time_since_epoch()).count();
        timing.controller=feedback_field<uint64_t>(b,32);timing.mode=feedback_field<uint64_t>(b,24);
        timing.audit_begin=steady_ns();
      }
      if (feedback_audit_) {
        const int64_t ns=host_received;
        std::string record(sizeof(ns)+b.size(),'\0');
        std::memcpy(record.data(),&ns,sizeof(ns));std::memcpy(record.data()+sizeof(ns),b.data(),b.size());
        feedback_audit_->write(std::move(record));
      }
      if (config_.feedback_trace) timing.audit_end=steady_ns();
      auto result=decodeFeedbackFrame(b,received,ros_received);
      // Admit a new disabled telemetry epoch only. This does NOT restore trust:
      // State and the durable checkpoint still gate every enable/motion.
      if(!config_.checkpoint_file.empty() && config_.checkpoint_no_unobserved_motion &&
          result.mode==4 && sequence_.regresses(b) &&
          std::all_of(result.speed.begin(),result.speed.end(),[&](double x){return std::abs(x)<=config_.stationary_speed;}))
        sequence_=FeedbackSequence{};
      const bool accepted=sequence_.accept(b);
      if (config_.feedback_trace) {
        timing.status=accepted ? 1 : 0;timing.decode_end=steady_ns();saveFeedbackTiming(timing);
      }
      if (!accepted) continue;
      return result;
    }
    throw Failure(FEEDBACK_FAULT, "duplicate feedback deadline exceeded");
  } catch (...) {
    if (config_.feedback_trace) {timing.status=-1;timing.decode_end=steady_ns();saveFeedbackTiming(timing);}
    feedback_socket_.reset();
    throw;
  }
}
void V4Driver::servo(const Joints &q, double period) {
  if (!config_.hardwareAllowed())
    throw Failure(HARDWARE_DISABLED, "ServoJ gated by hardware prerequisites");
  if (!std::isfinite(period) || period < 0.004 || period > 3600)
    throw Failure(INVALID_GOAL, "invalid ServoJ period");
  OfficialCommand sdk;
  auto command = sdk.ServoJ(sdkJoints(q),
      "t=" + number(config_.servo_command_time>0 ? config_.servo_command_time : period),
      "aheadtime=" + number(config_.servo_ahead), "gain=" + number(config_.servo_gain));
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    if (streaming_) {
      std::lock_guard<std::mutex> l(stream_mutex_);
      if (!stream_error_.empty()) throw Failure(EXECUTION_FAILED, stream_error_);
      if (pending_.size() >= config_.servo_max_pending)
        throw Failure(EXECUTION_FAILED, "ServoJ unacknowledged window full");
      const auto sent = Clock::now();
      // Queue before sending while holding the mutex: the reader must never
      // observe a reply without its exact expected echo and send deadline.
      pending_.push_back({command, sent});
      try { command_transport_->send(command); }
      catch (const std::exception &e) {
        stream_error_ = std::string("ambiguous ServoJ send: ") + e.what();
        stream_broken_ = true;
        stream_cv_.notify_all();
        throw Failure(EXECUTION_FAILED, stream_error_);
      }
      if (send_audit_) {
        std::ostringstream row;row << utc_ns() << ',' << std::setprecision(9)
          << std::chrono::duration<double>(Clock::now() - sent).count() << ',' << pending_.size() << '\n';
        send_audit_->write(row.str());
      }
      stream_cv_.notify_all();
      return;
    }
  }
  auto payload = call(command);
  validateServoReply(payload, false);
}
void V4Driver::validateServoReply(const std::string &payload, bool ordered) {
  // Queue ID is an acknowledgement, never an arrival assertion.
  uint64_t id = 0;
  auto r = std::from_chars(payload.data(), payload.data() + payload.size(), id);
  if (r.ec != std::errc() || r.ptr != payload.data() + payload.size())
    throw Failure(EXECUTION_FAILED, "invalid ServoJ queue ID");
  if (ordered && last_queue_id_ && id <= *last_queue_id_)
    throw Failure(EXECUTION_FAILED, "non-increasing ServoJ queue ID");
  if (ordered) last_queue_id_ = id;
}
void V4Driver::beginStream() {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  if (streaming_) throw Failure(EXECUTION_FAILED, "stream already active");
  if (session_fault_) throw Failure(EXECUTION_FAILED, "ambiguous session locked; no trajectory replay");
  if (!command_transport_) command_transport_ = transport_();
  if (!command_transport_->streaming()) return; // Query-only injected fixtures.
  std::lock_guard<std::mutex> l(stream_mutex_);
  pending_.clear(); stream_error_.clear(); stream_exit_ = stream_broken_ = false;
  last_queue_id_.reset(); streaming_ = true;
  reply_thread_ = std::thread([this] { replyLoop(); });
}
void V4Driver::replyLoop() {
  std::optional<Clock::time_point> fault_drain_deadline;
  auto deadline_fault = [&] {
    // A late ACK still fails the motion immediately. Keep the exclusive
    // socket only to drain its known FIFO before Stop; never resend a point.
    std::lock_guard<std::mutex> l(stream_mutex_);
    if (stream_error_.empty()) stream_error_ = "ServoJ acknowledgement deadline";
    if (!fault_drain_deadline)
      fault_drain_deadline = Clock::now()+std::chrono::milliseconds(config_.timeout_ms);
  };
  auto drain_remaining = [&] {
    auto ms=std::chrono::ceil<std::chrono::milliseconds>(*fault_drain_deadline-Clock::now()).count();
    if (ms<=0) throw std::runtime_error("Stop FIFO drain deadline");
    return int(ms);
  };
  for (;;) {
    Pending item;
    {
      std::unique_lock<std::mutex> l(stream_mutex_);
      stream_cv_.wait(l, [this] { return stream_exit_ || !pending_.empty(); });
      if (pending_.empty() || stream_broken_) return;
      item = pending_.front();
    }
    try {
      double remaining = config_.servo_ack_timeout -
          std::chrono::duration<double>(Clock::now() - item.sent).count();
      if (remaining <= 0 && !fault_drain_deadline) deadline_fault();
      std::string raw;
      try {
        raw = command_transport_->receive(fault_drain_deadline ? drain_remaining() : std::max(1,int(std::ceil(remaining*1000))));
      } catch (const QueryFailure &e) {
        if (fault_drain_deadline || std::string(e.what())!="socket deadline exceeded") throw;
        audit(item.command,std::string(e.what())+"; Stop-only FIFO drain",item.sent);
        deadline_fault();
        raw=command_transport_->receive(drain_remaining());
      }
      if (Clock::now()-item.sent>std::chrono::duration<double>(config_.servo_ack_timeout))
        deadline_fault();
      audit(item.command, raw, item.sent);
      auto reply = parse_dashboard_reply(raw, item.command);
      if (reply.error) throw ControllerRejected("ServoJ controller error " + std::to_string(reply.error));
      validateServoReply(reply.payload, true);
      std::lock_guard<std::mutex> l(stream_mutex_);
      pending_.pop_front(); stream_cv_.notify_all();
    } catch (const ControllerRejected &e) {
      std::lock_guard<std::mutex> l(stream_mutex_);
      if (stream_error_.empty()) stream_error_ = e.what();
      pending_.pop_front(); stream_cv_.notify_all();
    } catch (const std::exception &e) {
      audit(item.command, e.what(), item.sent);
      std::lock_guard<std::mutex> l(stream_mutex_);
      stream_error_ = std::string("ServoJ reply fault: ") + e.what();
      stream_broken_ = true; pending_.clear(); stream_cv_.notify_all(); return;
    }
  }
}
void V4Driver::checkStream() {
  std::lock_guard<std::mutex> l(stream_mutex_);
  if (!stream_error_.empty()) throw Failure(EXECUTION_FAILED, stream_error_);
  if (!pending_.empty() && Clock::now() - pending_.front().sent >
      std::chrono::duration<double>(config_.servo_ack_timeout))
    throw Failure(EXECUTION_FAILED, "ServoJ acknowledgement deadline");
}
void V4Driver::closeStream(bool stopping) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  if (!streaming_) return;
  {
    std::unique_lock<std::mutex> l(stream_mutex_);
    // No further writes are admitted. Reader drains the bounded FIFO so Stop
    // cannot consume an older ServoJ echo as its own acknowledgement.
    stream_exit_ = true; stream_cv_.notify_all();
    stream_cv_.wait(l, [this] { return pending_.empty() || stream_broken_; });
  }
  if (reply_thread_.joinable()) reply_thread_.join();
  streaming_ = false;
  std::lock_guard<std::mutex> l(stream_mutex_);
  if (stream_broken_) session_fault_ = true; // Retain exclusive socket for Stop.
  auto error = stream_error_;
  pending_.clear(); stream_error_.clear();
  if (!stopping && !error.empty()) throw Failure(EXECUTION_FAILED, error);
}
void V4Driver::endStream() { closeStream(false); }
void V4Driver::enableAfterHistoryRestore(bool checkpoint_attested,const std::function<void()> &before_enable) {
  const bool old_recovery=config_.power_cycle_handoff && config_.enable_after_history_restore;
  const bool checkpoint_recovery=!config_.checkpoint_file.empty() && config_.checkpoint_enable_after_restore &&
      (config_.checkpoint_no_unobserved_motion || checkpoint_attested);
  if (!config_.hardwareAllowed() || !(old_recovery || checkpoint_recovery))
    throw Failure(HARDWARE_DISABLED,"explicit powered recovery is not enabled");
  if(!before_enable)throw Failure(REFERENCE_INVALID,"recovery enable requires a live state guard");
  OfficialCommand sdk;
  const auto errors=call(sdk.GetErrorID());
  if (errors.empty() || errors.find_first_not_of("[], \t\r\n")!=std::string::npos)
    throw Failure(FEEDBACK_FAULT,"recovery enable refused with active/unknown alarms: "+errors);
  if (!call(sdk.EnableRobot(),before_enable).empty())
    throw Failure(EXECUTION_FAILED,"unexpected recovery EnableRobot payload");
}
void V4Driver::stop() {
  if (!config_.hardwareAllowed())
    throw Failure(HARDWARE_DISABLED,
                  "Stop gated; passive backend cannot actuate");
  closeStream(true);
  OfficialCommand sdk;
  // A broken FIFO is never replaced by a competing connection. Send Stop once
  // on its original session, then look only for its exact echo within one budget.
  // Older complete replies are audit evidence, not acknowledgements of Stop.
  if (session_fault_ && command_transport_ && command_transport_->streaming()) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const auto started=Clock::now();
    const auto deadline=started+std::chrono::milliseconds(config_.stop_ack_timeout_ms);
    command_transport_->send(sdk.Stop());
    for (size_t i=0;i<config_.servo_max_pending+1;++i) {
      auto remaining=std::chrono::ceil<std::chrono::milliseconds>(deadline-Clock::now()).count();
      if (remaining<=0) throw Failure(STOP_FAILED,"Stop acknowledgement deadline");
      auto raw=command_transport_->receive(int(remaining));
      audit("Stop()",raw,started);
      DashboardReply reply{};
      try { reply=parse_dashboard_reply(raw,"Stop()"); }
      catch (const std::exception &) { continue; }
      if (reply.error || !reply.payload.empty())
        throw Failure(STOP_FAILED,"Stop rejected on original session: "+raw);
      return; // Executor still requires fresh measured idle/stationary dwell.
    }
    throw Failure(STOP_FAILED,"Stop echo missing after bounded residual replies");
  }
  if (!call(sdk.Stop()).empty())
    throw Failure(STOP_FAILED, "unexpected Stop payload");
}
} // namespace chess_robot_arm
