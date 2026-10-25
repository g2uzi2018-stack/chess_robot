#include "chess_robot_arm/driver.hpp"
#include "chess_robot_arm/dashboard_reply.hpp"
#include "chess_robot_arm/net.hpp"
#include <filesystem>
#include <iomanip>
#include <locale>
#include <sstream>

namespace chess_robot_arm {
namespace {
struct ControllerRejected : Failure {
  explicit ControllerRejected(const std::string &detail)
      : Failure(EXECUTION_FAILED, detail) {}
};
class TcpTransport final : public CommandTransport {
  Socket socket_;
  int timeout_;
  int stop_timeout_;

public:
  explicit TcpTransport(const Config &c)
      : socket_(c.ip, c.dashboard_port, c.timeout_ms), timeout_(c.timeout_ms),
        stop_timeout_(c.stop_ack_timeout_ms) {}
  std::string query(const std::string &c) override {
    return socket_.query(c, c == "Stop()" ? stop_timeout_ : timeout_);
  }
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
V4Driver::V4Driver(const Config &c, TransportFactory factory)
    : config_(c), transport_(std::move(factory)) {
  if (!transport_)
    transport_ = [c]() { return std::make_unique<TcpTransport>(c); };
  if (!c.audit_directory.empty()) {
    if (!std::filesystem::create_directory(c.audit_directory))
      throw Failure(INVALID_GOAL, "audit directory must be new");
    command_audit_.open(c.audit_directory + "/commands.csv");
    feedback_audit_.open(c.audit_directory + "/feedback.bin", std::ios::binary);
    if (!command_audit_ || !feedback_audit_)
      throw Failure(INVALID_GOAL, "cannot open audit files");
    command_audit_ << "host_utc_ns,latency_s,command,reply_or_error\n";
  }
}
V4Driver::~V4Driver() = default;
std::string V4Driver::call(const std::string &command) {
  // Reuse a single ordered stream: per-command connection setup caused
  // intermittent query timeouts in the real burst test. The real controller
  // rejects a second command connection from this host, so Stop must use the
  // same stream. An in-flight call can delay Stop by its bounded I/O deadline;
  // callers must budget that latency. Never retry an ambiguous command.
  auto query = [&](CommandTransport &transport) {
    const auto start = Clock::now();
    auto audit = [&](const std::string &value) {
      if (!command_audit_.is_open())
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
      command_audit_
          << ns << "," << std::setprecision(9)
          << std::chrono::duration<double>(Clock::now() - start).count()
          << ",\"" << escape(command) << "\",\"" << escape(value) << "\"\n";
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
  if (!command_transport_)
    command_transport_ = transport_();
  try {
    return query(*command_transport_);
  } catch (const ControllerRejected &) {
    // A matching, well-formed negative reply does not corrupt the stream.
    // Keep its exclusive session available for the next IK seed or Stop.
    throw;
  } catch (...) {
    command_transport_.reset();
    throw;
  }
}
Transform V4Driver::fk(const Joints &q) {
  return sdkPose(
      six(call("PositiveKin(" + list(q, 180 / pi) + ",user=0,tool=0)")));
}
Joints V4Driver::ik(const Transform &flange, const Joints &seed) {
  const auto a = toSdkPose(flange);
  auto values =
      six(call("InverseKin(" + list(a, 1) + ",useJointNear=1,jointNear={" +
               list(seed, 180 / pi) + "},user=0,tool=0)"));
  for (auto &v : values)
    v *= pi / 180;
  return values;
}
Feedback V4Driver::readFeedback() {
  try {
    if (!feedback_socket_)
      feedback_socket_ = std::make_unique<Socket>(
          config_.ip, config_.feedback_port, config_.timeout_ms);
    // Overall deadline also covers duplicates. Duplicate packets do not refresh
    // host freshness.
    auto deadline =
        Clock::now() + std::chrono::milliseconds(config_.timeout_ms);
    while (Clock::now() < deadline) {
      FeedbackFrame b{};
      auto remaining =
          std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now())
              .count();
      feedback_socket_->exact(
          b.data(), b.size(),
          static_cast<int>(std::max<int64_t>(1, remaining)));
      if (feedback_audit_.is_open()) {
        const int64_t ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        feedback_audit_.write(reinterpret_cast<const char *>(&ns), sizeof(ns));
        feedback_audit_.write(reinterpret_cast<const char *>(b.data()),
                              b.size());
      }
      if (!sequence_.accept(b))
        continue;
      Feedback f;
      f.received = Clock::now();
      f.controller_stamp = feedback_field<uint64_t>(b, 32);
      f.mode = feedback_field<uint64_t>(b, 24);
      for (size_t i = 0; i < 6; ++i) {
        f.q[i] = feedback_field<double>(b, 432 + 8 * i) * pi / 180;
        f.speed[i] = feedback_field<double>(b, 480 + 8 * i) * pi / 180;
      }
      return f;
    }
    throw Failure(FEEDBACK_FAULT, "duplicate feedback deadline exceeded");
  } catch (...) {
    feedback_socket_.reset();
    throw;
  }
}
void V4Driver::servo(const Joints &q, double period) {
  if (!config_.hardwareAllowed())
    throw Failure(HARDWARE_DISABLED, "ServoJ gated by hardware prerequisites");
  if (!std::isfinite(period) || period < 0.004 || period > 3600)
    throw Failure(INVALID_GOAL, "invalid ServoJ period");
  auto payload = call("ServoJ(" + list(q, 180 / pi) + ",t=" + number(period) +
                      ",aheadtime=" + number(config_.servo_ahead) +
                      ",gain=" + number(config_.servo_gain) + ")");
  // Queue ID is an acknowledgement, never an arrival assertion.
  uint64_t id = 0;
  auto r = std::from_chars(payload.data(), payload.data() + payload.size(), id);
  if (r.ec != std::errc() || r.ptr != payload.data() + payload.size())
    throw Failure(EXECUTION_FAILED, "invalid ServoJ queue ID");
}
void V4Driver::stop() {
  if (!config_.hardwareAllowed())
    throw Failure(HARDWARE_DISABLED,
                  "Stop gated; passive backend cannot actuate");
  if (!call("Stop()").empty())
    throw Failure(STOP_FAILED, "unexpected Stop payload");
}
} // namespace chess_robot_arm
