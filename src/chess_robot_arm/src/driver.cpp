#include "chess_robot_arm/driver.hpp"
#include "chess_robot_arm/dashboard_reply.hpp"
#include "chess_robot_arm/net.hpp"
#include <iomanip>
#include <locale>
#include <sstream>

namespace chess_robot_arm {
namespace {
class TcpTransport final : public CommandTransport {
  Socket socket_;
  int timeout_;

public:
  explicit TcpTransport(const Config &c)
      : socket_(c.ip, c.dashboard_port, c.timeout_ms), timeout_(c.timeout_ms) {}
  std::string query(const std::string &c) override {
    return socket_.query(c, timeout_);
  }
};
std::string number(double x) {
  if (!std::isfinite(x))
    throw Failure(INVALID_GOAL, "nonfinite SDK argument");
  std::ostringstream s;
  s.imbue(std::locale::classic());
  s << std::setprecision(17) << x;
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
}
V4Driver::~V4Driver() = default;
std::string V4Driver::call(const std::string &command) {
  // One bounded connection per command prevents a delayed reply poisoning a
  // later query, and gives Stop its own connection rather than a planning
  // transport mutex.
  auto transport = transport_();
  std::string raw;
  try {
    raw = transport->query(command);
  } catch (const QueryFailure &e) {
    throw Failure(EXECUTION_FAILED, std::string(e.what()) +
                                        "; partial reply: " + e.partial_reply);
  }
  auto reply = parse_dashboard_reply(raw, command);
  if (reply.error != 0)
    throw Failure(EXECUTION_FAILED, "controller error " +
                                        std::to_string(reply.error) + " for " +
                                        command);
  return reply.payload;
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
