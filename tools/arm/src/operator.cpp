#include "arm_tools/operator.hpp"
#include "arm_tools/dashboard_reply.hpp"
#include "arm_tools/feedback_frame.hpp"
#include "arm_tools/net.hpp"
#include <algorithm>
#include <charconv>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace arm_tools {
namespace {
void require(bool condition, const std::string &why) {
  if (!condition)
    throw std::runtime_error(why);
}
double speed(const Feedback &f) {
  double v = 0;
  for (double x : f.speed) {
    require(std::isfinite(x), "nonfinite measured speed");
    v = std::max(v, std::abs(x));
  }
  return v;
}
void idle(const Feedback &f, const Settings &s) {
  require(f.mode == 4 || f.mode == 5,
          "Home capture requires disabled or enabled-idle mode (4/5)");
  require(speed(f) <= s.stationary_rad_s, "arm is moving; capture refused");
  for (double q : f.q)
    require(std::isfinite(q), "nonfinite measured joint");
}
void write(const std::filesystem::path &p, const std::string &bytes) {
  std::ofstream out(p, std::ios::binary | std::ios::trunc);
  out.exceptions(std::ios::failbit | std::ios::badbit);
  out << bytes;
  out.flush();
  out.close();
}
void replace(const std::filesystem::path &p, const std::string &bytes) {
  auto temporary = p;
  temporary += ".tmp." + std::to_string(getpid());
  try {
    write(temporary, bytes);
    std::filesystem::rename(temporary, p);
  } catch (...) {
    std::error_code ec;
    std::filesystem::remove(temporary, ec);
    throw;
  }
}
std::string readFile(const std::filesystem::path &p) {
  std::ifstream in(p, std::ios::binary);
  require(bool(in), "cannot read " + p.string());
  return {std::istreambuf_iterator<char>(in), {}};
}
} // namespace
void Settings::validate() const {
  in_addr address{};
  require(inet_pton(AF_INET, controller.ip.c_str(), &address) == 1,
          "IPv4 address required");
  require(controller.dashboard_port > 0 && controller.dashboard_port < 65536 &&
              controller.feedback_port > 0 && controller.feedback_port < 65536,
          "invalid ports");
  require(controller.timeout_ms > 0 && controller.timeout_ms <= 5000,
          "invalid socket timeout");
  for (double x :
       {deadline_s, capture_s, max_gap_s, stationary_rad_s, max_span_rad})
    require(std::isfinite(x) && x > 0,
            "operator durations/tolerances must be positive finite");
  require(deadline_s <= 60 && capture_s <= 30 && capture_s >= .1 &&
              max_gap_s <= capture_s && min_frames >= 2 && min_frames <= 100000,
          "invalid capture/deadline bounds");
  require(home_velocity_percent >= 1 && home_velocity_percent <= 100 &&
              home_acceleration_percent >= 1 &&
              home_acceleration_percent <= 100,
          "invalid Home speed/acceleration percentage");
  for (double x : {home_timeout_s, home_joint_tolerance_rad, arrival_dwell_s,
                   stop_timeout_s})
    require(std::isfinite(x) && x > 0, "invalid Home tolerances/timeouts");
  require(home_timeout_s <= 3600 && stop_timeout_s <= 30 &&
              stop_timeout_s >
                  arrival_dwell_s + 2 * controller.timeout_ms / 1000.,
          "invalid Home timing bounds");
  for (size_t j = 0; j < 6; ++j)
    require(std::isfinite(lower[j]) && std::isfinite(upper[j]) &&
                lower[j] < upper[j],
            "invalid joint bounds");
}
struct FeedbackReader::Impl {
  Network config;
  std::unique_ptr<Socket> socket;
  FeedbackSequence sequence;
  explicit Impl(const Network &n) : config(n) {}
};
FeedbackReader::FeedbackReader(const Network &n)
    : impl_(std::make_unique<Impl>(n)) {}
FeedbackReader::~FeedbackReader() = default;
Feedback FeedbackReader::read() {
  auto &i = *impl_;
  if (!i.socket)
    i.socket = std::make_unique<Socket>(i.config.ip, i.config.feedback_port,
                                        i.config.timeout_ms);
  auto deadline = Clock::now() + std::chrono::milliseconds(i.config.timeout_ms);
  std::optional<Feedback> latest;
  size_t drained = 0;
  while (Clock::now() < deadline) {
    if (latest && !i.socket->hasInput())
      return *latest;
    require(++drained <= 512, "excessive feedback backlog; freshness unknown");
    FeedbackFrame bytes{};
    auto left =
        std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now())
            .count();
    i.socket->exact(bytes.data(), bytes.size(), std::max<int64_t>(1, left));
    if (!i.sequence.accept(bytes))
      continue;
    Feedback f;
    f.received = Clock::now();
    f.mode = feedback_field<uint64_t>(bytes, 24);
    f.controller_stamp = feedback_field<uint64_t>(bytes, 32);
    for (size_t j = 0; j < 6; ++j) {
      f.q[j] = feedback_field<double>(bytes, 432 + 8 * j) * pi / 180;
      f.speed[j] = feedback_field<double>(bytes, 480 + 8 * j) * pi / 180;
    }
    latest = f;
  }
  throw std::runtime_error("duplicate feedback deadline exceeded");
}
Administrator::Administrator(Settings s, Reader r, Query q)
    : settings_(std::move(s)), read_(std::move(r)), query_(std::move(q)) {
  settings_.validate();
}
std::string Administrator::call(const std::string &command) {
  auto reply = parse_dashboard_reply(query_(command), command);
  require(reply.error == 0, "controller rejected " + command + ": " +
                                std::to_string(reply.error));
  return reply.payload;
}
uint64_t Administrator::mode() {
  auto payload = call("RobotMode()");
  uint64_t value = 0;
  auto parsed =
      std::from_chars(payload.data(), payload.data() + payload.size(), value);
  require(parsed.ec == std::errc() &&
              parsed.ptr == payload.data() + payload.size() && value >= 1 &&
              value <= 11,
          "invalid RobotMode payload");
  return value;
}
void Administrator::stationary() {
  auto f = read_();
  require(Clock::now() - f.received <=
              std::chrono::duration<double>(settings_.max_gap_s),
          "stale feedback");
  require(speed(f) <= settings_.stationary_rad_s,
          "moving arm: preparation refused");
  require(f.mode == 3 || f.mode == 4 || f.mode == 5 || f.mode == 9 ||
              f.mode == 11,
          "unsupported mode: no automatic pause/drag/brake recovery");
}
void Administrator::waitMode(const std::function<bool(uint64_t)> &predicate) {
  auto deadline =
      Clock::now() + std::chrono::duration<double>(settings_.deadline_s);
  do {
    auto f = read_();
    require(speed(f) <= settings_.stationary_rad_s,
            "motion observed during preparation");
    if (predicate(mode()) && predicate(f.mode))
      return;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while (Clock::now() < deadline);
  throw std::runtime_error("controller state transition deadline exceeded");
}
std::string Administrator::prepare(bool request_tcp, bool apply) {
  stationary();
  auto f = read_();
  bool tcp = true;
  uint64_t current = f.mode;
  try {
    current = mode();
  } catch (const QueryFailure &e) {
    if (reply_normalize(e.partial_reply).find("controlmodeisnottcp") ==
        std::string::npos)
      throw;
    tcp = false;
  }
  if (!apply)
    return "PREVIEW: mode=" + std::to_string(current) +
           ", TCP query=" + (tcp ? "available" : "unavailable") +
           "; no state-changing commands sent";
  if (!tcp) {
    require(request_tcp, "not in TCP mode; use --request-tcp while disabled, "
                         "or switch on teach pendant");
    require(f.mode == 3 || f.mode == 4,
            "RequestControl requires unpowered/disabled state; disable on "
            "teach pendant first");
    require(call("RequestControl()").empty(),
            "unexpected RequestControl payload");
    waitMode([](uint64_t m) { return m == 3 || m == 4; });
    current = mode();
  }
  require(current != 3,
          "robot is unpowered; PowerOn is not performed automatically");
  stationary();
  require(call("ClearError()").empty(), "unexpected ClearError payload");
  waitMode([](uint64_t m) { return m == 4 || m == 5; });
  if (mode() == 4) {
    stationary();
    require(call("EnableRobot()").empty(), "unexpected EnableRobot payload");
  }
  waitMode([](uint64_t m) { return m == 5; });
  return "READY: TCP commands available, errors cleared, mode=5 enabled and "
         "idle; payload settings unchanged";
}
Capture collect(const Settings &s, Reader read,
                const std::filesystem::path &directory,
                const std::function<bool()> &interrupted) {
  s.validate();
  require(!std::filesystem::exists(directory),
          "capture directory already exists");
  std::filesystem::create_directories(directory);
  std::ofstream csv(directory / "feedback.csv");
  csv.exceptions(std::ios::failbit | std::ios::badbit);
  csv << std::setprecision(17) << "host_steady_ns,controller_stamp,mode";
  for (int j = 1; j <= 6; ++j)
    csv << ",q" << j << "_rad";
  csv << '\n';
  std::vector<Joints> samples;
  auto first = Clock::now(), previous = first;
  uint64_t stamp = 0;
  while (Clock::now() - first < std::chrono::duration<double>(s.capture_s)) {
    require(!interrupted(), "capture interrupted; canonical Home unchanged");
    auto f = read();
    auto now = Clock::now();
    idle(f, s);
    require(now - f.received <= std::chrono::duration<double>(s.max_gap_s) &&
                f.received - previous <=
                    std::chrono::duration<double>(s.max_gap_s),
            "feedback gap during capture");
    require(samples.empty() || f.controller_stamp > stamp,
            "non-increasing capture timestamp");
    stamp = f.controller_stamp;
    previous = f.received;
    samples.push_back(f.q);
    require(samples.size() <= 100000, "capture frame cap exceeded");
    csv << std::chrono::duration_cast<std::chrono::nanoseconds>(
               f.received.time_since_epoch())
               .count()
        << ',' << stamp << ',' << f.mode;
    for (double q : f.q)
      csv << ',' << q;
    csv << '\n';
  }
  require(samples.size() >= s.min_frames,
          "insufficient fresh stationary frames");
  Capture result;
  result.frames = samples.size();
  for (size_t j = 0; j < 6; ++j) {
    std::vector<double> axis;
    for (const auto &q : samples)
      axis.push_back(q[j]);
    std::sort(axis.begin(), axis.end());
    result.span[j] = axis.back() - axis.front();
    require(result.span[j] <= s.max_span_rad,
            "joint drift exceeds capture tolerance");
    size_t n = axis.size();
    result.median[j] =
        n % 2 ? axis[n / 2] : (axis[n / 2 - 1] + axis[n / 2]) / 2;
  }
  return result;
}
void saveHome(const Capture &c, const std::filesystem::path &directory,
              const std::filesystem::path &canonical, bool apply) {
  require(c.frames >= 2, "invalid Home capture");
  YAML::Node root;
  if (std::filesystem::exists(canonical))
    root = YAML::LoadFile(canonical.string());
  auto home = root["chess_robot_arm"]["ros__parameters"]["home"];
  YAML::Node names, positions;
  for (size_t j = 0; j < 6; ++j) {
    require(std::isfinite(c.median[j]) && std::isfinite(c.span[j]) &&
                c.span[j] >= 0,
            "invalid capture result");
    names.push_back("joint" + std::to_string(j + 1));
    positions.push_back(c.median[j]);
  }
  home["joint_names"] = names;
  home["joint_positions_rad"] = positions;
  home["taught"] = true;
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << root;
  require(emitter.good(), "YAML serialization failed");
  std::string yaml = std::string(emitter.c_str()) + "\n";
  std::ostringstream json;
  json << std::setprecision(17)
       << "{\n  \"schema_version\": 2,\n  \"source\": "
          "\"operator_stationary_feedback_capture\",\n  \"capture_frames\": "
       << c.frames << ",\n  \"joint_positions_rad\": [";
  for (size_t j = 0; j < 6; ++j)
    json << (j ? ", " : "") << c.median[j];
  json << "],\n  \"joint_span_rad\": [";
  for (size_t j = 0; j < 6; ++j)
    json << (j ? ", " : "") << c.span[j];
  json << "],\n  \"capture_csv\": \"feedback.csv\",\n  "
          "\"physical_cable_reference_certified\": false,\n  "
          "\"automatic_return_path_validated\": false\n}\n";
  write(directory / "home.yaml", yaml);
  write(directory / "home_reference.json", json.str());
  if (!apply)
    return;
  require(std::filesystem::is_regular_file(canonical),
          "canonical Home must already exist; supply --home-file");
  auto metadata = canonical.parent_path() / "home_reference.json";
  const auto old_yaml = readFile(canonical);
  const bool had_metadata = std::filesystem::exists(metadata);
  auto old_metadata = had_metadata ? readFile(metadata) : std::string();
  write(directory / "previous_home.yaml", old_yaml);
  if (had_metadata)
    write(directory / "previous_home_reference.json", old_metadata);
  // A marker makes an interrupted two-file update observable at the next
  // launch.
  auto marker = canonical.parent_path() / ".home_update_pending";
  require(!std::filesystem::exists(marker),
          "unfinished Home update exists; inspect capture backups first");
  write(marker, std::filesystem::absolute(directory).string() + "\n");
  try {
    replace(canonical, yaml);
    replace(metadata, json.str());
    std::filesystem::remove(marker);
  } catch (...) {
    replace(canonical, old_yaml);
    if (had_metadata)
      replace(metadata, old_metadata);
    else
      std::filesystem::remove(metadata);
    std::filesystem::remove(marker);
    throw;
  }
}

std::string returnHome(const Settings &s, const Joints &target, Reader read,
                       Query query, bool execute,
                       const std::function<bool()> &interrupted) {
  s.validate();
  for (size_t j = 0; j < 6; ++j)
    require(std::isfinite(target[j]) && target[j] >= s.lower[j] &&
                target[j] <= s.upper[j],
            "Home joint outside configured position bounds");
  auto call = [&](const std::string &command) {
    auto reply = parse_dashboard_reply(query(command), command);
    require(reply.error == 0, "controller rejected " + command + ": " +
                                  std::to_string(reply.error));
    return reply.payload;
  };
  auto number = [](const std::string &text) {
    uint64_t result = 0;
    auto parsed =
        std::from_chars(text.data(), text.data() + text.size(), result);
    require(parsed.ec == std::errc() && parsed.ptr == text.data() + text.size(),
            "invalid numeric controller payload");
    return result;
  };
  auto fresh = [&] {
    auto f = read();
    require(Clock::now() - f.received <=
                std::chrono::duration<double>(s.max_gap_s),
            "stale Home feedback");
    for (size_t j = 0; j < 6; ++j)
      require(std::isfinite(f.q[j]) && f.q[j] >= s.lower[j] &&
                  f.q[j] <= s.upper[j],
              "measured joint outside configured bounds");
    return f;
  };
  auto initial = fresh();
  require(speed(initial) <= s.stationary_rad_s,
          "Home requires stationary start");
  std::ostringstream command;
  command.imbue(std::locale::classic());
  command << std::setprecision(17) << "MovJ(joint={";
  for (size_t j = 0; j < 6; ++j)
    command << (j ? "," : "") << target[j] * 180 / pi;
  command << "},user=0,tool=0,a=" << s.home_acceleration_percent
          << ",v=" << s.home_velocity_percent << ",cp=0)";
  if (!execute)
    return "PREVIEW: independent controller joint move, no motion sent: " +
           command.str();
  require(initial.mode == 5 && number(call("RobotMode()")) == 5,
          "Home requires TCP enabled-idle mode=5; run prepare separately");
  require(!interrupted(), "Home interrupted before submission");
  bool owned = true;
  try {
    // Ownership begins before sending: a missing ACK does not imply no motion.
    const auto id = number(call(command.str()));
    auto deadline =
        Clock::now() + std::chrono::duration<double>(s.home_timeout_s);
    std::optional<Clock::time_point> since;
    while (Clock::now() < deadline) {
      require(!interrupted(), "Home interrupted");
      auto f = fresh();
      require(f.mode == 5 || f.mode == 7,
              "controller left supported Home motion modes");
      double error = 0;
      for (size_t j = 0; j < 6; ++j)
        error = std::max(error, std::abs(f.q[j] - target[j]));
      const bool reached = f.mode == 5 && speed(f) <= s.stationary_rad_s &&
                           error <= s.home_joint_tolerance_rad;
      if (reached && number(call("GetCurrentCommandID()")) == id) {
        if (!since)
          since = Clock::now();
        if (Clock::now() - *since >=
            std::chrono::duration<double>(s.arrival_dwell_s)) {
          owned = false;
          return "HOME_REACHED: matching queue, fresh measured joints, low "
                 "speed and idle dwell verified";
        }
      } else
        since.reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("Home arrival timeout");
  } catch (const std::exception &error) {
    const std::string original = error.what();
    if (owned) {
      try {
        require(call("Stop()").empty(), "unexpected Stop payload");
        auto deadline =
            Clock::now() + std::chrono::duration<double>(s.stop_timeout_s);
        std::optional<Clock::time_point> since;
        while (Clock::now() < deadline) {
          auto f = fresh();
          if (f.mode == 5 && speed(f) <= s.stationary_rad_s) {
            if (!since)
              since = Clock::now();
            if (Clock::now() - *since >=
                std::chrono::duration<double>(s.arrival_dwell_s)) {
              owned = false;
              break;
            }
          } else
            since.reset();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        require(!owned, "stop could not be verified from feedback");
      } catch (const std::exception &stop) {
        throw std::runtime_error(original + "; STOP_FAILED: " + stop.what() +
                                 "; inspect controller before retrying");
      }
    }
    throw std::runtime_error(original + "; STOP_VERIFIED");
  }
}
} // namespace arm_tools
