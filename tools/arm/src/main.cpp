#include "arm_tools/net.hpp"
#include "arm_tools/operator.hpp"
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <set>
#include <sys/file.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
struct Args {
  std::string operation;
  std::map<std::string, std::string> values;
  std::set<std::string> flags;
};
Args parse(int argc, char **argv) {
  if (argc < 2)
    throw std::runtime_error(
        "expected prepare | home | capture-home; use --help");
  Args a;
  a.operation = argv[1];
  const std::set<std::string> flags{"--apply", "--request-tcp", "--execute",
                                    "--help"};
  const std::set<std::string> values{"--config",         "--ip",
                                     "--dashboard-port", "--feedback-port",
                                     "--home-file",      "--output-dir"};
  for (int i = 2; i < argc; ++i) {
    std::string key = argv[i];
    if (flags.count(key))
      a.flags.insert(key);
    else if (values.count(key) && i + 1 < argc)
      a.values[key] = argv[++i];
    else
      throw std::runtime_error("unknown/missing argument: " + key);
  }
  if (a.operation != "prepare" && a.operation != "home" &&
      a.operation != "capture-home" && a.operation != "--help")
    throw std::runtime_error("unknown operation");
  if (a.flags.count("--execute") && a.operation != "home")
    throw std::runtime_error("--execute is only for home");
  if (a.flags.count("--apply") && a.operation == "home")
    throw std::runtime_error("use --execute for Home motion");
  if (a.flags.count("--request-tcp") && a.operation != "prepare")
    throw std::runtime_error("--request-tcp is only for prepare");
  return a;
}
std::string value(const Args &a, const std::string &key,
                  const std::string &fallback = "") {
  auto it = a.values.find(key);
  return it == a.values.end() ? fallback : it->second;
}
template <class T> T setting(const YAML::Node &n, const char *key, T fallback) {
  return n[key] ? n[key].as<T>() : fallback;
}
int integer(const std::string &text) {
  size_t used = 0;
  int result = std::stoi(text, &used);
  if (used != text.size())
    throw std::runtime_error("invalid integer");
  return result;
}
void help() {
  std::cout
      << "Independent C++ Dobot V4 maintenance tools; no ROS or business "
         "libraries.\n"
      << "prepare --config FILE [--request-tcp] [--apply]\n"
      << "home --config FILE --home-file YAML [--execute]\n"
      << "capture-home --config FILE --home-file YAML --output-dir NEW_DIR "
         "[--apply]\n"
      << "Network overrides: --ip IPv4 --dashboard-port N --feedback-port N\n"
      << "Defaults: prepare previews; home previews; capture saves a "
         "candidate.\n";
}
struct Lock {
  int fd = -1;
  explicit Lock(const std::string &ip) {
    auto path =
        "/tmp/arm_tools_" + std::to_string(getuid()) + "_" + ip + ".lock";
    fd = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (fd < 0 || flock(fd, LOCK_EX | LOCK_NB)) {
      if (fd >= 0)
        close(fd);
      throw std::runtime_error(
          "another maintenance tool is using this controller");
    }
  }
  ~Lock() {
    if (fd >= 0)
      close(fd);
  }
};
} // namespace
int main(int argc, char **argv) {
  using namespace arm_tools;
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  try {
    auto a = parse(argc, argv);
    if (a.operation == "--help" || a.flags.count("--help")) {
      help();
      return 0;
    }
    if (value(a, "--config").empty())
      throw std::runtime_error("--config is required");
    auto config = YAML::LoadFile(value(a, "--config"));
    Settings s;
    auto n = config["controller"];
    s.controller.ip =
        value(a, "--ip", setting<std::string>(n, "ip", "127.0.0.1"));
    s.controller.dashboard_port =
        integer(value(a, "--dashboard-port",
                      std::to_string(setting(n, "dashboard_port", 29999))));
    s.controller.feedback_port =
        integer(value(a, "--feedback-port",
                      std::to_string(setting(n, "feedback_port", 30004))));
    s.controller.timeout_ms = setting(n, "timeout_ms", 500);
    s.deadline_s = setting(config, "transition_timeout_s", 10.);
    auto c = config["capture"];
    s.capture_s = setting(c, "duration_s", 3.);
    s.min_frames = setting<size_t>(c, "min_frames", 100);
    s.max_gap_s = setting(c, "max_gap_s", .25);
    s.stationary_rad_s = setting(c, "stationary_rad_s", .01);
    s.max_span_rad = setting(c, "max_span_rad", .003);
    auto h = config["home"];
    s.home_velocity_percent = setting(h, "velocity_percent", 5);
    s.home_acceleration_percent = setting(h, "acceleration_percent", 5);
    s.home_timeout_s = setting(h, "timeout_s", 120.);
    s.home_joint_tolerance_rad = setting(h, "joint_tolerance_rad", .015);
    s.arrival_dwell_s = setting(h, "arrival_dwell_s", .3);
    s.stop_timeout_s = setting(h, "stop_timeout_s", 3.);
    auto limits = config["limits"];
    for (const auto *key : {"lower_rad", "upper_rad"})
      if (!limits[key] || !limits[key].IsSequence() || limits[key].size() != 6)
        throw std::runtime_error("six configured joint limits required");
    for (size_t j = 0; j < 6; ++j) {
      s.lower[j] = limits["lower_rad"][j].as<double>();
      s.upper[j] = limits["upper_rad"][j].as<double>();
    }
    s.validate();
    Lock lock(s.controller.ip);
    FeedbackReader feedback(s.controller);
    Reader reader = [&] { return feedback.read(); };
    Query query = [&](const std::string &command) {
      Socket socket(s.controller.ip, s.controller.dashboard_port,
                    s.controller.timeout_ms);
      return socket.query(command, s.controller.timeout_ms);
    };
    if (a.operation == "prepare") {
      Administrator admin(
          s,
          [&] {
            if (interrupted)
              throw std::runtime_error("preparation interrupted");
            return reader();
          },
          query);
      std::cout << admin.prepare(a.flags.count("--request-tcp"),
                                 a.flags.count("--apply"))
                << '\n';
      return 0;
    }
    const auto canonical = value(a, "--home-file");
    if (canonical.empty())
      throw std::runtime_error("--home-file required");
    auto marker =
        std::filesystem::path(canonical).parent_path() / ".home_update_pending";
    if (std::filesystem::exists(marker))
      throw std::runtime_error("unfinished Home update; inspect capture "
                               "backups and marker before use");
    if (a.operation == "home") {
      auto yaml = YAML::LoadFile(canonical);
      auto home = yaml["chess_robot_arm"]["ros__parameters"]["home"];
      if (!home["taught"].as<bool>() || home["joint_names"].size() != 6 ||
          home["joint_positions_rad"].size() != 6)
        throw std::runtime_error("taught six-joint Home required");
      Joints q;
      for (size_t j = 0; j < 6; ++j) {
        if (home["joint_names"][j].as<std::string>() !=
            "joint" + std::to_string(j + 1))
          throw std::runtime_error("unexpected Home joint order");
        q[j] = home["joint_positions_rad"][j].as<double>();
      }
      std::cout << returnHome(s, q, reader, query, a.flags.count("--execute"),
                              [] { return interrupted != 0; })
                << '\n';
      return 0;
    }
    const auto output = value(a, "--output-dir");
    if (output.empty())
      throw std::runtime_error("--output-dir required");
    auto captured = collect(
        s,
        [&] {
          auto f = reader();
          for (size_t j = 0; j < 6; ++j)
            if (f.q[j] < s.lower[j] || f.q[j] > s.upper[j])
              throw std::runtime_error(
                  "captured joint outside configured limits");
          return f;
        },
        output, [] { return interrupted != 0; });
    if (interrupted)
      throw std::runtime_error("capture interrupted before save");
    saveHome(captured, output, canonical, a.flags.count("--apply"));
    std::cout << "Captured " << captured.frames << " frames: " << output
              << "/home.yaml\n"
              << (a.flags.count("--apply")
                      ? "Canonical Home updated; previous files backed up. "
                        "Reload any consumers explicitly. No motion or "
                        "winding-reference reset performed.\n"
                      : "Candidate saved; canonical Home unchanged.\n");
    return 0;
  } catch (const QueryFailure &e) {
    std::cerr << "operator failed: " << e.what()
              << "; partial reply=" << e.partial_reply << '\n';
  } catch (const std::exception &e) {
    std::cerr << "operator failed: " << e.what() << '\n';
  }
  return 2;
}
