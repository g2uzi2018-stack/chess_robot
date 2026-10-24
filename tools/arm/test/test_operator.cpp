#include "arm_tools/net.hpp"
#include "arm_tools/operator.hpp"
#include <algorithm>
#include <fstream>
#include <gtest/gtest.h>
#include <thread>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

using namespace arm_tools;
namespace {
Settings settings() {
  Settings s;
  s.capture_s = .1;
  s.min_frames = 3;
  s.max_gap_s = .05;
  s.deadline_s = .05;
  return s;
}
Feedback feedback(uint64_t mode = 5) {
  Feedback f;
  f.mode = mode;
  f.received = Clock::now();
  return f;
}
struct Fixture {
  uint64_t mode = 4;
  bool tcp = true, fail_enable = false;
  std::vector<std::string> commands;
  Feedback read() { return feedback(mode); }
  std::string query(const std::string &command) {
    commands.push_back(command);
    if (command == "RequestControl()")
      tcp = true;
    else if (!tcp)
      throw QueryFailure("notice", "Control Mode Is Not Tcp\t");
    else if (command == "ClearError()")
      mode = 4;
    else if (command == "EnableRobot()") {
      if (fail_enable)
        return "-1,{},EnableRobot();";
      mode = 5;
    }
    return "0,{" + (command == "RobotMode()" ? std::to_string(mode) : "") +
           "}," + command + ";";
  }
};
struct Directory {
  std::filesystem::path path;
  Directory() {
    char p[] = "/tmp/arm_operator_XXXXXX";
    auto *result = mkdtemp(p);
    if (!result)
      throw std::runtime_error("temp directory");
    path = result;
  }
  ~Directory() {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
};
} // namespace
TEST(Operator, PreviewNeverChangesState) {
  Fixture f;
  Administrator admin(
      settings(), [&] { return f.read(); },
      [&](const auto &c) { return f.query(c); });
  EXPECT_NE(admin.prepare(true, false).find("PREVIEW"), std::string::npos);
  EXPECT_EQ(f.commands, std::vector<std::string>{"RobotMode()"});
  EXPECT_EQ(f.mode, 4u);
}
TEST(Operator, TcpRequestThenClearAndEnable) {
  Fixture f;
  f.tcp = false;
  Administrator admin(
      settings(), [&] { return f.read(); },
      [&](const auto &c) { return f.query(c); });
  EXPECT_NE(admin.prepare(true, true).find("READY"), std::string::npos);
  auto request =
      std::find(f.commands.begin(), f.commands.end(), "RequestControl()");
  auto clear = std::find(f.commands.begin(), f.commands.end(), "ClearError()");
  auto enable =
      std::find(f.commands.begin(), f.commands.end(), "EnableRobot()");
  EXPECT_LT(request, clear);
  EXPECT_LT(clear, enable);
  EXPECT_EQ(f.mode, 5u);
}
TEST(Operator, EnabledNonTcpDoesNotDisableOrRequestControl) {
  Fixture f;
  f.mode = 5;
  f.tcp = false;
  Administrator admin(
      settings(), [&] { return f.read(); },
      [&](const auto &c) { return f.query(c); });
  EXPECT_THROW(admin.prepare(true, true), std::runtime_error);
  EXPECT_EQ(f.commands, std::vector<std::string>{"RobotMode()"});
}
TEST(Operator, EnableRejectionIsNotReady) {
  Fixture f;
  f.fail_enable = true;
  Administrator admin(
      settings(), [&] { return f.read(); },
      [&](const auto &c) { return f.query(c); });
  EXPECT_THROW(admin.prepare(false, true), std::runtime_error);
  EXPECT_EQ(f.mode, 4u);
}
TEST(Operator, MovingFeedbackBlocksAllCommands) {
  Fixture f;
  Administrator admin(
      settings(),
      [] {
        auto x = feedback();
        x.speed[0] = .1;
        return x;
      },
      [&](const auto &c) { return f.query(c); });
  EXPECT_THROW(admin.prepare(true, true), std::runtime_error);
  EXPECT_TRUE(f.commands.empty());
}
TEST(Operator, CaptureHasMedianAndRejectsMotionOrTimestampRegression) {
  Directory d;
  auto s = settings();
  uint64_t stamp = 0;
  auto c = collect(
      s,
      [&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto f = feedback();
        f.controller_stamp = ++stamp;
        f.q.fill(.1);
        return f;
      },
      d.path / "good", [] { return false; });
  EXPECT_GE(c.frames, 3u);
  EXPECT_DOUBLE_EQ(c.median[0], .1);
  EXPECT_DOUBLE_EQ(c.span[0], 0);
  EXPECT_THROW(collect(
                   s,
                   [] {
                     auto f = feedback();
                     f.speed[2] = .2;
                     return f;
                   },
                   d.path / "moving", [] { return false; }),
               std::runtime_error);
  EXPECT_THROW(collect(
                   s, [] { return feedback(); }, d.path / "duplicate",
                   [] { return false; }),
               std::runtime_error);
}
TEST(Operator, CandidateDoesNotOverwriteAndApplyPreservesBackup) {
  Directory d;
  auto canonical = d.path / "home.yaml";
  {
    std::ofstream out(canonical);
    out << "chess_robot_arm:\n  ros__parameters:\n    home:\n      taught: "
           "false\n";
  }
  auto original = YAML::LoadFile(canonical.string());
  Capture c;
  c.frames = 100;
  c.median.fill(.25);
  std::filesystem::create_directory(d.path / "candidate");
  saveHome(c, d.path / "candidate", canonical, false);
  EXPECT_FALSE(
      YAML::LoadFile(canonical.string())["chess_robot_arm"]["ros__parameters"]
                                        ["home"]["taught"]
                                            .as<bool>());
  std::filesystem::create_directory(d.path / "apply");
  saveHome(c, d.path / "apply", canonical, true);
  auto saved = YAML::LoadFile(
      canonical.string())["chess_robot_arm"]["ros__parameters"]["home"];
  EXPECT_TRUE(saved["taught"].as<bool>());
  EXPECT_DOUBLE_EQ(saved["joint_positions_rad"][0].as<double>(), .25);
  EXPECT_TRUE(std::filesystem::exists(d.path / "apply/previous_home.yaml"));
  EXPECT_TRUE(std::filesystem::exists(d.path / "home_reference.json"));
  EXPECT_FALSE(std::filesystem::exists(d.path / ".home_update_pending"));
}

TEST(Home, PreviewDoesNotSendAnyCommand) {
  auto s = settings();
  size_t calls = 0;
  EXPECT_NE(returnHome(
                s, Joints{}, [] { return feedback(); },
                [&](const auto &) {
                  ++calls;
                  return std::string();
                },
                false, [] { return false; })
                .find("PREVIEW"),
            std::string::npos);
  EXPECT_EQ(calls, 0u);
}
TEST(Home, RequiresQueueAndMeasuredArrival) {
  auto s = settings();
  s.arrival_dwell_s = .005;
  Joints target{};
  target.fill(.1);
  bool moved = false;
  std::vector<std::string> commands;
  auto result = returnHome(
      s, target,
      [&] {
        auto f = feedback();
        if (moved)
          f.q = target;
        return f;
      },
      [&](const std::string &command) {
        commands.push_back(command);
        if (command.find("MovJ(") == 0)
          moved = true;
        return "0,{" +
               (command == "RobotMode()" ? std::string("5")
                                         : std::string("7")) +
               "}," + command + ";";
      },
      true, [] { return false; });
  EXPECT_NE(result.find("HOME_REACHED"), std::string::npos);
  EXPECT_NE(
      std::find(commands.begin(), commands.end(), "GetCurrentCommandID()"),
      commands.end());
  EXPECT_EQ(std::find(commands.begin(), commands.end(), "Stop()"),
            commands.end());
}
TEST(Home, AmbiguousAckStopsAndVerifies) {
  auto s = settings();
  s.arrival_dwell_s = .005;
  bool stopped = false;
  try {
    returnHome(
        s, Joints{}, [] { return feedback(); },
        [&](const std::string &command) {
          if (command == "Stop()") {
            stopped = true;
            return std::string("0,{},Stop();");
          }
          return "0,{" +
                 (command == "RobotMode()" ? std::string("5")
                                           : std::string("bad")) +
                 "}," + command + ";";
        },
        true, [] { return false; });
    FAIL();
  } catch (const std::exception &e) {
    EXPECT_NE(std::string(e.what()).find("STOP_VERIFIED"), std::string::npos);
  }
  EXPECT_TRUE(stopped);
}
TEST(Home, InterruptAfterSendStops) {
  auto s = settings();
  s.arrival_dwell_s = .005;
  bool sent = false, stopped = false;
  EXPECT_THROW(returnHome(
                   s, Joints{}, [] { return feedback(); },
                   [&](const std::string &command) {
                     if (command.find("MovJ(") == 0)
                       sent = true;
                     if (command == "Stop()") {
                       stopped = true;
                       return std::string("0,{},Stop();");
                     }
                     return "0,{" +
                            (command == "RobotMode()" ? std::string("5")
                                                      : std::string("7")) +
                            "}," + command + ";";
                   },
                   true, [&] { return sent; }),
               std::runtime_error);
  EXPECT_TRUE(stopped);
}
