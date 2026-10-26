#include "chess_robot_arm/core.hpp"
#include "chess_robot_arm/dashboard_reply.hpp"
#include "chess_robot_arm/driver.hpp"
#include "chess_robot_arm/geometry_guard.hpp"
#include "chess_robot_arm/net.hpp"
#include <Eigen/Dense>
#include <cstring>
#include <gtest/gtest.h>
#include <limits>
#include <thread>
#include <fstream>
#include <condition_variable>
#include <deque>

using namespace chess_robot_arm;
namespace {
struct StreamFixtureState {
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<std::string> queued;
  std::atomic<int> acknowledged{0};
  int sent=0, stops=0, delay_ms=90;
  std::vector<std::string> sent_commands;
  bool wrong_echo=false, repeat_id=false;
};
class StreamFixture : public CommandTransport {
  std::shared_ptr<StreamFixtureState> state_;
  std::optional<std::string> held_;
  Clock::time_point ready_;
public:
  explicit StreamFixture(std::shared_ptr<StreamFixtureState> s):state_(s){}
  bool streaming() const override {return true;}
  std::string query(const std::string &command) override {
    if (command!="Stop()") throw std::runtime_error("only Stop query expected");
    ++state_->stops; return "0,{},Stop();";
  }
  void send(const std::string &command) override {
    std::lock_guard<std::mutex> l(state_->mutex);
    if(command=="Stop()") ++state_->stops;
    else { ++state_->sent;state_->sent_commands.push_back(command); }
    state_->queued.push_back(command); state_->cv.notify_one();
  }
  std::string receive(int timeout_ms) override {
    if (!held_) {
      std::unique_lock<std::mutex> l(state_->mutex);
      if(!state_->cv.wait_for(l,std::chrono::milliseconds(timeout_ms),[&]{return !state_->queued.empty();}))
        throw std::runtime_error("fixture receive deadline");
      held_=state_->queued.front();state_->queued.pop_front();
      ready_=Clock::now()+std::chrono::milliseconds(state_->acknowledged ? 0 : state_->delay_ms);
    }
    std::this_thread::sleep_until(std::min(ready_,Clock::now()+std::chrono::milliseconds(timeout_ms)));
    if(Clock::now()<ready_) throw QueryFailure("socket deadline exceeded","");
    auto command=*held_;held_.reset();
    if(command=="Stop()") return "0,{},Stop();";
    int id=++state_->acknowledged;
    if(state_->repeat_id) id=1;
    return "0,{"+std::to_string(id)+"},"+(state_->wrong_echo ? "Other()" : command)+";";
  }
};
Config streamConfig() {
  Config c; c.servo_max_pending=4;c.motion_paused=false;c.controller_family="v4";c.controller_firmware="4.6.2.4-test-fixture"; c.backend="hardware";c.enable=c.tcp_verified=c.limits_verified=c.execution_verified=c.servo_certified=true;
  return c;
}
}
TEST(Streaming, DelayedAckDoesNotBlockSubsequentSends) {
  auto c=streamConfig();auto shared=std::make_shared<StreamFixtureState>();
  V4Driver driver(c,[shared]{return std::make_unique<StreamFixture>(shared);});
  driver.beginStream();
  for(int i=0;i<3;++i) driver.servo(c.home,.03);
  EXPECT_EQ(shared->sent,3);EXPECT_EQ(shared->acknowledged,0);
  EXPECT_NO_THROW(driver.endStream());EXPECT_EQ(shared->acknowledged,3);
  EXPECT_NO_THROW(driver.stop());EXPECT_EQ(shared->stops,1);
}
TEST(Streaming, ExplorationWindowCovers125msReplyWithoutChangingDeadline) {
  auto c=streamConfig();c.servo_max_pending=6;
  auto shared=std::make_shared<StreamFixtureState>();shared->delay_ms=125;
  V4Driver driver(c,[shared]{return std::make_unique<StreamFixture>(shared);});
  driver.beginStream();auto start=Clock::now();
  for(int i=0;i<5;++i) {
    if(i) std::this_thread::sleep_until(start+std::chrono::milliseconds(30*i));
    EXPECT_NO_THROW(driver.servo(c.home,.03));
  }
  EXPECT_NO_THROW(driver.endStream());EXPECT_EQ(shared->acknowledged,5);
  EXPECT_DOUBLE_EQ(c.servo_ack_timeout,.15);
  EXPECT_NO_THROW(driver.stop());EXPECT_EQ(shared->stops,1);
}
TEST(Streaming, LateReplyFailsMotionButPreservesExclusiveSessionForStop) {
  auto c=streamConfig();auto shared=std::make_shared<StreamFixtureState>();shared->delay_ms=200;
  int connections=0;
  V4Driver driver(c,[&] {
    if(++connections>1) throw std::runtime_error("IP:Port has been occupied");
    return std::make_unique<StreamFixture>(shared);
  });
  driver.beginStream();driver.servo(c.home,.03);
  std::this_thread::sleep_for(std::chrono::milliseconds(165));
  EXPECT_THROW(driver.checkStream(),Failure);
  EXPECT_NO_THROW(driver.stop());
  EXPECT_EQ(connections,1);EXPECT_EQ(shared->sent,1);
  EXPECT_EQ(shared->acknowledged,1);EXPECT_EQ(shared->stops,1);
}
TEST(Streaming, StopDrainCannotWaitForeverForMissingReply) {
  auto c=streamConfig();c.timeout_ms=100;c.stop_ack_timeout_ms=100;auto shared=std::make_shared<StreamFixtureState>();shared->delay_ms=2000;
  int connections=0;
  V4Driver driver(c,[&] {
    if(++connections>1) throw std::runtime_error("IP:Port has been occupied");
    return std::make_unique<StreamFixture>(shared);
  });
  driver.beginStream();driver.servo(c.home,.03);
  std::this_thread::sleep_for(std::chrono::milliseconds(165));
  auto start=Clock::now();EXPECT_THROW(driver.stop(),std::exception);
  EXPECT_LT(std::chrono::duration<double>(Clock::now()-start).count(),2.0);
  EXPECT_EQ(connections,1);EXPECT_EQ(shared->sent,1);EXPECT_EQ(shared->stops,1);
}
TEST(Streaming, WindowDeadlineAndEchoFaultsStopWithoutReplay) {
  for(int variant=0;variant<4;++variant) {
    auto c=streamConfig();auto shared=std::make_shared<StreamFixtureState>();
    if(variant==1) shared->delay_ms=300;
    if(variant==2) shared->wrong_echo=true;
    if(variant==3) shared->repeat_id=true;
    V4Driver driver(c,[shared]{return std::make_unique<StreamFixture>(shared);});
    driver.beginStream();
    for(int i=0;i<4;++i) driver.servo(c.home,.03);
    if(variant==0) EXPECT_THROW(driver.servo(c.home,.03),Failure);
    else EXPECT_THROW(driver.endStream(),Failure);
    EXPECT_NO_THROW(driver.stop());
    EXPECT_EQ(shared->sent,4);EXPECT_EQ(shared->stops,1);
  }
}
namespace {
Config config() {
  Config c;
  c.motion_paused=false;c.controller_family="v4";c.controller_firmware="4.6.2.4-test-fixture";
  c.default_profile="conservative";
  c.urdf = ARM_TEST_URDF;
  c.workspace_min = Eigen::Vector3d(-2, -2, -2);
  c.workspace_max = Eigen::Vector3d(2, 2, 2);
  c.flange_tcp.translation().z() = 0.2;
  c.flange_tcp.linear() =
      (Eigen::AngleAxisd(-pi / 2, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  c.dwell = 0.001;
  c.freshness = 1;
  c.clearance = 0.01;
  return c;
}
std::shared_ptr<const chess_robot_twist::Model> model(const Config &c) {
  return std::make_shared<chess_robot_twist::Model>(
      c.urdf, "base_link", "Link6", c.flange_tcp, c.home);
}
Feedback feedback(const Config &c, uint64_t stamp = 1) {
  Feedback f;
  f.q = c.home;
  f.mode = 5;
  f.controller_stamp = stamp;
  f.received = Clock::now();
  return f;
}
Snapshot trusted(const Config &c,
                 std::shared_ptr<const chess_robot_twist::Model> m) {
  Snapshot s{chess_robot_twist::Tracker(m, c.tracker_options)};
  s.tracker.initializeAtHome(c.home, c.home_tolerance);
  s.feedback = feedback(c);
  s.fresh = s.reference = true;
  return s;
}
// Genuine finite-difference Newton IK over the URDF FK; never an accept-all
// solver.
class OfflineKinematics final : public Kinematics {
  std::shared_ptr<const chess_robot_twist::Model> model_;
  Eigen::Matrix<double, 6, 1> residual(const Transform &a, const Transform &b) {
    Eigen::Matrix<double, 6, 1> e;
    e.head<3>() = b.translation() - a.translation();
    Eigen::AngleAxisd r(b.linear() * a.linear().transpose());
    e.tail<3>() = r.axis() * r.angle();
    return e;
  }

public:
  explicit OfflineKinematics(std::shared_ptr<const chess_robot_twist::Model> m)
      : model_(m) {}
  Transform fk(const Joints &q) override { return model_->flange(q); }
  Joints ik(const Transform &desired, const Joints &seed) override {
    auto q = seed;
    for (size_t iteration = 0; iteration < 150; ++iteration) {
      const auto current = fk(q);
      auto e = residual(current, desired);
      if (e.norm() < 1e-7)
        return q;
      Eigen::Matrix<double, 6, 6> jac;
      for (size_t j = 0; j < 6; ++j) {
        auto perturbed = q;
        perturbed[j] += 1e-6;
        jac.col(j) = residual(current, fk(perturbed)) / 1e-6;
      }
      Eigen::Matrix<double, 6, 1> step =
          jac.transpose() * (jac * jac.transpose() +
                             1e-6 * Eigen::Matrix<double, 6, 6>::Identity())
                                .ldlt()
                                .solve(e);
      if (!step.allFinite())
        break;
      if (step.norm() > 0.1)
        step *= 0.1 / step.norm();
      for (size_t j = 0; j < 6; ++j)
        q[j] += step[j];
      if (!model_->withinJointBounds(q))
        break;
    }
    throw Failure(PLANNING_FAILED, "offline IK did not converge");
  }
};
template <class T> void field(FeedbackFrame &f, size_t offset, T value) {
  std::memcpy(f.data() + offset, &value, sizeof(value));
}
FeedbackFrame frame(uint64_t stamp) {
  FeedbackFrame f{};
  field<uint16_t>(f, 0, 1440);
  field<uint64_t>(f, 48, 0x0123456789ABCDEFULL);
  field<uint64_t>(f, 32, stamp);
  return f;
}
class ReplyTransport final : public CommandTransport {
  std::function<std::string(const std::string &)> reply_;

public:
  explicit ReplyTransport(std::function<std::string(const std::string &)> r)
      : reply_(std::move(r)) {}
  std::string query(const std::string &c) override { return reply_(c); }
};
// The only socket opened by target tests binds/connects to IPv4 loopback.
class Loopback {
  int fd_ = -1;
  std::thread thread_;

public:
  int port = 0;
  explicit Loopback(
      const std::vector<std::string> &fragments,
      std::chrono::milliseconds delay = std::chrono::milliseconds(0)) {
    fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0)
      throw std::runtime_error("fixture socket");
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd_, reinterpret_cast<sockaddr *>(&a), sizeof(a)) ||
        listen(fd_, 1))
      throw std::runtime_error("fixture bind/listen");
    socklen_t len = sizeof(a);
    if (getsockname(fd_, reinterpret_cast<sockaddr *>(&a), &len))
      throw std::runtime_error("fixture getsockname");
    port = ntohs(a.sin_port);
    thread_ = std::thread([this, fragments, delay] {
      pollfd p{fd_, POLLIN, 0};
      if (poll(&p, 1, 2000) <= 0)
        return;
      int peer = accept(fd_, nullptr, nullptr);
      if (peer < 0)
        return;
      char command[256];
      pollfd in{peer, POLLIN, 0};
      if (poll(&in, 1, 1000) > 0)
        (void)recv(peer, command, sizeof(command), 0);
      std::this_thread::sleep_for(delay);
      for (const auto &f : fragments) {
        size_t off = 0;
        while (off < f.size()) {
          auto n = send(peer, f.data() + off, f.size() - off, MSG_NOSIGNAL);
          if (n <= 0)
            break;
          off += static_cast<size_t>(n);
        }
      }
      shutdown(peer, SHUT_RDWR);
      close(peer);
    });
  }
  ~Loopback() {
    if (thread_.joinable())
      thread_.join();
    if (fd_ >= 0)
      close(fd_);
  }
};
} // namespace
TEST(Protocol, EchoErrorsAndNestedPayload) {
  auto r = parse_dashboard_reply("0,{{1,2},{3}},RobotMode();", "RobotMode()");
  EXPECT_EQ(r.payload, "{1,2},{3}");
  EXPECT_EQ(parse_dashboard_reply("-1,{},Stop();", "Stop()").error, -1);
  EXPECT_THROW(parse_dashboard_reply("0,{},Other();", "Stop()"),
               std::runtime_error);
  EXPECT_THROW(parse_dashboard_reply("0,{1,Stop();", "Stop()"),
               std::runtime_error);
  EXPECT_THROW(parse_dashboard_reply("x,{},Stop();", "Stop()"),
               std::runtime_error);
}
TEST(Protocol, FragmentedLoopbackAndPartialNotice) {
  {
    Loopback server({"0,{", "5},Robot", "Mode();"});
    Socket socket("127.0.0.1", server.port, 500);
    EXPECT_EQ(
        parse_dashboard_reply(socket.query("RobotMode()", 500), "RobotMode()")
            .payload,
        "5");
  }
  {
    Loopback server({"controller notice without terminator"});
    Socket socket("127.0.0.1", server.port, 500);
    try {
      socket.query("RobotMode()", 500);
      FAIL();
    } catch (const QueryFailure &e) {
      EXPECT_EQ(e.partial_reply, "controller notice without terminator");
    }
  }
}
TEST(Protocol, RotationUnitsAndBranchBoundaries) {
  for (double angle : {-180.0, 180.0, -179.999, 179.999}) {
    auto t = sdkPose({{123, 456, 789, angle, 0, angle}});
    auto round = sdkPose(toSdkPose(t));
    EXPECT_NEAR(t.translation().x(), 0.123, 1e-12);
    EXPECT_LT(rotationDistance(t, round), 1e-10);
    EXPECT_LT((t.translation() - round.translation()).norm(), 1e-12);
  }
}
TEST(Protocol, StopUsesWholeDedicatedAcknowledgementBudget) {
  Loopback server({"0,{},Stop();"}, std::chrono::milliseconds(600));
  auto c = config();
  c.backend = "hardware";
  c.dashboard_port = server.port;
  c.enable = c.tcp_verified = c.limits_verified = c.execution_verified =
      c.servo_certified = true;
  V4Driver driver(c);
  EXPECT_NO_THROW(driver.stop());
}
TEST(Protocol, TypedSdkAndGatedZeroActuation) {
  auto c = config();
  std::vector<std::string> calls;
  TransportFactory factory = [&]() {
    return std::make_unique<ReplyTransport>([&](const std::string &command) {
      calls.push_back(command);
      return "0,{0,0,0,180,0,0}," + command + ";";
    });
  };
  V4Driver driver(c, factory);
  auto fk = driver.fk(c.home);
  EXPECT_NEAR(fk.linear()(2, 2), -1, 1e-12);
  ASSERT_EQ(calls.size(), 1u);
  EXPECT_NE(calls.front().find("user=0,tool=0"), std::string::npos);
  EXPECT_THROW(driver.servo(c.home, 0.03), Failure);
  EXPECT_THROW(driver.stop(), Failure);
  EXPECT_EQ(calls.size(), 1u);
}
TEST(Protocol, ControllerErrorAndEchoRefused) {
  auto c = config();
  V4Driver error(c, [] {
    return std::make_unique<ReplyTransport>(
        [](const std::string &x) { return "-100,{}," + x + ";"; });
  });
  EXPECT_THROW(error.fk(c.home), Failure);
  V4Driver mismatch(c, [] {
    return std::make_unique<ReplyTransport>(
        [](const std::string &) { return "0,{},Other();"; });
  });
  EXPECT_THROW(mismatch.fk(c.home), std::runtime_error);
}
TEST(Protocol, NearZeroAnglesUseDecimalWireFormat) {
  auto c = config();
  std::string command;
  V4Driver driver(c, [&] {
    return std::make_unique<ReplyTransport>([&](const std::string &s) {
      command = s;
      return "0,{0,0,0,0,0,0}," + s + ";";
    });
  });
  auto target = sdkPose({{30, -576, 480, 179.9997, .000034052, 173.14}});
  driver.ik(target, c.home);
  const auto numeric =
      command.substr(command.find('(') + 1,
                     command.find(",useJointNear") - command.find('(') - 1);
  EXPECT_EQ(numeric.find_first_of("eE"), std::string::npos);
  EXPECT_NE(command.find("0.000034052"), std::string::npos);
}
TEST(Protocol, AmbiguousReplyKeepsSessionLockedWithoutReconnect) {
  auto c = config();
  int created = 0, calls = 0;
  V4Driver driver(c, [&] {
    ++created;
    return std::make_unique<ReplyTransport>([&](const std::string &s) {
      ++calls;
      return calls == 3 ? "0,{},WrongEcho();" : "0,{0,0,0,0,0,0}," + s + ";";
    });
  });
  driver.fk(c.home);
  driver.fk(c.home);
  EXPECT_EQ(created, 1);
  EXPECT_THROW(driver.fk(c.home), std::exception);
  EXPECT_THROW(driver.fk(c.home),Failure);
  EXPECT_EQ(created, 1);
  EXPECT_EQ(calls,3);
}
TEST(Protocol, StopWorksWithExclusiveControllerSession) {
  auto c = config();
  c.backend = "hardware";
  c.enable = c.tcp_verified = c.limits_verified = c.execution_verified =
      c.servo_certified = true;
  int created = 0;
  bool stopped = false;
  V4Driver driver(c, [&] {
    if (++created > 1)
      throw std::runtime_error("IP:Port occupied");
    return std::make_unique<ReplyTransport>([&](const std::string &s) {
      if (s == "Stop()") {
        stopped = true;
        return "0,{}," + s + ";";
      }
      return "0,{0,0,0,0,0,0}," + s + ";";
    });
  });
  driver.fk(c.home);
  EXPECT_NO_THROW(driver.stop());
  EXPECT_EQ(created, 1);
  EXPECT_TRUE(stopped);
}
TEST(GeometryGuard, HomePassesAndWristSingularityIsRejected) {
  auto c = config();
  c.safety_model_file = ARM_TEST_GUARD;
  auto m = model(c);
  GeometryGuard guard(c, m);
  EXPECT_NO_THROW(guard.check(c.home));
  auto q = c.home;
  q[4] = 0;
  EXPECT_THROW(guard.check(q), Failure);
}
TEST(GeometryGuard, MissingOrInconsistentGeometryFailsClosed) {
  auto c = config();
  c.safety_model_file = "/definitely-missing-arm-geometry.yaml";
  EXPECT_THROW(GeometryGuard(c, model(c)), std::exception);
  c.safety_model_file = ARM_TEST_GUARD;
  c.flange_tcp.translation().z() = .21;
  EXPECT_THROW(GeometryGuard(c, model(c)), Failure);
}
TEST(Commissioning, NarrowCapsAndGuardAreMandatory) {
  auto c = config();
  c.backend = "commissioning";
  c.enable = true;
  EXPECT_THROW(c.validate(), Failure);
  c.safety_model_file = ARM_TEST_GUARD;
  c.freshness = .1;
  c.max_tracking_error = .005;
  c.velocity.fill(.02);
  c.acceleration.fill(.05);
  c.profiles = {{"conservative", .01, .04}};
  c.twist_limits = {pi / 9, pi / 9};
  EXPECT_NO_THROW(c.validate());
  EXPECT_TRUE(c.hardwareAllowed());
  auto m = model(c);
  OfflineKinematics k(m);
  Planner no_guard(c, m, k, [] {});
  Goal goal;
  goal.operation = Operation::Home;
  EXPECT_THROW(no_guard.plan(goal, trusted(c, m)), Failure);
  GeometryGuard guard(c, m);
  auto q = c.home;
  q[0] += .11;
  EXPECT_THROW(guard.check(q), Failure);
  c.commissioning_tcp_radius = .05;
  EXPECT_THROW(c.validate(), Failure);
}
TEST(Planner, GeometryGuardIsAppliedToActualSamples) {
  auto c = config();
  auto m = model(c);
  OfflineKinematics k(m);
  size_t checked = 0;
  Planner planner(
      c, m, k, [] {},
      [&](const Joints &) {
        ++checked;
        throw Failure(PLANNING_FAILED, "injected geometry reject");
      });
  Goal goal;
  goal.operation = Operation::Home;
  EXPECT_THROW(planner.plan(goal, trusted(c, m)), Failure);
  EXPECT_GT(checked, 0u);
}
TEST(Planner, NeutralityPrecedesTimeForExactEquivalentSolutions) {
  auto c = config();
  c.safety_model_file = ARM_TEST_GUARD;
  auto m = model(c);
  GeometryGuard geometry(c, m);
  auto current = c.home, neutral = c.home;
  current[5] += 330 * pi / 180;
  neutral[5] -= 30 * pi / 180;
  // Two exact FK-equivalent inverse solutions, not an accept-all IK mock.
  class EquivalentSolutions final : public Kinematics {
    std::shared_ptr<const chess_robot_twist::Model> m_;
    Joints home_, wound_, neutral_;

  public:
    EquivalentSolutions(std::shared_ptr<const chess_robot_twist::Model> m,
                        Joints h, Joints a, Joints b)
        : m_(m), home_(h), wound_(a), neutral_(b) {}
    Transform fk(const Joints &q) override { return m_->flange(q); }
    Joints ik(const Transform &target, const Joints &seed) override {
      if ((target.matrix() - m_->flange(wound_).matrix()).norm() > 1e-8)
        throw Failure(PLANNING_FAILED, "fixture target has no known inverse");
      return std::abs(seed[5] - home_[5]) > .1 ? wound_ : neutral_;
    }
  } k(m, c.home, current, neutral);
  auto s = trusted(c, m);
  for (int i = 1; i <= 120; ++i) {
    auto q = c.home;
    q[5] += (current[5] - c.home[5]) * i / 120;
    s.tracker.advance(q);
  }
  s.feedback.q = current;
  Goal goal;
  goal.operation = Operation::Pregrasp;
  goal.target = m->tcp(current);
  goal.target.translation().z() -= c.clearance;
  Planner normal(c, m, k, [] {}, [&](const Joints &q) { geometry.check(q); });
  auto a = normal.plan(goal, s);
  EXPECT_NEAR(a.segments.back().samples.back().q[5], neutral[5], 1e-10);
  EXPECT_EQ(s.tracker.lastJoints(), current);
  c.cost_duration = 1e9;
  c.profiles = {{"conservative", .01, .04}};
  c.max_samples = 100000;
  Planner twist_priority(
      c, m, k, [] {}, [&](const Joints &q) { geometry.check(q); });
  auto b = twist_priority.plan(goal, s);
  EXPECT_NEAR(b.segments.back().samples.back().q[5], neutral[5], 1e-10);
  c.profiles = {{"conservative", .25, .5}};
  Planner blocked_neutral(c, m, k, [] {}, [&](const Joints &q) {
    geometry.check(q);
    if (q[5] < current[5] - .01)
      throw Failure(PLANNING_FAILED, "injected blocked unwind path");
  });
  auto safe = blocked_neutral.plan(goal, s);
  EXPECT_NEAR(safe.segments.back().samples.back().q[5], current[5], 1e-10);
  c.twist_limits = {.001, .001};
  Planner tight(c, m, k, [] {}, [&](const Joints &q) { geometry.check(q); });
  EXPECT_THROW(tight.plan(goal, s), Failure);
  EXPECT_EQ(s.tracker.lastJoints(), current);
}
TEST(Planner, NeutralBandAndIncumbentHysteresisPreventChatter) {
  auto c = config();
  chess_robot_twist::Metrics a, b;
  a.total = a.net = 4 * pi / 180;
  b.total = b.net = 6 * pi / 180;
  // Across the neutral boundary, sub-hysteresis jitter retains the incumbent.
  for (int i = 0; i < 100; ++i) {
    a.total = a.net = (4 + .1 * std::sin(i)) * pi / 180;
    EXPECT_FALSE(preferCandidate(a, 10, b, 10.5, c));
    EXPECT_FALSE(preferCandidate(b, 10, a, 10.5, c));
  }
  a.total = a.net = 1 * pi / 180;
  b.total = b.net = 10 * pi / 180;
  EXPECT_TRUE(preferCandidate(a, 1000, b, 1, c));
  EXPECT_FALSE(preferCandidate(b, 1, a, 1000, c));
  b.total = b.net = 1 * pi / 180;
  EXPECT_TRUE(preferCandidate(a, 1, b, 3, c));
  EXPECT_FALSE(preferCandidate(a, 2.5, b, 3, c));
  // Net cancellation is not neutrality when local absolute twist stays large.
  a.net = 0; a.total = 1;
  b.net = .2; b.total = .2;
  EXPECT_FALSE(preferCandidate(a, 1, b, 100, c));
  c.urdf = ARM_TEST_URDF;
  c.twist_hysteresis = c.neutral_band;
  EXPECT_THROW(c.validate(), Failure);
}
TEST(Feedback, DuplicateAndTimestampFaults) {
  FeedbackSequence seq;
  auto f = frame(100);
  EXPECT_TRUE(seq.accept(f));
  EXPECT_FALSE(seq.accept(f));
  auto changed = f;
  field<double>(changed, 432, 1);
  EXPECT_FALSE(seq.accept(changed));
  EXPECT_THROW(seq.accept(frame(99)), std::runtime_error);
  auto bad = frame(101);
  field<double>(bad, 480, std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(seq.accept(bad), std::runtime_error);
  EXPECT_TRUE(seq.accept(frame(101)));
}
TEST(Protocol, ValidNegativeReplyKeepsExclusiveSession) {
  auto c = config();
  int created = 0, calls = 0;
  V4Driver driver(c, [&] {
    if (++created > 1)
      throw std::runtime_error("exclusive session lost");
    return std::make_unique<ReplyTransport>([&](const std::string &s) {
      if (++calls == 1)
        return "-1,{}," + s + ";";
      return "0,{0,0,0,0,0,0}," + s + ";";
    });
  });
  EXPECT_THROW(driver.fk(c.home), Failure);
  EXPECT_NO_THROW(driver.fk(c.home));
  EXPECT_EQ(created, 1);
}
TEST(Validation, ConfigurationCaptureAndPose) {
  auto c = config();
  EXPECT_NO_THROW(c.validate());
  c.velocity[0] = 0;
  EXPECT_THROW(c.validate(), Failure);
  c = config();
  c.joint_step = 0.2;
  EXPECT_THROW(c.validate(), Failure);
  c = config();
  EXPECT_NO_THROW(validateCapture("base_link", 10, 10.5, c));
  EXPECT_THROW(validateCapture("camera", 10, 10, c), Failure);
  EXPECT_THROW(validateCapture("base_link", 0, 10, c), Failure);
  EXPECT_THROW(validateCapture("base_link", 9, 11, c), Failure);
  EXPECT_THROW(validateCapture("base_link", 11, 10, c), Failure);
  Transform t = Transform::Identity();
  t.translation().z() = 0.1;
  t.linear() =
      Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  EXPECT_NO_THROW(validatePose(t, c));
  t.translation().x() = 3;
  EXPECT_THROW(validatePose(t, c), Failure);
  t.translation().x() = 0;
  t.linear().setIdentity();
  EXPECT_THROW(validatePose(t, c), Failure);
}
TEST(Lifecycle, BusyCancelExactConfirmation) {
  TaskGate gate;
  EXPECT_FALSE(gate.confirm("a", true, "late"));
  EXPECT_TRUE(gate.reserve("a"));
  EXPECT_FALSE(gate.reserve("b"));
  EXPECT_FALSE(gate.confirm("a", true, "wrong phase"));
  gate.beginGripperWait(Clock::now() + std::chrono::seconds(1));
  EXPECT_FALSE(gate.confirm("b", true, "wrong job"));
  EXPECT_TRUE(gate.confirm("a", false, "failed"));
  EXPECT_FALSE(gate.confirm("a", true, "duplicate"));
  ASSERT_TRUE(gate.confirmation().has_value());
  EXPECT_FALSE(*gate.confirmation());
  EXPECT_FALSE(gate.cancel("b"));
  EXPECT_TRUE(gate.cancel("a"));
  EXPECT_TRUE(gate.canceled());
  gate.release();
  EXPECT_FALSE(gate.confirm("a", true, "late"));
  EXPECT_TRUE(gate.reserve("c"));
  EXPECT_FALSE(gate.canceled());
  EXPECT_FALSE(gate.confirmation());
}
TEST(Lifecycle, ExplicitReferenceAndMissingFeedback) {
  auto c = config();
  auto m = model(c);
  State s(c, m);
  EXPECT_FALSE(s.snapshot().reference);
  EXPECT_THROW(s.initialize(), Failure);
  s.update(feedback(c));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  s.update(feedback(c, 2));
  EXPECT_NO_THROW(s.initialize());
  EXPECT_TRUE(s.snapshot().reference);
  auto q = feedback(c, 3);
  q.q[0] += c.tracker_options.max_joint_step_rad * 2;
  EXPECT_THROW(s.update(q), chess_robot_twist::Error);
  EXPECT_FALSE(s.snapshot().reference);
  s.update(feedback(c, 4));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  s.update(feedback(c, 5));
  s.initialize();
  s.invalidate("gap");
  s.update(feedback(c, 6));
  EXPECT_FALSE(s.snapshot().reference);
  TaskGate gate;
  EXPECT_TRUE(gate.reserve("busy"));
  std::string detail;
  EXPECT_FALSE(gate.initialize(s, detail));
}
TEST(Lifecycle, HostFreshnessAndGapInvalidates) {
  auto c = config();
  c.freshness = 0.01;
  auto m = model(c);
  State s(c, m);
  s.update(feedback(c));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  s.update(feedback(c, 2));
  s.initialize();
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  EXPECT_FALSE(s.snapshot().fresh);
  EXPECT_FALSE(s.snapshot().reference);
  s.update(feedback(c, 3));
  EXPECT_FALSE(s.snapshot().reference);
}
TEST(Geometry, RealUrdfIkAndIndependentTracker) {
  auto c = config();
  auto m = model(c);
  OfflineKinematics k(m);
  auto desired = c.home;
  desired[0] += 0.01;
  auto solved = k.ik(k.fk(desired), c.home);
  EXPECT_LT((k.fk(solved).matrix() - k.fk(desired).matrix()).norm(), 1e-5);
  auto live = trusted(c, m).tracker;
  auto candidate = live;
  auto q = c.home;
  q[0] += 0.01;
  candidate.advance(q);
  EXPECT_EQ(live.lastJoints(), c.home);
  EXPECT_TRUE(chess_robot_twist::withinLimits(live.metrics(), c.twist_limits));
  auto out = c.home;
  out[0] = m->upperLimits()[0] + 0.01;
  EXPECT_FALSE(m->withinJointBounds(out));
  auto metrics = chess_robot_twist::metricsFromLocal({pi, -pi}, {1, 1});
  EXPECT_NEAR(metrics.net, 0, 1e-12);
  EXPECT_NEAR(metrics.total, 2 * pi, 1e-12);
  EXPECT_FALSE(chess_robot_twist::withinLimits(metrics, {pi, pi}));
}
TEST(Planner, HomeTimedCapsCancellationAndNoReference) {
  auto c = config();
  auto m = model(c);
  OfflineKinematics k(m);
  auto s = trusted(c, m);
  Goal g;
  g.operation = Operation::Home;
  Planner planner(c, m, k, [] {});
  auto plan = planner.plan(g, s);
  ASSERT_EQ(plan.segments.size(), 1u);
  EXPECT_FALSE(plan.wait_for_gripper);
  auto &samples = plan.segments[0].samples;
  ASSERT_GE(samples.size(), 2u);
  for (size_t i = 1; i < samples.size(); ++i)
    EXPECT_NEAR(samples[i].time - samples[i - 1].time, c.sample_period, 1e-12);
  s.reference = false;
  EXPECT_THROW(planner.plan(g, s), Failure);
  s.reference = true;
  s.fresh = false;
  EXPECT_THROW(planner.plan(g, s), Failure);
  s.fresh = true;
  Planner canceled(c, m, k,
                   [] { throw Failure(CANCELED, "test cancellation"); });
  EXPECT_THROW(canceled.plan(g, s), Failure);
}
TEST(Planner, PregraspAndVerticalRejectSideways) {
  auto c = config();
  auto m = model(c);
  auto home = m->tcp(c.home);
  c.grasp_axis = home.linear().transpose() * (-Eigen::Vector3d::UnitZ());
  OfflineKinematics k(m);
  auto s = trusted(c, m);
  Planner planner(c, m, k, [] {});
  Goal goal;
  goal.operation = Operation::Pregrasp;
  goal.target = home;
  goal.target.translation().z() -= c.clearance;
  auto plan = planner.plan(goal, s);
  ASSERT_EQ(plan.segments.size(), 1u);
  EXPECT_NEAR(jointDistance(plan.segments[0].samples.back().q, c.home), 0,
              1e-9);
  goal.operation = Operation::Descend;
  goal.target.translation().x() += 0.05;
  EXPECT_THROW(planner.plan(goal, s), Failure);
}
TEST(Planner, NonzeroHomePathCapsAndBudget) {
  auto c = config();
  auto m = model(c);
  OfflineKinematics k(m);
  auto s = trusted(c, m);
  s.feedback.q[0] += 0.05;
  s.tracker.advance(s.feedback.q);
  Planner planner(c, m, k, [] {});
  Goal g;
  g.operation = Operation::Home;
  auto plan = planner.plan(g, s);
  auto samples = plan.segments[0].samples;
  Joints previous{};
  for (size_t i = 1; i < samples.size(); ++i) {
    for (size_t j = 0; j < 6; ++j) {
      double v = (samples[i].q[j] - samples[i - 1].q[j]) / c.sample_period;
      EXPECT_LE(std::abs(v), c.velocity[j] + 1e-9);
      EXPECT_LE(std::abs(v - previous[j]) / c.sample_period,
                c.acceleration[j] + 1e-9);
      previous[j] = v;
    }
  }
  for (size_t j = 0; j < 6; ++j)
    EXPECT_LE(std::abs(previous[j]) / c.sample_period,
              c.acceleration[j] + 1e-9);
  c.max_samples = 2;
  Planner bounded(c, m, k, [] {});
  EXPECT_THROW(bounded.plan(g, s), Failure);
}
TEST(Executor, DisabledBackendMakesZeroActuationCalls) {
  auto c = config();
  auto m = model(c);
  State state(c, m);
  TaskGate gate;
  size_t calls = 0;
  V4Driver driver(c, [&]() {
    ++calls;
    return std::make_unique<ReplyTransport>(
        [](const std::string &) { return std::string(); });
  });
  Executor executor(c, driver, state, gate, m);
  EXPECT_THROW(executor.run(Plan{}), Failure);
  EXPECT_NO_THROW(executor.stopOwned());
  EXPECT_EQ(calls, 0u);
}

namespace {
// Explicitly offline execution fixture: updates State directly and publishes no
// ROS data.
class FixtureDriver final : public Driver {
  Config config_;
  std::shared_ptr<const chess_robot_twist::Model> model_;
  State &state_;
  TaskGate &gate_;
  uint64_t stamp_ = 100;

public:
  size_t servo_calls = 0, stop_calls = 0;
  bool cancel_on_servo = false, fail_stop = false;
  int servo_delay_ms = 0;
  FixtureDriver(const Config &c,
                std::shared_ptr<const chess_robot_twist::Model> m, State &s,
                TaskGate &g)
      : config_(c), model_(m), state_(s), gate_(g) {}
  Transform fk(const Joints &q) override { return model_->flange(q); }
  Joints ik(const Transform &, const Joints &) override {
    throw Failure(
        PLANNING_FAILED,
        "fixture IK unsupported; use real offline kinematics fixture");
  }
  Feedback readFeedback() override {
    throw Failure(FEEDBACK_FAULT, "no socket in fixture");
  }
  void servo(const Joints &q, double) override {
    ++servo_calls;
    std::this_thread::sleep_for(std::chrono::milliseconds(servo_delay_ms));
    auto f = feedback(config_, stamp_++);
    f.q = q;
    state_.update(f);
    if (cancel_on_servo)
      gate_.cancel(gate_.view().job);
  }
  void stop() override {
    ++stop_calls;
    if (fail_stop)
      throw Failure(STOP_FAILED, "injected Stop transport fault");
    auto f = state_.snapshot().feedback;
    f.controller_stamp = stamp_++;
    f.received = Clock::now();
    f.speed.fill(0);
    f.mode = 5;
    state_.update(f);
  }
};
Config hardwareFixture() {
  auto c = config();
  c.backend = "hardware";
  c.enable = c.tcp_verified = c.limits_verified = c.execution_verified =
      c.servo_certified = true;
  return c;
}
void initializeFixture(State &state, const Config &c) {
  state.update(feedback(c));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  state.update(feedback(c, 2));
  state.initialize();
}
Plan stationaryPlan(const Config &c) {
  return {{{"HOME", {{c.home, 0}, {c.home, c.sample_period}}}}, false};
}
} // namespace
TEST(Executor, MeasuredDwellCancelStopAndFailure) {
  auto c = hardwareFixture();
  auto m = model(c);
  State state(c, m);
  initializeFixture(state, c);
  TaskGate gate;
  ASSERT_TRUE(gate.reserve("job"));
  FixtureDriver driver(c, m, state, gate);
  Executor executor(c, driver, state, gate, m);
  driver.cancel_on_servo = true;
  try {
    executor.run(stationaryPlan(c));
    FAIL();
  } catch (const Failure &e) {
    EXPECT_EQ(e.code, CANCELED);
  }
  EXPECT_TRUE(executor.attempted());
  EXPECT_TRUE(executor.owned());
  EXPECT_EQ(driver.servo_calls, 1u);
  EXPECT_NO_THROW(executor.stopOwned());
  EXPECT_EQ(driver.stop_calls, 1u);
  EXPECT_FALSE(executor.owned());
  gate.release();
  ASSERT_TRUE(gate.reserve("next"));
  Executor failing(c, driver, state, gate, m);
  driver.fail_stop = true;
  EXPECT_THROW(failing.run(stationaryPlan(c)), Failure);
  EXPECT_THROW(failing.stopOwned(), Failure);
  EXPECT_TRUE(failing.owned());
}
TEST(Executor, SlowServoReplyAbortsWithoutReplayAndVerifiesStop) {
  auto c = hardwareFixture();
  auto m = model(c);
  State state(c, m);
  initializeFixture(state, c);
  TaskGate gate;
  ASSERT_TRUE(gate.reserve("slow-reply"));
  FixtureDriver driver(c, m, state, gate);
  driver.servo_delay_ms = 90;
  Executor executor(c, driver, state, gate, m);
  auto plan = stationaryPlan(c);
  plan.segments.front().samples.push_back({c.home, 2 * c.sample_period});
  try {
    executor.run(plan);
    FAIL();
  } catch (const Failure &e) {
    EXPECT_EQ(e.code, EXECUTION_FAILED);
    EXPECT_NE(std::string(e.what()).find("previous_io_s="), std::string::npos);
  }
  EXPECT_EQ(driver.servo_calls, 1u);
  EXPECT_TRUE(executor.owned());
  EXPECT_NO_THROW(executor.stopOwned());
  EXPECT_EQ(driver.stop_calls, 1u);
  EXPECT_FALSE(executor.owned());
}
TEST(Executor, GripperFailureCannotRunRetreat) {
  auto c = hardwareFixture();
  auto m = model(c);
  State state(c, m);
  initializeFixture(state, c);
  TaskGate gate;
  ASSERT_TRUE(gate.reserve("sequence"));
  FixtureDriver driver(c, m, state, gate);
  Executor executor(c, driver, state, gate, m);
  auto plan = stationaryPlan(c);
  plan.segments.front().phase = "DESCEND";
  plan.segments.push_back(
      {"RETREAT", {{c.home, 0}, {c.home, c.sample_period}}});
  plan.wait_for_gripper = true;
  std::thread confirmer([&] {
    auto deadline = Clock::now() + std::chrono::seconds(1);
    while (Clock::now() < deadline) {
      if (gate.view().phase == "WAIT_GRIPPER") {
        gate.confirm("sequence", false, "injected failure");
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  try {
    executor.run(plan);
    FAIL();
  } catch (const Failure &e) {
    EXPECT_EQ(e.code, GRIPPER_FAILED);
  }
  confirmer.join();
  EXPECT_EQ(driver.servo_calls, 1u);
  EXPECT_TRUE(executor.attempted());
  EXPECT_FALSE(executor.owned());
}
TEST(Executor, MissingFeedbackNeverMoves) {
  auto c = hardwareFixture();
  auto m = model(c);
  State state(c, m);
  TaskGate gate;
  ASSERT_TRUE(gate.reserve("job"));
  FixtureDriver driver(c, m, state, gate);
  Executor executor(c, driver, state, gate, m);
  EXPECT_THROW(executor.run(stationaryPlan(c)), Failure);
  EXPECT_EQ(driver.servo_calls, 0u);
  EXPECT_EQ(driver.stop_calls, 0u);
}
TEST(Planner, WholePathTwistRejectsAndLiveReferenceUnchanged) {
  auto c = config();
  c.twist_limits = {0.001, 0.001};
  auto m = model(c);
  OfflineKinematics k(m);
  auto s = trusted(c, m);
  s.feedback.q[0] += 0.05;
  s.tracker.advance(s.feedback.q);
  Planner planner(c, m, k, [] {});
  Goal home;
  home.operation = Operation::Home;
  EXPECT_THROW(planner.plan(home, s), Failure);
  EXPECT_EQ(s.tracker.lastJoints(), s.feedback.q);
}
TEST(Geometry, ContinuousBaseSpinRetainsFullTurn) {
  auto c = config();
  auto m = model(c);
  auto tracker = trusted(c, m).tracker;
  // Model deliberately does not impose joint bounds: this is a rod/turn-history
  // counterexample, not an executable motion candidate.
  for (size_t i = 1; i <= 256; ++i) {
    auto q = c.home;
    q[0] += 2 * pi * double(i) / 256;
    tracker.advance(q);
  }
  auto metrics = tracker.metrics();
  EXPECT_NEAR(std::abs(metrics.net), 2 * pi, 1e-5);
  EXPECT_NEAR(metrics.total, 2 * pi, 1e-5);
}
TEST(Executor, MatchingConfirmationPermitsRetreatAndTimeoutDoesNot) {
  auto c = hardwareFixture();
  c.gripper_timeout = 0.03;
  auto m = model(c);
  State state(c, m);
  initializeFixture(state, c);
  TaskGate gate;
  ASSERT_TRUE(gate.reserve("sequence"));
  FixtureDriver driver(c, m, state, gate);
  Executor executor(c, driver, state, gate, m);
  auto plan = stationaryPlan(c);
  plan.segments.front().phase = "DESCEND";
  plan.segments.push_back(
      {"RETREAT", {{c.home, 0}, {c.home, c.sample_period}}});
  plan.wait_for_gripper = true;
  std::thread confirmer([&] {
    auto deadline = Clock::now() + std::chrono::seconds(1);
    while (Clock::now() < deadline) {
      if (gate.view().phase == "WAIT_GRIPPER") {
        gate.confirm("sequence", true, "closed");
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  EXPECT_NO_THROW(executor.run(plan));
  confirmer.join();
  EXPECT_EQ(driver.servo_calls, 2u);
  gate.release();
  ASSERT_TRUE(gate.reserve("timeout"));
  Executor timeout(c, driver, state, gate, m);
  try {
    timeout.run(plan);
    FAIL();
  } catch (const Failure &e) {
    EXPECT_EQ(e.code, GRIPPER_FAILED);
  }
  EXPECT_EQ(driver.servo_calls, 3u);
}

TEST(Lifecycle, StopFaultLatchesAdmissionAndCannotResetReference) {
  auto c = config();
  auto m = model(c);
  State state(c, m);
  TaskGate gate;
  ASSERT_TRUE(gate.reserve("faulted"));
  gate.latchFault("Stop not verified");
  gate.release();
  EXPECT_EQ(gate.view().phase, "FAULT");
  EXPECT_FALSE(gate.reserve("next"));
  std::string detail;
  EXPECT_FALSE(gate.initialize(state, detail));
}

TEST(Validation, FixtureCannotUsePhysicalController) {
  auto c = config();
  c.test_fixture = true;
  c.ip = "192.168.30.234";
  EXPECT_THROW(c.validate(), Failure);
  c.ip = "127.0.0.1";
  EXPECT_NO_THROW(c.validate());
}

TEST(Lifecycle, GripperDeadlineIsCheckedInsideAdmissionMutex) {
  TaskGate gate;
  ASSERT_TRUE(gate.reserve("job"));
  gate.beginGripperWait(Clock::now() - std::chrono::milliseconds(1));
  EXPECT_FALSE(gate.confirm("job", true, "late"));
  EXPECT_FALSE(gate.finishGripperWait().has_value());
  EXPECT_FALSE(gate.confirm("job", true, "closed phase"));
  gate.beginGripperWait(Clock::now() + std::chrono::seconds(1));
  EXPECT_TRUE(gate.confirm("job", true, "on time"));
  auto confirmation = gate.finishGripperWait();
  ASSERT_TRUE(confirmation.has_value());
  EXPECT_TRUE(*confirmation);
  EXPECT_FALSE(gate.confirm("job", true, "already closed"));
}

TEST(Lifecycle, TerminalCommitOrdersCancellation) {
  TaskGate gate;
  ASSERT_TRUE(gate.reserve("complete"));
  EXPECT_FALSE(gate.commitTerminal());
  EXPECT_FALSE(gate.cancel("complete"));
  gate.release();
  ASSERT_TRUE(gate.reserve("cancel"));
  EXPECT_TRUE(gate.cancel("cancel"));
  EXPECT_TRUE(gate.commitTerminal());
  EXPECT_FALSE(gate.cancel("cancel"));
}

TEST(Validation, ImpossibleReferenceToleranceRejectedAtStartup) {
  auto c = config();
  c.home_tolerance = .2;
  c.tracker_options.max_joint_step_rad = .1;
  EXPECT_THROW(c.validate(), Failure);
  c = config();
  c.tracker_options.max_phase_step_rad = pi;
  EXPECT_THROW(c.validate(), Failure);
}
TEST(ReferenceRestore, ReplaysWindingAndRejectsUnattestedOrMovedHandoff) {
  auto c=config(); auto m=model(c);auto expected=trusted(c,m).tracker;
  const auto path="/tmp/chess_arm_history_"+std::to_string(getpid())+".bin";
  Joints last=c.home;
  {
    std::ofstream out(path,std::ios::binary);
    for(int i=0;i<=50;++i) {
      auto bytes=frame(100+i);last=c.home;last[0]+=.001*i;
      field<uint64_t>(bytes,24,(i==0 || i==50) ? 5 : 7);
      for(size_t j=0;j<6;++j) field<double>(bytes,432+8*j,last[j]*180/pi);
      int64_t host=1000000000LL+i*1000000LL;
      out.write(reinterpret_cast<char *>(&host),sizeof(host));
      out.write(reinterpret_cast<char *>(bytes.data()),bytes.size());
      expected.advance(last);
    }
  }
  State state(c,m);
  auto f=feedback(c,1000);f.q=last;state.update(f);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  f.controller_stamp++;f.received=Clock::now();state.update(f);
  EXPECT_THROW(state.restore({path},false),Failure);
  EXPECT_NO_THROW(state.restore({path},true));
  auto restored=state.snapshot();EXPECT_TRUE(restored.reference);
  EXPECT_NEAR(restored.tracker.metrics().net,expected.metrics().net,1e-10);
  EXPECT_GT(restored.tracker.metrics().total,.01);
  EXPECT_THROW(state.restore({path},true),Failure);
  State moved(c,m);f.q[0]+=.01;f.controller_stamp++;f.received=Clock::now();moved.update(f);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));f.controller_stamp++;f.received=Clock::now();moved.update(f);
  EXPECT_THROW(moved.restore({path},true),Failure);EXPECT_FALSE(moved.snapshot().reference);
  std::remove(path.c_str());
}
TEST(Exploration, ConfiguredStageBoundsPreserveGlobalGuards) {
  auto c=config();c.backend="exploration";c.enable=true;c.safety_model_file=ARM_TEST_GUARD;
  c.freshness=.1;c.max_tracking_error=.005;c.velocity.fill(.02);c.acceleration.fill(.05);
  c.profiles={{"near",.01,.04},{"wide",.02,.04}};c.default_profile="near";
  c.exploration_boundaries={{"near",.08,.3,pi/6},{"wide",.7,3,pi}};
  EXPECT_NO_THROW(c.validate());EXPECT_TRUE(c.hardwareAllowed());
  c.freshness=.2;EXPECT_NO_THROW(c.validate());
  c.freshness=.200001;EXPECT_THROW(c.validate(),Failure);
  c.freshness=.1;
  auto effective=c.forProfile("wide");EXPECT_EQ(effective.commissioning_tcp_radius,.7);
  EXPECT_EQ(effective.twist_limits.total_limit_rad,pi);
  GeometryGuard guard(effective,model(c));EXPECT_NO_THROW(guard.check(c.home));
  auto singular=c.home;singular[2]=.035;EXPECT_THROW(guard.check(singular),Failure);
  EXPECT_THROW(c.forProfile("unconfigured"),Failure);
  c.exploration_boundaries.back().twist_limit=3*pi;EXPECT_THROW(c.validate(),Failure);
}
TEST(Planner, SingleQuinticUsesProfileCapsWithoutDuplicateBlending) {
  auto c=config();c.profiles={{"conservative",.01,.04}};
  auto m=model(c);OfflineKinematics k(m);auto s=trusted(c,m);
  s.feedback.q[0]+=.05;s.tracker.advance(s.feedback.q);
  Planner planner(c,m,k,[]{});Goal g;g.operation=Operation::Home;
  auto plan=planner.plan(g,s);const auto &samples=plan.segments.front().samples;
  EXPECT_LT(samples.back().time,9.6);
  Joints previous{};
  for(size_t i=1;i<samples.size();++i)
    for(size_t j=0;j<6;++j) {
      double v=(samples[i].q[j]-samples[i-1].q[j])/c.sample_period;
      EXPECT_LE(std::abs(v),.01+1e-10);
      EXPECT_LE(std::abs(v-previous[j])/c.sample_period,.04+1e-10);
      previous[j]=v;
    }
  for(double v:previous) EXPECT_LE(std::abs(v)/c.sample_period,.04+1e-10);
}
TEST(ReferenceRestore, PreservesRecorded330DegreesRatherThanPrincipalMinus30) {
  auto c=config();auto m=model(c);auto expected=trusted(c,m).tracker;
  const auto path="/tmp/chess_arm_full_turn_"+std::to_string(getpid())+".bin";
  Joints last=c.home;
  {
    std::ofstream out(path,std::ios::binary);
    for(int i=0;i<=400;++i) {
      auto bytes=frame(100+i);last=c.home;last[5]+=330*pi/180*i/400;
      field<uint64_t>(bytes,24,(i==0 || i==400) ? 5 : 7);
      for(size_t j=0;j<6;++j) field<double>(bytes,432+8*j,last[j]*180/pi);
      int64_t host=1000000000LL+i*1000000LL;
      out.write(reinterpret_cast<char *>(&host),sizeof(host));out.write(reinterpret_cast<char *>(bytes.data()),bytes.size());
      expected.advance(last);
    }
  }
  State state(c,m);auto f=feedback(c,1000);f.q=last;state.update(f);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));f.controller_stamp++;f.received=Clock::now();state.update(f);
  ASSERT_NO_THROW(state.restore({path},true));auto restored=state.snapshot();
  EXPECT_NEAR(restored.tracker.metrics().net,expected.metrics().net,1e-10);
  EXPECT_GT(std::abs(restored.tracker.metrics().net),5.0);
  EXPECT_GT(restored.tracker.metrics().total,5.0);std::remove(path.c_str());
}
TEST(ReferenceRestore, StationaryServoHoldRequiresSeparateStoppedAuditAndDwell) {
  for(int variant=0;variant<5;++variant) {
    SCOPED_TRACE(variant);
    auto c=config();c.stationary_speed=.002;auto m=model(c);auto expected=trusted(c,m).tracker;
    auto base="/tmp/chess_arm_hold_"+std::to_string(getpid())+"_"+std::to_string(variant);
    auto path=base+".bin",stopped=base+"_stopped.bin";Joints last=c.home;
    {
      std::ofstream out(path,std::ios::binary);
      for(int i=0;i<=12;++i) {
        auto bytes=frame(100+i);last=c.home;last[5]+=.2*std::min(i,10)/10;
        field<uint64_t>(bytes,24,i==0 ? 5 : 7);
        for(size_t j=0;j<6;++j) field<double>(bytes,432+8*j,last[j]*180/pi);
        if(i>0 && (i<=10 || variant==1)) field<double>(bytes,480,.01*180/pi);
        if(i==11 && variant==4) field<double>(bytes,480,std::numeric_limits<double>::quiet_NaN());
        int64_t host=1000000000LL+i*10000000LL;
        out.write(reinterpret_cast<char *>(&host),sizeof(host));out.write(reinterpret_cast<char *>(bytes.data()),bytes.size());
        expected.advance(last);
      }
    }
    {
      auto bytes=frame(200);field<uint64_t>(bytes,24,5);
      auto joined=last;if(variant==2) joined[5]+=.01;
      for(size_t j=0;j<6;++j) field<double>(bytes,432+8*j,joined[j]*180/pi);
      int64_t host=2000000000LL;std::ofstream out(stopped,std::ios::binary);
      out.write(reinterpret_cast<char *>(&host),sizeof(host));out.write(reinterpret_cast<char *>(bytes.data()),bytes.size());
    }
    State state(c,m);auto f=feedback(c,1000);f.q=last;state.update(f);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));f.controller_stamp++;f.received=Clock::now();state.update(f);
    std::vector<std::string> files{path};if(variant!=3) files.push_back(stopped);
    if(variant==0) {
      ASSERT_NO_THROW(state.restore(files,true));
      EXPECT_NEAR(state.snapshot().tracker.metrics().net,expected.metrics().net,1e-10);
      EXPECT_GT(state.snapshot().tracker.metrics().total,.1);
    } else EXPECT_THROW(state.restore(files,true),std::exception);
    std::remove(path.c_str());std::remove(stopped.c_str());
  }
}
TEST(Planner, SmoothVerticalRetainsLineAndProfileDerivativeBounds) {
  auto c=config();c.profiles={{"conservative",.02,.04}};
  auto m=model(c);OfflineKinematics k(m);auto s=trusted(c,m);
  Goal g;g.operation=Operation::Descend;g.target=m->tcp(c.home);g.target.translation().z()-=.05;
  Planner planner(c,m,k,[]{});auto plan=planner.plan(g,s);const auto &samples=plan.segments.front().samples;
  Joints previous{};auto origin=m->tcp(c.home);
  for(size_t i=0;i<samples.size();++i) {
    auto actual=m->tcp(samples[i].q);
    EXPECT_LE((actual.translation().head<2>()-origin.translation().head<2>()).norm(),c.position_tolerance);
    EXPECT_LE(rotationDistance(actual,origin),c.rotation_tolerance);
    if(!i)continue;
    for(size_t j=0;j<6;++j) {
      double v=(samples[i].q[j]-samples[i-1].q[j])/c.sample_period;
      EXPECT_LE(std::abs(v),.02+1e-10);EXPECT_LE(std::abs(v-previous[j])/c.sample_period,.04+1e-10);previous[j]=v;
    }
  }
  for(double v:previous) EXPECT_LE(std::abs(v)/c.sample_period,.04+1e-10);
}
TEST(Streaming, ControllerFollowingHorizonIsIndependentOfSendPeriod) {
  auto c=streamConfig();c.servo_command_time=.1;auto shared=std::make_shared<StreamFixtureState>();
  V4Driver driver(c,[shared]{return std::make_unique<StreamFixture>(shared);});
  driver.beginStream();driver.servo(c.home,.03);driver.endStream();
  ASSERT_EQ(shared->sent_commands.size(),1u);
  EXPECT_NE(shared->sent_commands.front().find("t=0.100000000"),std::string::npos);
  auto invalid=config();invalid.servo_command_time=.01;EXPECT_THROW(invalid.validate(),Failure);
}
TEST(Protocol, CoalescedRepliesAreRetainedAcrossReceives) {
  Loopback server({"0,{},One();0,{},Two();"});
  Socket socket("127.0.0.1",server.port,500);socket.send_command("One()",500);
  EXPECT_EQ(socket.receive_reply(500),"0,{},One();");
  EXPECT_EQ(socket.receive_reply(500),"0,{},Two();");
}

TEST(Protocol, LongFragmentedReplyAndOversizeBound) {
  std::string payload(12000,'1');
  Loopback server({"0,{"+payload.substr(0,3000),payload.substr(3000)+"},Large();"});
  Socket socket("127.0.0.1",server.port,500);
  EXPECT_EQ(parse_dashboard_reply(socket.query("Large()",500),"Large()").payload,payload);
  Loopback oversized({"0,{"+std::string(17000,'1')+"},Large();"});
  Socket capped("127.0.0.1",oversized.port,500);
  EXPECT_THROW(capped.query("Large()",500),QueryFailure);
}

TEST(FeedbackTimestamp, ReceiptClockIsIndependentOfControllerAndPublication) {
  auto c=config();auto b=frame(123456789);field<uint64_t>(b,24,5);
  const auto received=Clock::now();
  const int64_t ros=42000000017LL;
  auto f=decodeFeedbackFrame(b,received,ros);
  EXPECT_EQ(f.ros_received_ns,ros);EXPECT_EQ(f.controller_stamp,123456789u);
  EXPECT_EQ(f.received,received);
  State state(c,model(c));state.update(f);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  EXPECT_EQ(state.snapshot().feedback.ros_received_ns,ros);
  EXPECT_THROW(decodeFeedbackFrame(FeedbackFrame{},received,ros),std::runtime_error);
}
TEST(SpeedModes, DefaultAliasOfficialCapsAndPauseGate) {
  auto c=config();c.profiles={{"low",.708,1.416},{"high",2.36,4.72},{"conservative",.708,1.416}};
  c.default_profile="high";
  EXPECT_EQ(c.profile("").name,"high");
  EXPECT_DOUBLE_EQ(c.profile("low").speed,.708);
  EXPECT_DOUBLE_EQ(c.profile("conservative").speed,.708);
  c.backend="hardware";c.enable=c.tcp_verified=c.limits_verified=c.execution_verified=c.servo_certified=true;
  EXPECT_TRUE(c.hardwareAllowed());c.motion_paused=true;EXPECT_FALSE(c.hardwareAllowed());
  c.motion_paused=false;c.controller_family="v3";EXPECT_FALSE(c.hardwareAllowed());
  c.controller_family="v4";c.controller_firmware="3.5.5";EXPECT_FALSE(c.hardwareAllowed());
}
TEST(SpeedModes, BothProfilesKeepDerivativeAndSampleGuards) {
  for (const auto &name : {"low","high"}) {
    auto c=config();c.profiles={{"low",.708,1.416},{"high",2.36,4.72}};c.default_profile="low";
    auto m=model(c);OfflineKinematics k(m);auto s=trusted(c,m);
    for(int i=0;i<10;++i){s.feedback.q[0]+=.05;s.tracker.advance(s.feedback.q);}
    Planner p(c,m,k,[]{});Goal g;g.operation=Operation::Home;g.profile=name;
    auto plan=p.plan(g,s);const auto &samples=plan.segments.front().samples;
    Joints previous_velocity{};
    for(size_t i=1;i<samples.size();++i) {
      double dt=samples[i].time-samples[i-1].time;EXPECT_GT(dt,0);
      for(size_t j=0;j<6;++j) {
        double v=(samples[i].q[j]-samples[i-1].q[j])/dt;
        EXPECT_LE(std::abs(v),std::min(c.profile(name).speed,c.velocity[j])+1e-8);
        EXPECT_LE(std::abs(v-previous_velocity[j])/dt,std::min(c.profile(name).acceleration,c.acceleration[j])+1e-7);
        previous_velocity[j]=v;
      }
    }
  }
}

TEST(Roundtrip, ZeroClearanceIsDirectTcpAndReturnUsesPredictedEndpoint) {
  auto c=config();c.clearance=0;c.home_roundtrip_precheck=true;
  EXPECT_NO_THROW(c.validate());
  auto m=model(c);OfflineKinematics k(m);auto s=trusted(c,m);
  auto target_q=c.home;target_q[0]+=.04;
  // Use a real reachable TCP transform with fixed outward orientation for this
  // node semantic test; the runner's manifest retains the formal Home orientation.
  Goal g;g.operation=Operation::Pregrasp;g.target=m->tcp(target_q);
  auto untouched=s.tracker.lastJoints();
  Planner p(c,m,k,[]{});auto plan=p.plan(g,s);
  ASSERT_EQ(plan.segments.size(),1u);
  auto end=plan.segments.front().samples.back().q;
  EXPECT_LT((m->tcp(end).translation()-g.target.translation()).norm(),1e-5);
  EXPECT_NE(plan.selection.find("home_return_checked=true"),std::string::npos);
  EXPECT_EQ(s.tracker.lastJoints(),untouched);
  // Only the predicted return reverses q0; a Home->Home fake would not. Reject
  // that reverse path specifically and ensure no outward plan is admitted.
  double last=c.home[0];bool reversed=false;
  Planner blocked(c,m,k,[]{},[&](const Joints &q){
    const double displacement=q[0]-c.home[0];
    if(q[0]<last-1e-7 && displacement>.005 && displacement<.035) {
      reversed=true;throw Failure(PLANNING_FAILED,"test return-only obstacle");
    }
    last=q[0];
  });
  EXPECT_THROW(blocked.plan(g,s),Failure);EXPECT_TRUE(reversed);
  EXPECT_EQ(s.tracker.lastJoints(),untouched);
}
TEST(Roundtrip, RejectsOffsetAndWrongStartButKeepsFormalHome) {
  auto c=config();c.home_roundtrip_precheck=true;
  EXPECT_THROW(c.validate(),Failure);c.clearance=0;EXPECT_NO_THROW(c.validate());
  auto m=model(c);OfflineKinematics k(m);auto s=trusted(c,m);
  Planner p(c,m,k,[]{});Goal g;g.operation=Operation::Pregrasp;g.target=m->tcp(c.home);
  auto off=s;off.feedback.q[0]+=.03;
  EXPECT_THROW(p.plan(g,off),Failure);
  g.operation=Operation::Home;EXPECT_NO_THROW(p.plan(g,off));
  g.operation=Operation::Sequence;EXPECT_THROW(p.plan(g,s),Failure);
}

TEST(PowerCycle, RecordedFaultTailPreservesWindingAcrossDisabledEpoch) {
  auto c=config();auto m=model(c);auto expected=trusted(c,m).tracker;
  const auto path="/tmp/chess_power_history_"+std::to_string(getpid())+".bin";
  Joints last=c.home;
  {
    std::ofstream out(path,std::ios::binary);
    for(int i=0;i<5;++i) {
      auto bytes=frame(100+i);last=c.home;
      if(i)last[5]+=.01;
      if(i>=3)last[1]+=.003;
      field<uint64_t>(bytes,24,i>=3 ? 9 : i==1 ? 7 : 5);
      if(i>=3)field<double>(bytes,480+8,.02*180/pi);
      for(size_t j=0;j<6;++j)field<double>(bytes,432+8*j,last[j]*180/pi);
      int64_t host=1000000000LL+i*8000000LL;
      out.write(reinterpret_cast<char*>(&host),sizeof(host));
      out.write(reinterpret_cast<char*>(bytes.data()),bytes.size());
      expected.advance(last);
    }
  }
  auto populate=[&](State &state,Joints q) {
    auto f=feedback(c,1);f.q=q;f.mode=4;state.update(f);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    f.controller_stamp=2;f.received=Clock::now();state.update(f);
  };
  State normal(c,m);populate(normal,last);EXPECT_THROW(normal.restore({path},true),Failure);
  c.power_cycle_handoff=true;
  State recovered(c,m);populate(recovered,last);
  EXPECT_THROW(recovered.restore({path},false),Failure);
  EXPECT_NO_THROW(recovered.restore({path},true));
  auto snapshot=recovered.snapshot();EXPECT_TRUE(snapshot.reference);EXPECT_EQ(snapshot.feedback.mode,4u);
  EXPECT_NEAR(snapshot.tracker.metrics().net,expected.metrics().net,1e-10);
  EXPECT_GT(snapshot.tracker.metrics().total,.005);
  State moved(c,m);auto wrong=last;wrong[0]+=.002;populate(moved,wrong);
  EXPECT_THROW(moved.restore({path},true),Failure);
  State not_terminal(c,m);populate(not_terminal,last);
  EXPECT_THROW(not_terminal.restore({path,path},true),Failure);
  State far(c,m);wrong=last;wrong[5]+=.04;populate(far,wrong);
  EXPECT_THROW(far.restore({path},true),Failure);
  std::remove(path.c_str());
}
TEST(PowerCycle, OfficialEnableIsExplicitAndNeverClearsAlarms) {
  for(bool alarm : {false,true}) {
    auto c=hardwareFixture();c.power_cycle_handoff=c.enable_after_history_restore=true;
    std::vector<std::string> calls;
    V4Driver d(c,[&]{return std::make_unique<ReplyTransport>([&](const std::string &cmd){
      calls.push_back(cmd);
      if(cmd=="GetErrorID()")return std::string("0,{")+(alarm ? "[123]" : "[]")+"},GetErrorID();";
      if(cmd=="EnableRobot()")return std::string("0,{},EnableRobot();");
      throw std::runtime_error("unexpected command including ClearError");
    });});
    if(alarm)EXPECT_THROW(d.enableAfterHistoryRestore(),Failure);
    else EXPECT_NO_THROW(d.enableAfterHistoryRestore());
    EXPECT_EQ(calls.size(),alarm ? 1u : 2u);
    EXPECT_EQ(calls.front(),"GetErrorID()");
  }
  auto c=config();int connections=0;
  V4Driver passive(c,[&]()->std::unique_ptr<CommandTransport>{++connections;throw std::runtime_error("must not connect");});
  EXPECT_THROW(passive.enableAfterHistoryRestore(),Failure);EXPECT_EQ(connections,0);
}

TEST(PowerCycle, EnableUsesDedicatedOfficialAcknowledgementBudget) {
  int listener=::socket(AF_INET,SOCK_STREAM,0);ASSERT_GE(listener,0);
  sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::bind(listener,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)),0);
  ASSERT_EQ(::listen(listener,1),0);socklen_t length=sizeof(addr);
  ASSERT_EQ(::getsockname(listener,reinterpret_cast<sockaddr*>(&addr),&length),0);
  std::string server_error;
  std::thread server([&]{
    pollfd p{listener,POLLIN,0};
    if(::poll(&p,1,2000)<=0){server_error="no connection";return;}
    int peer=::accept(listener,nullptr,nullptr);
    timeval timeout{2,0};::setsockopt(peer,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    for(int i=0;i<2;++i) {
      std::string cmd;char c;
      while(cmd.empty() || cmd.back()!=')') {
        if(::recv(peer,&c,1,0)!=1){server_error="incomplete command";::close(peer);return;}
        cmd+=c;
      }
      const auto expected=i ? "EnableRobot()" : "GetErrorID()";
      if(cmd!=expected){server_error="wrong command";break;}
      if(i)std::this_thread::sleep_for(std::chrono::milliseconds(600));
      std::string reply=i ? "0,{},EnableRobot();" : "0,{[]},GetErrorID();";
      (void)::send(peer,reply.data(),reply.size(),MSG_NOSIGNAL);
    }
    ::close(peer);
  });
  auto c=hardwareFixture();c.power_cycle_handoff=c.enable_after_history_restore=true;
  c.dashboard_port=ntohs(addr.sin_port);c.timeout_ms=100;c.enable_ack_timeout_ms=1200;
  {V4Driver driver(c);EXPECT_NO_THROW(driver.enableAfterHistoryRestore());}
  server.join();::close(listener);EXPECT_TRUE(server_error.empty())<<server_error;
}

TEST(PowerCycle, FullOriginalAuditsJoinOneExplicitRecordedEpochAndKeepGap) {
  auto c=config();c.freshness=.2;c.power_cycle_handoff=true;
  auto m=model(c);auto expected=trusted(c,m).tracker;
  const auto prefix="/tmp/chess_recorded_epoch_"+std::to_string(getpid());
  const std::string oldfile=prefix+"_old.bin",newfile=prefix+"_new.bin";
  Joints last=c.home;
  auto write=[&](const std::string &path,bool old,bool moved=false) {
    std::ofstream out(path,std::ios::binary);
    for(int i=0;i<5;++i) {
      auto bytes=frame(old ? 100+i : 1+i);
      auto q=c.home;
      if(old) {if(i)q[5]+=.01;if(i>=3)q[1]+=.003;}
      else {q[5]+=.01;q[1]+=.003;if(i>=2)q[0]+=.05;if(moved)q[0]+=.002;}
      const uint64_t mode=old ? (i>=3 ? 9 : i==1 ? 7 : 5) : (i==0 ? 4 : i==2 ? 7 : 5);
      field<uint64_t>(bytes,24,mode);
      for(size_t j=0;j<6;++j)field<double>(bytes,432+8*j,q[j]*180/pi);
      const int64_t host=(old ? 1000000000LL : 10000000000LL)+i*8000000LL+
          (!old && i>=2 ? 138000000LL : 0); // Actual 146 ms gap is retained.
      out.write(reinterpret_cast<const char*>(&host),sizeof(host));
      out.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
      if(!moved)expected.advance(q);
      last=q;
    }
  };
  write(oldfile,true);write(newfile,false);
  auto populate=[&](State &state) {
    auto f=feedback(c,1000);f.q=last;f.mode=5;state.update(f);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    f.controller_stamp++;f.received=Clock::now();state.update(f);
  };
  State unmarked(c,m);populate(unmarked);EXPECT_THROW(unmarked.restore({oldfile,newfile},true),Failure);
  c.power_cycle_join_file=newfile;
  State recovered(c,m);populate(recovered);
  EXPECT_NO_THROW(recovered.restore({oldfile,newfile},true));
  auto snapshot=recovered.snapshot();EXPECT_TRUE(snapshot.reference);
  EXPECT_GT(jointDistance(snapshot.feedback.q,c.home),c.home_tolerance);
  EXPECT_NEAR(snapshot.tracker.metrics().total,expected.metrics().total,1e-9);
  EXPECT_GT(snapshot.tracker.metrics().total,.005);
  c.freshness=.1;State narrow(c,m);populate(narrow);
  EXPECT_THROW(narrow.restore({oldfile,newfile},true),Failure);
  c.freshness=.2;write(newfile,false,true);
  State moved(c,m);populate(moved);EXPECT_THROW(moved.restore({oldfile,newfile},true),Failure);
  c.power_cycle_join_file=oldfile;State initial(c,m);populate(initial);
  EXPECT_THROW(initial.restore({oldfile,newfile},true),Failure);
  std::remove(oldfile.c_str());std::remove(newfile.c_str());
}
