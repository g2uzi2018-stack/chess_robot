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

using namespace chess_robot_arm;
namespace {
Config config() {
  Config c;
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
TEST(Protocol, OrderedConnectionReusedAndDiscardedOnBadReply) {
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
  driver.fk(c.home);
  EXPECT_EQ(created, 2);
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
TEST(Planner, TwistPreferenceIsWeightedAndCanBeOverruledByTime) {
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
  EXPECT_NEAR(a.segments.back().samples.back().q[5], current[5], 1e-10);
  c.cost_duration = .001;
  Planner twist_priority(
      c, m, k, [] {}, [&](const Joints &q) { geometry.check(q); });
  auto b = twist_priority.plan(goal, s);
  EXPECT_NEAR(b.segments.back().samples.back().q[5], neutral[5], 1e-10);
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
