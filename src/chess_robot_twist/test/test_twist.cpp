#include "chess_robot_twist/model.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace chess_robot_twist;
namespace {
constexpr double pi = 3.14159265358979323846;
const Joints home{1.3873624514421463,  0.29600795831732624,
                  1.4554299194273725,  -0.18064158423931584,
                  -1.5707963267948966, -0.06370430298043014};
Transform tcpTransform() {
  Transform value = Transform::Identity();
  value.translation() = Eigen::Vector3d(0, 0, .2);
  value.linear() = (Eigen::AngleAxisd(-pi / 2, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitX()))
                       .toRotationMatrix();
  return value;
}
std::shared_ptr<const Model> model() {
  return std::make_shared<Model>(TEST_URDF_PATH, "base_link", "Link6",
                                 tcpTransform(), home);
}
void follow(Tracker &tracker, const Joints &start, const Joints &delta,
            int count = 720) {
  for (int step = 1; step <= count; ++step) {
    Joints q{};
    for (size_t i = 0; i < q.size(); ++i)
      q[i] = start[i] + delta[i] * step / count;
    tracker.advance(q);
  }
}
std::vector<MaterialSpan> bentSpans() {
  std::vector<MaterialSpan> spans;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; ++i) {
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(i * .35, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d end = point + rotation * Eigen::Vector3d::UnitZ();
    spans.push_back({point, end, rotation * Eigen::Vector3d::UnitX()});
    point = end;
  }
  return spans;
}
} // namespace

TEST(FramedRod, PureBendingHasNoTwist) {
  auto sample = measure(bentSpans());
  for (const double phase : sample.phase)
    EXPECT_NEAR(phase, 0, 1e-12);
}
TEST(FramedRod, CommonRigidTransformDoesNotChangeTwist) {
  auto spans = bentSpans();
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(.8, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  const Eigen::Vector3d translation(1, 2, 3);
  for (auto &span : spans) {
    span.start = rotation * span.start + translation;
    span.end = rotation * span.end + translation;
    span.director = rotation * span.director;
  }
  for (const double phase : measure(spans).phase)
    EXPECT_NEAR(phase, 0, 1e-12);
}
TEST(FramedRod, SignedLocalTwistAndEnergy) {
  std::vector<MaterialSpan> spans;
  for (const double angle : {0., 1.2, -.4}) {
    const double z = spans.size();
    spans.push_back({Eigen::Vector3d(0, 0, z), Eigen::Vector3d(0, 0, z + 1),
                     Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
                         Eigen::Vector3d::UnitX()});
  }
  const auto sample = measure(spans);
  const auto result = metricsFromLocal(sample.phase, sample.dual_length);
  EXPECT_NEAR(result.net, -.4, 1e-12);
  EXPECT_NEAR(result.total, 2.8, 1e-12);
  EXPECT_NEAR(result.energy, 4.0, 1e-12);
}
TEST(FramedRod, OppositeTwistCannotCancelTheTotal) {
  const auto result = metricsFromLocal({2 * pi, -2 * pi}, {1, 1});
  EXPECT_NEAR(result.net, 0, 1e-12);
  EXPECT_NEAR(result.total, 4 * pi, 1e-12);
  EXPECT_FALSE(withinLimits(result, Limits{}));
}
TEST(FramedRod, RejectsInvalidGeometry) {
  auto spans = bentSpans();
  spans[1].end = spans[1].start;
  EXPECT_THROW(measure(spans), Error);
  spans = bentSpans();
  spans[1].start.x() += .01;
  EXPECT_THROW(measure(spans), Error);
  spans = bentSpans();
  spans[0].director = Eigen::Vector3d::UnitZ();
  EXPECT_THROW(measure(spans), Error);
  spans = {{Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
            Eigen::Vector3d::UnitX()},
           {Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::UnitX()}};
  EXPECT_THROW(measure(spans), Error);
  EXPECT_THROW(measure({}), Error);
}
TEST(FramedRod, RejectsInvalidNumbersAndLimits) {
  EXPECT_THROW(metricsFromLocal({1}, {0}), Error);
  EXPECT_THROW(metricsFromLocal({1}, {1, 2}), Error);
  EXPECT_THROW(metricsFromLocal({std::numeric_limits<double>::infinity()}, {1}),
               Error);
  EXPECT_THROW(withinLimits(Metrics{}, Limits{0, 1}), Error);
  Metrics bad;
  bad.net = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(withinLimits(bad, Limits{}));
}
TEST(NovaModel, CollapseCoincidentOriginsAndPreserveTcpFk) {
  const auto robot = model();
  EXPECT_EQ(robot->spans(home).size(), 6u);
  Joints zero{};
  const auto flange = robot->flange(zero);
  EXPECT_NEAR(flange.translation().x(), .000001818, 1e-6);
  EXPECT_NEAR(flange.translation().y(), -.223327727, 1e-6);
  EXPECT_NEAR(flange.translation().z(), 1.090000273, 1e-6);
  EXPECT_TRUE(robot->tcp(home).matrix().isApprox(
      (robot->flange(home) * tcpTransform()).matrix(), 1e-12));
  EXPECT_EQ(robot->jointNames()[0], "joint1");
  EXPECT_EQ(robot->jointNames()[5], "joint6");
}
TEST(NovaModel, PositionBoundsAreSeparateFromGeometry) {
  const auto robot = model();
  EXPECT_TRUE(robot->withinJointBounds(home));
  auto q = home;
  q[0] += 2 * pi;
  EXPECT_FALSE(robot->withinJointBounds(q));
  EXPECT_NO_THROW(
      robot->phases(q)); // Mathematical counterexample, NOT a motion goal.
}
TEST(NovaModel, RejectsBadConfiguration) {
  EXPECT_THROW(
      Model(TEST_URDF_PATH, "base_link", "absent", tcpTransform(), home),
      Error);
  EXPECT_THROW(
      Model(TEST_URDF_PATH, "base_link", "Link6", Transform::Identity(), home),
      Error);
  auto wrong = tcpTransform();
  wrong.linear() *= 2;
  EXPECT_THROW(Model(TEST_URDF_PATH, "base_link", "Link6", wrong, home), Error);
  auto q = home;
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Model(TEST_URDF_PATH, "base_link", "Link6", tcpTransform(), q),
               Error);
}
TEST(ContinuousTracking, ExplicitHomeReferenceRequired) {
  Tracker tracker(model());
  EXPECT_FALSE(tracker.valid());
  EXPECT_THROW(tracker.metrics(), Error);
  EXPECT_THROW(tracker.advance(home), Error);
  auto away = home;
  away[0] += .2;
  EXPECT_THROW(tracker.initializeAtHome(away, .01), Error);
  EXPECT_FALSE(tracker.valid());
  tracker.initializeAtHome(home, .01);
  EXPECT_TRUE(tracker.valid());
  EXPECT_NEAR(tracker.metrics().total, 0, 1e-12);
}
TEST(ContinuousTracking, BaseFullTurnRetained) {
  Tracker tracker(model());
  tracker.initializeAtHome(home, .01);
  Joints delta{};
  delta[0] = 2 * pi;
  follow(tracker, home, delta);
  EXPECT_NEAR(tracker.metrics().net, 2 * pi, 1e-9);
  EXPECT_NEAR(tracker.metrics().total, 2 * pi, 1e-9);
  EXPECT_TRUE(withinLimits(tracker.metrics(), Limits{}));
}
TEST(ContinuousTracking, DistalFullTurnRetained) {
  Tracker tracker(model());
  tracker.initializeAtHome(home, .01);
  Joints delta{};
  delta[5] = 2 * pi;
  follow(tracker, home, delta);
  EXPECT_NEAR(tracker.metrics().net, 2 * pi, 1e-9);
  EXPECT_NEAR(tracker.metrics().total, 2 * pi, 1e-9);
}
TEST(ContinuousTracking, OppositeFullTurnsHaveNonzeroTotal) {
  Tracker tracker(model());
  tracker.initializeAtHome(home, .01);
  Joints delta{};
  delta[0] = 2 * pi;
  delta[5] = -2 * pi;
  follow(tracker, home, delta);
  EXPECT_NEAR(tracker.metrics().net, 0, 1e-9);
  EXPECT_NEAR(tracker.metrics().total, 4 * pi, 1e-9);
  EXPECT_FALSE(withinLimits(tracker.metrics(), Limits{}));
}
TEST(ContinuousTracking, ReverseMotionUnwindsInsteadOfAddingTravel) {
  Tracker tracker(model());
  tracker.initializeAtHome(home, .01);
  Joints delta{};
  delta[0] = 3;
  follow(tracker, home, delta, 360);
  auto start = home;
  start[0] += 3;
  delta[0] = -3;
  follow(tracker, start, delta, 360);
  EXPECT_NEAR(tracker.metrics().total, 0, 1e-9);
}
TEST(ContinuousTracking, ValidEndsCanHideInvalidIntermediateStates) {
  Tracker tracker(model());
  tracker.initializeAtHome(home, .01);
  Limits limits{1, 1};
  bool violated = false;
  for (int i = 1; i <= 360; ++i) {
    auto q = home;
    q[0] += 2 * pi * i / 360;
    tracker.advance(q);
    violated = violated || !withinLimits(tracker.metrics(), limits);
  }
  for (int i = 359; i >= 0; --i) {
    auto q = home;
    q[0] += 2 * pi * i / 360;
    tracker.advance(q);
  }
  EXPECT_TRUE(violated);
  EXPECT_TRUE(withinLimits(tracker.metrics(), limits));
}
TEST(ContinuousTracking, CandidateCopyDoesNotMutateLiveHistory) {
  Tracker live(model());
  live.initializeAtHome(home, .01);
  auto candidate = live;
  Joints delta{};
  delta[0] = .2;
  follow(candidate, home, delta, 20);
  EXPECT_NEAR(live.metrics().total, 0, 1e-12);
  EXPECT_NEAR(candidate.metrics().total, .2, 1e-9);
}
TEST(ContinuousTracking, DiscontinuityInvalidatesRatherThanGuessingTurns) {
  Tracker tracker(model());
  tracker.initializeAtHome(home, .01);
  auto q = home;
  q[0] += .2;
  EXPECT_THROW(tracker.advance(q), Error);
  EXPECT_FALSE(tracker.valid());
  EXPECT_THROW(tracker.advance(home), Error);
  tracker.initializeAtHome(home, .01);
  EXPECT_TRUE(tracker.valid());
}
TEST(ContinuousTracking, AmbiguousPhaseStepInvalidates) {
  Tracker tracker(model(), TrackerOptions{.1, .001});
  tracker.initializeAtHome(home, .01);
  auto q = home;
  q[0] += .01;
  EXPECT_THROW(tracker.advance(q), Error);
  EXPECT_FALSE(tracker.valid());
}
TEST(ContinuousTracking, HomePhaseBoundaryNoiseDoesNotCreateFullTurns) {
  Tracker tracker(model());
  tracker.initializeAtHome(home, .01);
  for (int i = 0; i < 100; ++i) {
    auto q = home;
    q[4] += (i % 2 ? 1 : -1) * .00001;
    tracker.advance(q);
  }
  EXPECT_LT(tracker.metrics().total, .0001);
}
TEST(ContinuousTracking, IndependentActualModelBendingOracle) {
  Tracker tracker(model());
  tracker.initializeAtHome(home, .01);
  Joints delta{};
  delta[1] = .2;
  follow(tracker, home, delta, 100);
  EXPECT_LT(tracker.metrics().total, 1e-6);
  tracker.initializeAtHome(home, .01);
  delta = {};
  delta[2] = .2;
  follow(tracker, home, delta, 100);
  EXPECT_NEAR(tracker.metrics().total * 180 / pi, 4.2899694087601725, 1e-6);
}
