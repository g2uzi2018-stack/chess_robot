// Offline audit of the current production Planner. No SDK or motion commands.
#include <Eigen/Dense>
#include <chess_robot_arm/core.hpp>
#include <fstream>
#include <iostream>
using namespace chess_robot_arm;
class KnownPoseSolver : public Kinematics {
  std::shared_ptr<const chess_robot_twist::Model> model_;
  Joints q_;

public:
  KnownPoseSolver(std::shared_ptr<const chess_robot_twist::Model> m, Joints q)
      : model_(m), q_(q) {}
  Transform fk(const Joints &q) override { return model_->flange(q); }
  Joints ik(const Transform &target, const Joints &) override {
    // One independently specified, exactly FK-verified inverse solution. This
    // fixture refuses all other targets; it is not an accept-all mock IK.
    if ((target.matrix() - model_->flange(q_).matrix()).norm() > 1e-8)
      throw Failure(PLANNING_FAILED, "fixture does not solve this target");
    return q_;
  }
};
int main(int argc, char **argv) {
  if (argc != 3)
    return 2;
  Config config;
  config.urdf = argv[1];
  config.flange_tcp.translation().z() = .2;
  config.flange_tcp.linear() =
      (Eigen::AngleAxisd(-pi / 2, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  config.validate();
  auto model = std::make_shared<chess_robot_twist::Model>(
      config.urdf, "base_link", "Link6", config.flange_tcp, config.home);
  Snapshot start{chess_robot_twist::Tracker(model, config.tracker_options)};
  start.tracker.initializeAtHome(config.home, config.home_tolerance);
  start.feedback.q = config.home;
  start.feedback.mode = 5;
  start.fresh = start.reference = true;
  std::ifstream input(argv[2], std::ios::binary);
  Joints q;
  size_t row = 0;
  while (input.read(reinterpret_cast<char *>(q.data()), sizeof(q))) {
    bool pose_ok = false;
    try {
      validatePose(model->tcp(q), config);
      pose_ok = model->withinJointBounds(q);
    } catch (...) {
    }
    KnownPoseSolver solver(model, q);
    Planner planner(config, model, solver, [] {});
    Goal goal;
    goal.operation = Operation::Pregrasp;
    goal.target = model->tcp(q);
    goal.target.translation().z() -= config.clearance;
    try {
      auto plan = planner.plan(goal, start);
      std::cout << "{\"row\":" << row << ",\"pose_validation_accepted\":"
                << (pose_ok ? "true" : "false")
                << ",\"accepted\":true,\"samples\":"
                << plan.segments[0].samples.size() << "}\n";
    } catch (const std::exception &e) {
      std::cout << "{\"row\":" << row << ",\"pose_validation_accepted\":"
                << (pose_ok ? "true" : "false") << ",\"accepted\":false}\n";
    }
    ++row;
  }
}
