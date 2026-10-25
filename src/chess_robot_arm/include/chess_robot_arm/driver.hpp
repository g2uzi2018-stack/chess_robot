#pragma once
#include "chess_robot_arm/core.hpp"
#include "chess_robot_arm/feedback_frame.hpp"
#include <fstream>
#include <memory>

class Socket;
namespace chess_robot_arm {
// A query has a whole-operation deadline and must retain unterminated notice
// bytes.
class CommandTransport {
public:
  virtual ~CommandTransport() = default;
  virtual std::string query(const std::string &) = 0;
};
using TransportFactory = std::function<std::unique_ptr<CommandTransport>()>;
Transform sdkPose(const std::array<double, 6> &mm_deg);
std::array<double, 6> toSdkPose(const Transform &);
class V4Driver final : public Driver {
  Config config_;
  TransportFactory transport_;
  std::unique_ptr<CommandTransport> command_transport_;
  std::mutex command_mutex_;
  std::ofstream command_audit_, feedback_audit_;
  std::unique_ptr<Socket> feedback_socket_;
  FeedbackSequence sequence_;
  std::string call(const std::string &);

public:
  explicit V4Driver(const Config &, TransportFactory factory = {});
  ~V4Driver() override;
  Transform fk(const Joints &) override;
  Joints ik(const Transform &, const Joints &) override;
  Feedback readFeedback() override;
  void servo(const Joints &, double) override;
  void stop() override;
};
} // namespace chess_robot_arm
