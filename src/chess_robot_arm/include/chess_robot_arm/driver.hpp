#pragma once
#include "chess_robot_arm/core.hpp"
#include "chess_robot_arm/feedback_frame.hpp"
#include <fstream>
#include <memory>
#include <condition_variable>
#include <deque>
#include <thread>

class Socket;
namespace chess_robot_arm {
// A query has a whole-operation deadline and must retain unterminated notice
// bytes.
class CommandTransport {
public:
  virtual ~CommandTransport() = default;
  virtual std::string query(const std::string &) = 0;
  virtual bool streaming() const { return false; }
  virtual void send(const std::string &) { throw std::runtime_error("stream unsupported"); }
  virtual std::string receive(int) { throw std::runtime_error("stream unsupported"); }
};
using TransportFactory = std::function<std::unique_ptr<CommandTransport>()>;
Feedback decodeFeedbackFrame(const FeedbackFrame &, Clock::time_point, int64_t ros_ns);
Transform sdkPose(const std::array<double, 6> &mm_deg);
std::array<double, 6> toSdkPose(const Transform &);
class V4Driver final : public Driver {
  Config config_;
  TransportFactory transport_;
  std::unique_ptr<CommandTransport> command_transport_;
  std::mutex command_mutex_;
  struct Pending { std::string command; Clock::time_point sent; };
  std::mutex stream_mutex_;
  std::condition_variable stream_cv_;
  std::deque<Pending> pending_;
  std::thread reply_thread_;
  bool streaming_ = false, stream_exit_ = false, stream_broken_ = false;
  std::string stream_error_;
  std::optional<uint64_t> last_queue_id_;
  std::ofstream send_audit_;
  void replyLoop();
  void closeStream(bool stopping);
  void audit(const std::string &, const std::string &, Clock::time_point);
  void validateServoReply(const std::string &, bool ordered);
  std::ofstream command_audit_, feedback_audit_;
  std::unique_ptr<Socket> feedback_socket_;
  FeedbackSequence sequence_;
  std::function<int64_t()> ros_clock_;
  bool session_fault_ = false;
  std::string call(const std::string &);

public:
  explicit V4Driver(const Config &, TransportFactory factory = {},
                    std::function<int64_t()> ros_clock = {});
  ~V4Driver() override;
  Transform fk(const Joints &) override;
  Joints ik(const Transform &, const Joints &) override;
  Feedback readFeedback() override;
  void servo(const Joints &, double) override;
  void stop() override;
  void beginStream() override;
  void checkStream() override;
  void endStream() override;
};
} // namespace chess_robot_arm
