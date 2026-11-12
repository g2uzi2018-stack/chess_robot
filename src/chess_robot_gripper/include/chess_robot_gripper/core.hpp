#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace chess_robot_gripper {
using Clock = std::chrono::steady_clock;
using Bytes = std::vector<uint8_t>;
uint16_t crc16(const Bytes &data);
Bytes request(uint8_t slave, uint8_t function, uint16_t address, uint16_t value);
uint16_t parse_reply(const Bytes &sent, const Bytes &reply);

class Transport {
public:
  virtual ~Transport() = default;
  virtual Bytes exchange(const Bytes &request) = 0;
  virtual void close() = 0;
};
std::unique_ptr<Transport> tcp_transport(const std::string &ip, int port, int timeout_ms);
std::unique_ptr<Transport> mock_transport(int contact_raw = -1);

struct Sample {
  uint16_t initialization = 255, status = 255, position = 65535;
  Clock::time_point sampled{};
};
class Driver {
public:
  Driver(std::unique_ptr<Transport> transport, uint8_t slave);
  Sample poll();
  void initialize();
  void move(uint16_t position, uint8_t force);
private:
  uint16_t read(uint16_t address);
  void write(uint16_t address, uint16_t value);
  uint16_t transact(uint8_t function, uint16_t address, uint16_t value);
  std::unique_ptr<Transport> transport_;
  uint8_t slave_;
};
struct Command {
  uint8_t operation = 1;
  uint16_t position = 1000;
  uint8_t force = 20;
  bool require_contact = false;
  double timeout = 5.;
};
std::string validate(const Command &command, int max_force);
enum class Outcome : uint16_t {
  Pending = 65535, Initialized = 0, Reached = 1, Grasped = 2,
  NotInitialized = 10, EmptyGrasp = 11, UnexpectedContact = 12,
  Dropped = 13, Timeout = 14, CommunicationError = 15, Shutdown = 16
};
// Evaluate only complete, fresh, post-command polls. Contact needs evidence of a
// transition after the write, preventing an old 'held' flag satisfying a new grasp.
class Completion {
public:
  Completion(Command command, Sample before, int tolerance);
  Outcome observe(const Sample &sample);
private:
  Command command_;
  Sample before_;
  int tolerance_, stable_ = 0;
  bool transition_ = false;
  Outcome previous_ = Outcome::Pending;
  Clock::time_point last_observed_;
};
}  // namespace chess_robot_gripper
