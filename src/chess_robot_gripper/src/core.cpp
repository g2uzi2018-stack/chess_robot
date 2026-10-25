#include "chess_robot_gripper/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace chess_robot_gripper {
uint16_t crc16(const Bytes &data) {
  uint16_t crc = 0xffff;
  for (auto byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 1) ? (crc >> 1) ^ 0xa001 : crc >> 1;
  }
  return crc;
}
Bytes request(uint8_t slave, uint8_t function, uint16_t address, uint16_t value) {
  Bytes bytes{slave, function, uint8_t(address >> 8), uint8_t(address),
    uint8_t(value >> 8), uint8_t(value)};
  auto crc = crc16(bytes);
  bytes.push_back(uint8_t(crc)); bytes.push_back(uint8_t(crc >> 8));
  return bytes;
}
uint16_t parse_reply(const Bytes &sent, const Bytes &reply) {
  if (sent.size() != 8 || reply.size() < 5) throw std::runtime_error("short RTU frame");
  Bytes body(reply.begin(), reply.end() - 2);
  if (crc16(body) != uint16_t(reply[reply.size()-2] | (reply.back() << 8)))
    throw std::runtime_error("RTU CRC mismatch");
  if (reply[0] != sent[0]) throw std::runtime_error("wrong RTU slave");
  if (reply[1] == (sent[1] | 0x80))
    throw std::runtime_error("Modbus exception " + std::to_string(reply[2]));
  if (reply[1] != sent[1]) throw std::runtime_error("wrong RTU function");
  if (sent[1] == 6) {
    if (reply != sent) throw std::runtime_error("write echo mismatch");
    return uint16_t(sent[4] << 8 | sent[5]);
  }
  if (sent[1] != 3 || reply.size() != 7 || reply[2] != 2)
    throw std::runtime_error("invalid read frame length");
  return uint16_t(reply[3] << 8 | reply[4]);
}
Driver::Driver(std::unique_ptr<Transport> transport, uint8_t slave)
: transport_(std::move(transport)), slave_(slave) {
  if (!transport_ || slave == 0 || slave > 247) throw std::invalid_argument("slave must be 1..247");
}
uint16_t Driver::transact(uint8_t function, uint16_t address, uint16_t value) {
  try {
    auto sent = request(slave_, function, address, value);
    return parse_reply(sent, transport_->exchange(sent));
  } catch (...) { transport_->close(); throw; } // Never replay uncertain writes.
}
uint16_t Driver::read(uint16_t address) { return transact(3, address, 1); }
void Driver::write(uint16_t address, uint16_t value) { transact(6, address, value); }
Sample Driver::poll() {
  Sample sample;
  sample.sampled = Clock::now(); // Age includes all three sequential reads.
  sample.initialization = read(0x0200);
  sample.status = read(0x0201);
  sample.position = read(0x0202);
  if (sample.initialization > 2 || sample.status > 3 || sample.position > 1000)
    throw std::runtime_error("out-of-range AG95 feedback");
  return sample;
}
void Driver::initialize() { write(0x0100, 1); }
void Driver::move(uint16_t position, uint8_t force) {
  if (position > 1000 || force < 20 || force > 100) throw std::invalid_argument("invalid raw position/force");
  write(0x0101, force);
  write(0x0103, position);
}
std::string validate(const Command &c, int max_force) {
  if (c.operation > 1) return "operation must be INITIALIZE(0) or MOVE(1)";
  if (!std::isfinite(c.timeout) || c.timeout < .5 || c.timeout > 60.) return "timeout_sec must be finite, 0.5..60";
  if (c.operation == 0) {
    if (c.require_contact) return "initialization cannot require contact";
    return {};
  }
  if (c.position > 1000) return "position_raw must be 0..1000";
  if (c.force < 20 || c.force > max_force) return "force_percent outside configured 20..max_force_percent";
  return {};
}
Completion::Completion(Command command, Sample before, int tolerance)
: command_(command), before_(before), tolerance_(tolerance), last_observed_(before.sampled) {}
Outcome Completion::observe(const Sample &s) {
  if (s.sampled <= last_observed_) return Outcome::Pending;
  last_observed_ = s.sampled;
  if (s.initialization != before_.initialization || s.status != before_.status ||
      s.position != before_.position) transition_ = true;
  Outcome result = Outcome::Pending;
  if (command_.operation == 0) {
    // INITIALIZE on an already initialized device is a read-confirmed no-op in the node.
    if (s.initialization == 1) result = Outcome::Initialized;
  } else if (s.initialization == 0) result = Outcome::NotInitialized;
  else if (s.initialization == 1) {
    if (s.status == 3) result = Outcome::Dropped;
    else if (s.status == 2 && transition_) {
      result = command_.require_contact ? Outcome::Grasped : Outcome::UnexpectedContact;
    } else if (s.status == 1 && std::abs(int(s.position) - int(command_.position)) <= tolerance_) {
      result = command_.require_contact ? Outcome::EmptyGrasp : Outcome::Reached;
    }
  }
  stable_ = result == previous_ ? stable_ + 1 : 1;
  previous_ = result;
  return stable_ >= 2 ? result : Outcome::Pending;
}
}  // namespace chess_robot_gripper
