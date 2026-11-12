#include "chess_robot_gripper/core.hpp"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <stdexcept>

namespace chess_robot_gripper {
namespace {
class Tcp final : public Transport {
public:
  Tcp(std::string ip, int port, int timeout) : ip_(std::move(ip)), port_(port), timeout_(timeout) {
    in_addr addr{};
    if (inet_pton(AF_INET, ip_.c_str(), &addr) != 1 || port < 1 || port > 65535 || timeout < 10 || timeout > 1000)
      throw std::invalid_argument("TCP requires numeric IPv4, port 1..65535, timeout_ms 10..1000");
  }
  ~Tcp() override { close(); }
  void close() override { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
  Bytes exchange(const Bytes &sent) override {
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_);
    try {
      connect(deadline);
      // There must be exactly one reply per transaction. Reject unsolicited bytes.
      uint8_t extra;
      if (::recv(fd_, &extra, 1, MSG_PEEK | MSG_DONTWAIT) > 0)
        throw std::runtime_error("unsolicited or late RTU data");
      size_t offset = 0;
      while (offset < sent.size()) {
        ready(POLLOUT, deadline);
        auto n = ::send(fd_, sent.data() + offset, sent.size() - offset, MSG_NOSIGNAL);
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) continue;
        if (n <= 0) throw std::runtime_error("TCP write failed");
        offset += size_t(n);
      }
      Bytes reply;
      size_t length = 3;
      while (reply.size() < length) {
        ready(POLLIN, deadline);
        uint8_t data[256];
        auto n = ::recv(fd_, data, std::min(sizeof(data), length - reply.size()), 0);
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) continue;
        if (n <= 0) throw std::runtime_error("TCP disconnected");
        reply.insert(reply.end(), data, data+n);
        if (reply.size() == 3) {
          if (reply[1] & 0x80) length = 5;
          else if (reply[1] == 6) length = 8;
          else if (reply[1] == 3 && reply[2] == 2) length = 7;
          else throw std::runtime_error("invalid RTU reply header");
        }
      }
      return reply;
    } catch (...) { close(); throw; }
  }
private:
  void ready(short events, Clock::time_point deadline) {
    while (true) {
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
      if (ms <= 0) throw std::runtime_error("TCP transaction timeout");
      pollfd p{fd_, events, 0};
      int result = ::poll(&p, 1, int(ms));
      if (result < 0 && errno == EINTR) continue;
      if (result <= 0) throw std::runtime_error("TCP transaction timeout/poll failure");
      // Buffered bytes can still be read together with HUP.
      if (p.revents & events) return;
      throw std::runtime_error("TCP socket error");
    }
  }
  void connect(Clock::time_point deadline) {
    if (fd_ >= 0) return;
    fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0) throw std::runtime_error("socket failed");
    int one = 1;
    setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(port_);
    inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr);
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      if (errno != EINPROGRESS) throw std::runtime_error("TCP connect failed");
      ready(POLLOUT, deadline);
      int error = 0; socklen_t size = sizeof(error);
      if (getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &size) != 0 || error)
        throw std::runtime_error("TCP connect refused/failed");
    }
  }
  std::string ip_;
  int port_, timeout_, fd_ = -1;
};

class Mock final : public Transport {
public:
  explicit Mock(int contact) : contact_(contact), updated_(Clock::now()) {
    if (contact < -1 || contact > 1000) throw std::invalid_argument("mock_contact_raw must be -1..1000");
  }
  void close() override {} // Device state persists across a simulated reconnect.
  Bytes exchange(const Bytes &sent) override {
    if (sent.size() != 8) throw std::runtime_error("mock request length");
    advance();
    auto address = uint16_t(sent[2] << 8 | sent[3]);
    auto value = uint16_t(sent[4] << 8 | sent[5]);
    if (sent[1] == 6) {
      if (address == 0x0100 && value == 1) { init_ = 2; until_ = Clock::now() + std::chrono::milliseconds(200); }
      else if (address == 0x0101) force_ = value;
      else if (address == 0x0103) { target_ = value; status_ = 0; }
      else throw std::runtime_error("mock unsupported write");
      return sent;
    }
    if (sent[1] != 3 || value != 1) throw std::runtime_error("mock unsupported read");
    uint16_t answer;
    if (address == 0x0200) answer = init_;
    else if (address == 0x0201) answer = status_;
    else if (address == 0x0202) answer = uint16_t(std::lround(position_));
    else throw std::runtime_error("mock unsupported register");
    Bytes reply{sent[0], 3, 2, uint8_t(answer >> 8), uint8_t(answer)};
    auto crc = crc16(reply); reply.push_back(uint8_t(crc)); reply.push_back(uint8_t(crc >> 8));
    return reply;
  }
private:
  void advance() {
    auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - updated_).count(); updated_ = now;
    if (init_ == 2 && now >= until_) { init_ = 1; position_ = target_ = 1000; status_ = 1; }
    if (init_ != 1 || status_ != 0) return;
    double destination = target_;
    bool contact = contact_ >= 0 && target_ < contact_ && position_ >= contact_;
    if (contact) destination = contact_;
    double step = 1000. * dt;
    position_ += std::clamp(destination - position_, -step, step);
    if (std::abs(position_ - destination) < .01) status_ = contact ? 2 : 1;
  }
  int contact_, init_ = 0, status_ = 1, target_ = 1000, force_ = 20;
  double position_ = 1000;
  Clock::time_point updated_, until_{};
};
}  // namespace
std::unique_ptr<Transport> tcp_transport(const std::string &ip, int port, int timeout_ms) {
  return std::make_unique<Tcp>(ip, port, timeout_ms);
}
std::unique_ptr<Transport> mock_transport(int contact_raw) { return std::make_unique<Mock>(contact_raw); }
}  // namespace chess_robot_gripper
