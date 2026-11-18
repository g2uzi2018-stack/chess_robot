// 通信实现：Tcp 经网络透传原始 RTU 帧；Mock 在内存中模拟寄存器和开合过程。

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
  // 保存目标地址与单次通信超时；构造时不连接设备。
  Tcp(std::string ip, int port, int timeout) : ip_(std::move(ip)), port_(port), timeout_(timeout) {
    in_addr addr{};
    // 仅接受数字 IPv4、1～65535 端口和 10～1000 ms 超时。
    if (inet_pton(AF_INET, ip_.c_str(), &addr) != 1 || port < 1 || port > 65535 || timeout < 10 || timeout > 1000)
      throw std::invalid_argument("TCP requires numeric IPv4, port 1..65535, timeout_ms 10..1000");
  }

  // 释放连接；关闭 TCP 不代表夹爪停止运动。
  ~Tcp() override {
    close();
  }

  // 关闭当前套接字，下次读写时重新连接。
  void close() override {
    // 套接字已创建时才关闭，并恢复为未连接标记。
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // 发送一帧并收齐一帧回包；连接、发送和接收共用本次截止时间。
  Bytes exchange(const Bytes &sent) override {
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_);
    try {
      connect(deadline);
      uint8_t extra;
      // 新请求前已有未读取字节时，视为迟到或额外回包，避免认成本次应答。
      if (::recv(fd_, &extra, 1, MSG_PEEK | MSG_DONTWAIT) > 0)
        throw std::runtime_error("unsolicited or late RTU data");

      size_t offset = 0;
      // TCP 可能只发出部分字节；继续发送本帧剩余部分，不重发整条命令。
      while (offset < sent.size()) {
        ready(POLLOUT, deadline);  // 等套接字可写，最多等到本次截止时间。
        auto n = ::send(fd_, sent.data() + offset, sent.size() - offset, MSG_NOSIGNAL);
        // 暂时不可写或被系统信号打断时，继续等候。
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
          continue;
        // 其他写错误或没有写出数据时，本次通信失败。
        if (n <= 0)
          throw std::runtime_error("TCP write failed");
        offset += size_t(n);
      }

      Bytes reply;
      size_t length = 3;  // 先读从站、功能码和第三字节，再确定完整长度。
      // TCP 可能分段返回；收齐本帧需要的字节后才交给协议层校验。
      while (reply.size() < length) {
        ready(POLLIN, deadline);  // 等套接字可读。
        uint8_t data[256];
        auto n = ::recv(fd_, data, std::min(sizeof(data), length - reply.size()), 0);
        // 暂时无数据或被系统信号打断时，继续等候。
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
          continue;
        // 连接关闭或读取失败时，结束本次通信。
        if (n <= 0)
          throw std::runtime_error("TCP disconnected");
        reply.insert(reply.end(), data, data+n);
        // 收齐前 3 字节后，按功能码确定剩余回包长度。
        if (reply.size() == 3) {
          // 最高位置 1 是异常应答，总长 5 字节。
          if (reply[1] & 0x80)
            length = 5;
          // 功能码 6 是写回显，总长 8 字节。
          else if (reply[1] == 6)
            length = 8;
          // 功能码 3 且数据长 2 字节，是单寄存器读应答，总长 7 字节。
          else if (reply[1] == 3 && reply[2] == 2)
            length = 7;
          else
            throw std::runtime_error("invalid RTU reply header");
        }
      }
      return reply;
    } catch (...) {
      close();
      throw;
    }
  }

private:
  // 等待指定的可读/可写事件，始终使用调用方传入的同一截止时间。
  void ready(short events, Clock::time_point deadline) {
    while (true) {
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
      // 剩余等待时间用完时，报告超时。
      if (ms <= 0)
        throw std::runtime_error("TCP transaction timeout");
      pollfd p{fd_, events, 0};
      int result = ::poll(&p, 1, int(ms));
      // 等待被系统信号打断时，重新计算剩余时间并继续。
      if (result < 0 && errno == EINTR)
        continue;
      // 超时或其他 poll 错误时，本次通信失败。
      if (result <= 0)
        throw std::runtime_error("TCP transaction timeout/poll failure");
      // 已出现所需事件就返回；即使同时挂断，也先允许读取缓冲区剩余数据。
      if (p.revents & events)
        return;
      throw std::runtime_error("TCP socket error");
    }
  }

  // 按需建立非阻塞 TCP 连接；已有套接字时交给后续读写确认是否仍可用。
  void connect(Clock::time_point deadline) {
    // 已有套接字时直接复用。
    if (fd_ >= 0)
      return;
    fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    // 系统未能创建套接字时，终止本次通信。
    if (fd_ < 0)
      throw std::runtime_error("socket failed");
    int one = 1;
    setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr);
    // 非阻塞连接未立即完成时，检查它是仍在连接还是已经失败。
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      // EINPROGRESS 表示仍在建立连接，其他错误直接上报。
      if (errno != EINPROGRESS)
        throw std::runtime_error("TCP connect failed");
      ready(POLLOUT, deadline);
      int error = 0;
      socklen_t size = sizeof(error);
      // 无法查询连接结果，或套接字报告连接错误时，视为连接失败。
      if (getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &size) != 0 || error)
        throw std::runtime_error("TCP connect refused/failed");
    }
  }

  std::string ip_;
  int port_, timeout_, fd_ = -1;
};

class Mock final : public Transport {
public:
  // 设置模拟接触开度；-1 表示无物体，其他值使用 0～1000 原始单位。
  explicit Mock(int contact) : contact_(contact), updated_(Clock::now()) {
    // 不接受 -1～1000 以外的模拟接触位置。
    if (contact < -1 || contact > 1000)
      throw std::invalid_argument("mock_contact_raw must be -1..1000");
  }

  // 模拟重连只关闭通信语义，不清空设备状态。
  void close() override {}

  // 模拟寄存器收发，返回与真实驱动相同格式的 RTU 应答。
  Bytes exchange(const Bytes &sent) override {
    // 当前模拟协议只接受固定 8 字节请求。
    if (sent.size() != 8)
      throw std::runtime_error("mock request length");
    advance();  // 根据距上次收发的时间，推进模拟初始化和开合。
    auto address = uint16_t(sent[2] << 8 | sent[3]);
    auto value = uint16_t(sent[4] << 8 | sent[5]);
    // 功能码 6：修改模拟寄存器，并回显请求。
    if (sent[1] == 6) {
      // 初始化寄存器写 1 后，进入持续 200 ms 的模拟初始化。
      if (address == 0x0100 && value == 1) {
        init_ = 2;
        until_ = Clock::now() + std::chrono::milliseconds(200);
      // 力寄存器只保存设定值，不模拟实际夹持力。
      } else if (address == 0x0101)
        force_ = value;
      // 目标开度写入后，状态改为移动中（0）。
      else if (address == 0x0103) {
        target_ = value;
        status_ = 0;
      } else
        throw std::runtime_error("mock unsupported write");
      return sent;
    }
    // 其余请求只允许功能码 3，且每次读取一个寄存器。
    if (sent[1] != 3 || value != 1)
      throw std::runtime_error("mock unsupported read");
    uint16_t answer;
    // 分别返回初始化状态、运动/夹持状态和取整后的开度。
    if (address == 0x0200)
      answer = init_;
    else if (address == 0x0201)
      answer = status_;
    else if (address == 0x0202)
      answer = uint16_t(std::lround(position_));
    else
      throw std::runtime_error("mock unsupported register");
    Bytes reply{sent[0], 3, 2, uint8_t(answer >> 8), uint8_t(answer)};
    auto crc = crc16(reply);
    reply.push_back(uint8_t(crc));
    reply.push_back(uint8_t(crc >> 8));
    return reply;
  }

private:
  // 按经过时间推进模拟设备；由收发触发，没有独立模拟线程。
  void advance() {
    auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - updated_).count();
    updated_ = now;
    // 模拟初始化的 200 ms 已结束时，设为已初始化、全开且到位。
    if (init_ == 2 && now >= until_) {
      init_ = 1;
      position_ = target_ = 1000;
      status_ = 1;
    }
    // 尚未初始化完成，或当前不在移动时，不更新开度。
    if (init_ != 1 || status_ != 0)
      return;
    double destination = target_;
    // 设置了物体，且闭合目标越过接触位置、当前尚未越过该位置时，模拟接触。
    bool contact = contact_ >= 0 && target_ < contact_ && position_ >= contact_;
    // 本次会接触物体时，只移动到接触位置。
    if (contact)
      destination = contact_;
    double step = 1000. * dt;  // 模拟速度：每秒 1000 原始开度单位，不代表实机速度。
    position_ += std::clamp(destination - position_, -step, step);
    // 距本次终点小于 0.01 时，更新为夹住（2）或到位（1）。
    if (std::abs(position_ - destination) < .01)
      status_ = contact ? 2 : 1;
  }

  int contact_, init_ = 0, status_ = 1, target_ = 1000, force_ = 20;
  double position_ = 1000;
  Clock::time_point updated_, until_{};
};
}  // namespace

// 创建真实 TCP 透传实现；首次读写时才连接。
std::unique_ptr<Transport> tcp_transport(const std::string &ip, int port, int timeout_ms) {
  return std::make_unique<Tcp>(ip, port, timeout_ms);
}

// 创建不访问硬件的模拟实现。
std::unique_ptr<Transport> mock_transport(int contact_raw) {
  return std::make_unique<Mock>(contact_raw);
}
}  // namespace chess_robot_gripper
