#pragma once
// Adapted from chess_robot_validation; internal arm transport implementation.

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <netinet/tcp.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

class QueryFailure : public std::runtime_error {
public:
  std::string partial_reply;
  QueryFailure(const std::string &message, const std::string &partial)
      : std::runtime_error(message), partial_reply(partial) {}
};

using Steady = std::chrono::steady_clock;
inline long long steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Steady::now().time_since_epoch())
      .count();
}
inline long long utc_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

class Socket {
  int fd_ = -1;
  std::string buffered_;

  void wait_for(short events, Steady::time_point deadline) {
    for (;;) {
      const auto remaining = deadline - Steady::now();
      if (remaining <= Steady::duration::zero())
        throw std::runtime_error("socket deadline exceeded");
      const auto ms =
          std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
      pollfd p{fd_, events, 0};
      const int result = poll(&p, 1, static_cast<int>(ms));
      if (result < 0 && errno == EINTR)
        continue;
      if (result < 0)
        throw std::runtime_error(std::string("poll: ") + std::strerror(errno));
      if (result == 0)
        throw std::runtime_error("socket deadline exceeded");
      if (p.revents & POLLNVAL)
        throw std::runtime_error("invalid socket descriptor");
      // HUP can accompany the final buffered bytes; let recv/send/SO_ERROR
      // determine the result instead of discarding those bytes.
      if (p.revents & (events | POLLERR | POLLHUP))
        return;
    }
  }

  size_t receive_until(void *buffer, size_t n, Steady::time_point deadline) {
    for (;;) {
      wait_for(POLLIN, deadline);
      const auto got = recv(fd_, buffer, n, 0);
      if (got < 0 &&
          (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        continue;
      if (got < 0)
        throw std::runtime_error(std::string("recv: ") + std::strerror(errno));
      if (got == 0)
        throw std::runtime_error("connection closed before complete response");
#ifdef TCP_QUICKACK
      // QUICKACK is temporary on Linux; request it after each receive instead
      // of assuming a one-time setting permanently disables delayed ACKs.
      int quick_ack=1;
      (void)setsockopt(fd_,IPPROTO_TCP,TCP_QUICKACK,&quick_ack,sizeof(quick_ack));
#endif
      return static_cast<size_t>(got);
    }
  }

  void exact_until(void *buffer, size_t n, Steady::time_point deadline) {
    size_t done = 0;
    while (done < n)
      done +=
          receive_until(static_cast<char *>(buffer) + done, n - done, deadline);
  }
  void send_until(const std::string &command, Steady::time_point deadline) {
    size_t sent = 0;
    while (sent < command.size()) {
      wait_for(POLLOUT, deadline);
      const auto n = send(fd_, command.data() + sent, command.size() - sent, MSG_NOSIGNAL);
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
      if (n <= 0) throw std::runtime_error("send failed");
      sent += static_cast<size_t>(n);
    }
  }
  std::string reply_until(Steady::time_point deadline) {
    try {
      for (;;) {
        auto end = buffered_.find(';');
        if (end != std::string::npos) {
          if (end >= 16384) throw std::runtime_error("oversized reply");
          auto reply = buffered_.substr(0, end + 1);
          buffered_.erase(0, end + 1);
          return reply;
        }
        if (buffered_.size() >= 16384) throw std::runtime_error("oversized reply");
        char bytes[4096];
        auto count = receive_until(bytes, sizeof(bytes), deadline);
        buffered_.append(bytes, count);
      }
    } catch (const std::exception &e) {
      throw QueryFailure(e.what(), buffered_);
    }
  }

public:
  Socket(const std::string &ip, int port, int timeout_ms = 1500) {
    if (port < 1 || port > 65535)
      throw std::runtime_error("port must be 1..65535");
    if (timeout_ms <= 0)
      throw std::runtime_error("connect timeout must be positive");
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
      throw std::runtime_error("IPv4 literal required");
    fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0)
      throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
    try {
      int no_delay=1;
      if (setsockopt(fd_,IPPROTO_TCP,TCP_NODELAY,&no_delay,sizeof(no_delay))<0)
        throw std::runtime_error("cannot enable TCP_NODELAY");
      const int flags = fcntl(fd_, F_GETFL, 0);
      if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0)
        throw std::runtime_error(std::string("fcntl: ") + std::strerror(errno));
      const auto deadline =
          Steady::now() + std::chrono::milliseconds(timeout_ms);
      const int result =
          connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
      if (result < 0 && errno != EINPROGRESS && errno != EINTR)
        throw std::runtime_error(std::string("connect: ") +
                                 std::strerror(errno));
      if (result < 0)
        wait_for(POLLOUT, deadline);
      int error = 0;
      socklen_t len = sizeof(error);
      if (getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &len) < 0)
        throw std::runtime_error(std::string("SO_ERROR: ") +
                                 std::strerror(errno));
      if (error)
        throw std::runtime_error(std::string("connect: ") +
                                 std::strerror(error));
    } catch (...) {
      close(fd_);
      fd_ = -1;
      throw;
    }
  }
  Socket(const Socket &) = delete;
  Socket &operator=(const Socket &) = delete;
  ~Socket() {
    if (fd_ >= 0)
      close(fd_);
  }

  size_t receive_some(void *buffer, size_t capacity, int timeout_ms = 500) {
    if (capacity == 0 || timeout_ms <= 0)
      throw std::runtime_error("receive capacity and timeout must be positive");
    return receive_until(buffer, capacity,
                         Steady::now() + std::chrono::milliseconds(timeout_ms));
  }

  void exact(void *buffer, size_t n, int timeout_ms = 1500) {
    if (timeout_ms <= 0)
      throw std::runtime_error("read timeout must be positive");
    exact_until(buffer, n,
                Steady::now() + std::chrono::milliseconds(timeout_ms));
  }

  std::string query(const std::string &command, int timeout_ms = 3000) {
    if (timeout_ms <= 0)
      throw std::runtime_error("query timeout must be positive");
    // Sending and the complete reply share one budget. Fragments do not reset
    // it.
    const auto deadline = Steady::now() + std::chrono::milliseconds(timeout_ms);
    send_until(command, deadline);
    return reply_until(deadline);
  }
  void send_command(const std::string &command, int timeout_ms) {
    if (timeout_ms <= 0) throw std::runtime_error("invalid send deadline");
    send_until(command, Steady::now() + std::chrono::milliseconds(timeout_ms));
  }
  std::string receive_reply(int timeout_ms) {
    if (timeout_ms <= 0) throw std::runtime_error("invalid receive deadline");
    return reply_until(Steady::now() + std::chrono::milliseconds(timeout_ms));
  }
};
