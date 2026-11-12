#pragma once
#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace chess_robot_arm {
class AuditDirectory {
  int fd_=-1;
public:
  explicit AuditDirectory(const std::string &path);
  ~AuditDirectory();
};
// Diagnostic only: bounded asynchronous queue, whole-record rotation, bounded retention.
// Disk/queue failure cannot block the feedback receiver or become a motion reference.
class RotatingAudit {
  std::string path_,header_;
  size_t maximum_,keep_,queued_bytes_=0;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::string> queue_;
  bool stopping_=false;
  std::thread worker_;
  std::atomic<uint64_t> dropped_{0},written_{0},generation_{0};
  std::string error_;
  void run();
public:
  RotatingAudit(std::string path,size_t maximum,size_t keep,std::string header="");
  ~RotatingAudit();
  bool write(std::string record) noexcept;
  uint64_t dropped() const {return dropped_;}
  std::string error();
};
} // namespace chess_robot_arm
