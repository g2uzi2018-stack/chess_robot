#pragma once
#include "chess_robot_arm/core.hpp"
#include <mutex>

namespace chess_robot_arm {
// Linux durable atomic file with one writer lock. Raw audit logs are independent.
class CheckpointStore {
  Config config_;
  std::string identity_;
  int lock_fd_=-1;
  std::mutex mutex_;
  void write(const std::string &payload);
public:
  explicit CheckpointStore(const Config &);
  ~CheckpointStore();
  CheckpointStore(const CheckpointStore &)=delete;
  void save(const Snapshot &);
  // Persist BEFORE any motion send. A power cut during motion cannot use an old save.
  void markMotionPending();
  chess_robot_twist::TrackerCheckpoint load();
  const std::string &identity() const { return identity_; }
};
} // namespace chess_robot_arm
