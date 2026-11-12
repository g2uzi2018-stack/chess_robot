#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

// Documented common subset of the project's vendor RealTimeData (1440 bytes).
// No firmware-dependent tail fields or controller clock units are inferred.
using FeedbackFrame = std::array<unsigned char, 1440>;
template<class T> T feedback_field(const FeedbackFrame& b, size_t offset) {
  if (offset > b.size() || sizeof(T) > b.size() - offset)
    throw std::runtime_error("feedback field outside frame");
  T value;
  std::memcpy(&value, b.data() + offset, sizeof(T));
  return value;
}
inline void validate_feedback(const FeedbackFrame& b) {
  const uint16_t endian = 1;
  if (*reinterpret_cast<const unsigned char*>(&endian) != 1)
    throw std::runtime_error("little-endian host required");
  if (feedback_field<uint16_t>(b, 0) != b.size() ||
      feedback_field<uint64_t>(b, 48) != 0x0123456789ABCDEFULL)
    throw std::runtime_error("feedback framing/profile mismatch");
  if (!std::isfinite(feedback_field<double>(b, 64)))
    throw std::runtime_error("nonfinite speed scaling");
  for (const size_t offset : {size_t(192), size_t(432), size_t(480), size_t(624)})
    for (size_t j = 0; j < 6; ++j)
      if (!std::isfinite(feedback_field<double>(b, offset + 8 * j)))
        throw std::runtime_error("nonfinite feedback");
}
class FeedbackSequence {
  bool have_last_ = false;
  uint64_t last_ = 0;
  FeedbackFrame last_frame_{};
public:
  bool accept(const FeedbackFrame& b) {
    validate_feedback(b);
    const auto stamp = feedback_field<uint64_t>(b, 32);
    if (have_last_ && stamp == last_ && b == last_frame_) return false;
    if (have_last_ && stamp <= last_)
      throw std::runtime_error("non-increasing controller timestamp");
    last_ = stamp;
    last_frame_ = b;
    have_last_ = true;
    return true;
  }
};
