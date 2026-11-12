#pragma once
#include <charconv>
#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <string>

struct DashboardReply {
  int64_t error;
  std::string payload;
};
inline std::string reply_normalize(const std::string& value) {
  std::string normalized;
  for (unsigned char c : value)
    if (!std::isspace(c)) normalized += static_cast<char>(std::tolower(c));
  return normalized;
}
// ErrorID,{values},echoed_command; — values may contain nested arrays/braces.
inline DashboardReply parse_dashboard_reply(const std::string& raw, const std::string& command) {
  const auto text = reply_normalize(raw);
  const auto comma = text.find(',');
  if (comma == std::string::npos) throw std::runtime_error("missing reply error code");
  int64_t error = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + comma, error);
  if (parsed.ec != std::errc() || parsed.ptr != text.data() + comma)
    throw std::runtime_error("invalid reply error code");
  const auto start = comma + 1;
  if (start >= text.size() || text[start] != '{') throw std::runtime_error("missing reply payload");
  int depth = 0;
  size_t end = start;
  for (; end < text.size(); ++end) {
    if (text[end] == ';') throw std::runtime_error("unexpected reply terminator");
    if (text[end] == '{') ++depth;
    if (text[end] == '}' && --depth == 0) break;
  }
  if (end == text.size() || end + 1 >= text.size() || text[end + 1] != ',')
    throw std::runtime_error("unbalanced/missing reply payload delimiter");
  if (text.substr(end + 2) != reply_normalize(command) + ';')
    throw std::runtime_error("reply command echo mismatch");
  return {error, text.substr(start + 1, end - start - 1)};
}
