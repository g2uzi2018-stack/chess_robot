# Host receipt time shared with the corresponding images; NOT hardware exposure time.
std_msgs/Header header
uint64 frame_number
float64 hardware_timestamp_ms
string timestamp_domain
bool stream_ok
string diagnostic
# Empty while stream_ok=false. Array indices are per-frame, not tracking IDs.
PieceCandidate[] candidates
