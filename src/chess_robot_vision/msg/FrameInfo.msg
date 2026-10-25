# Header shared with this RGB-D pair and CameraInfo. Stamp = host receipt, NOT exposure time.
std_msgs/Header header
uint64 color_frame_number
uint64 depth_frame_number
# Timestamp exactly as returned by the SDK in timestamp_domain (may be Global Time).
float64 device_timestamp_ms
string timestamp_domain
# SDK Z16 native units to metres. Published 32FC1 is already in metres.
float64 depth_scale_m
bool stream_ok
string diagnostic
