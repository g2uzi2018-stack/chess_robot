std_msgs/Header header
bool simulated
bool connected
bool feedback_valid
float64 feedback_age_sec
bool busy
bool commands_enabled
# Unknown values are 255 (status) / 65535 (position).
uint8 initialization
uint8 grip_status
uint16 position_raw
# Device status: 0=moving, 1=at position, 2=object held, 3=object dropped.
# These booleans are false when feedback is invalid.
bool object_detected
bool object_dropped
string detail
