# Unclassified circle proposal, NOT an accepted chess piece or grasp command.
float32 pixel_u
float32 pixel_v
float32 radius_px
string label
# Surface point on visible top, metres, optical axes +X right +Y down +Z forward.
# When depth_valid=false, surface_point is NaN. No orientation/base transform implied.
bool depth_valid
geometry_msgs/Point surface_point
float32 depth_mad_m
float32 depth_valid_fraction
uint32 depth_samples
