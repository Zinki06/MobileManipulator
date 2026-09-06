# Raw camera observation used for visual heading recovery.
# This message is independent from whether the observation was accepted as a
# map->odom correction.
std_msgs/Header header
int32 marker_id
# Horizontal angle in base_link: positive is left/CCW, negative is right/CW.
float64 bearing
float64 planar_distance
float64 spatial_distance
bool correction_accepted
