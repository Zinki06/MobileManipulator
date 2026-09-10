# OpenCR proxy telemetry. These fields do not expose individual servo error/mode registers.
std_msgs/Header header
bool read_ok
bool command_ok
bool ros_connected
bool manipulator_connected
# Aggregate: false means at least one manipulator motor is not enabled.
bool all_joints_torque_enabled
uint32 board_millis
float64 requested_position
float64 position
float64 velocity
int16 present_current_raw
int16 goal_current_raw
# Refreshed through OpenCR register 222; firmware does not report servo read failure.
bool goal_refresh_ok
float64 goal_position_readback
string fault
