# This data structure contains the RobotModeData structure
# used by the Universal Robots controller
#
# This data structure is send at 10 Hz on TCP port 30002

uint64 timestamp
bool is_robot_connected
bool is_real_robot_enabled
bool is_power_on_robot
bool is_emergency_stopped
bool is_protective_stopped
bool is_program_running
bool is_program_paused
uint8 robot_mode
uint8 control_mode
float64 target_speed_fraction
float64 speed_scaling

# Extension: robot_mode == ROBOT_MODE_RUNNING
bool is_ready
