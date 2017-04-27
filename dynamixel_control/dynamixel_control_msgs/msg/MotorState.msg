# Published by dynamixel_controll
# Representing status of Dynamixels in present.

std_msgs/Header header
string motor_model
int64 id
bool torque_enable
int64 max_position_limit
int64 min_position_limit
int64 ccw_angle_limit
int64 cw_angle_limit
int64 goal_position
int64 goal_velocity
int64 goal_current
int64 moving_speed
int64 goal_acceleration
int64 profile_velocity
int64 profile_acceleration
int64 present_position
int64 present_velocity
int64 present_current
bool moving
