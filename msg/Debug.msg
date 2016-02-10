# header time is set to when we send the message (update loop_
Header header

bool control_active

# bebop sync time is set when we receive a state
time beb_sync_time

# the lag between header.stamp and bebop_sync_time
duration beb_sync_lag

float32 beb_maxtilt_rad
float32 beb_max_speed_vert_m
float32 beb_max_speed_rot_rad

float32 beb_roll_rad
float32 beb_pitch_rad
float32 beb_yaw_rad
float32 beb_alt_m

float32 beb_vx_enu
float32 beb_vy_enu
float32 beb_vz_enu

float32 beb_vx_m
float32 beb_vy_m
float32 beb_vyaw_rad

# only transimission delay compensated
float32 beb_vx_pred_delay_m
float32 beb_vy_pred_delay_m

# transmission delay + feedback lag compensated
float32 beb_vx_pred_m
float32 beb_vy_pred_m

time setpoint_time
duration setpoint_lag

geometry_msgs/Twist setpoint
geometry_msgs/Twist setpoint_filt
