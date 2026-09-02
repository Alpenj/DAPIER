# Sanitized command emitted by the safety bridge. A downstream sink still owns I/O.
string decision
string reason
string kind
uint64 sequence
string[] joint_names
float64[] joint_position_rad
float64[] joint_max_velocity_rad_s
float64 base_linear_x_mps
float64 base_angular_z_rad_s
int64 expires_at_monotonic_ns
bool dispatch_allowed
bool safe_stop_latched
bool hardware_execution
