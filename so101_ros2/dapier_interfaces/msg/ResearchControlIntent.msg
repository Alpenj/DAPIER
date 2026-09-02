# Python/research proposal. This message never grants hardware authority.
string schema_version
uint64 sequence
string kind
string source
int64 source_monotonic_ns
int64 ttl_ns
string[] joint_names
float64[] joint_position_rad
float64[] joint_max_velocity_rad_s
float64 base_linear_x_mps
float64 base_angular_z_rad_s
bool control_authorized
