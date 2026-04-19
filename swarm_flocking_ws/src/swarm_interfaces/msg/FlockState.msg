# FlockState.msg — Aggregated flock-wide metrics published by flock_monitor_node

std_msgs/Header header

# Center of mass of the entire flock (world frame)
geometry_msgs/Point centroid

# Mean distance of all robots from the centroid (meters)
float32 cohesion_radius

# Mean linear speed across all active robots (m/s)
float32 avg_speed

# Number of robots currently publishing pose data
int32 num_active_robots

# Cumulative count of robot-robot near-collision events detected
int32 collision_count

# True when graph connectivity analysis shows > 1 connected component
bool is_split

# Number of detected sub-groups when is_split is true
int32 num_subgroups

# Total elapsed time since the monitor started receiving data (seconds)
float32 total_time

# Rolling average of collisions per second
float32 collision_rate

# Rolling average of the cohesion radius (meters)
float32 mean_cohesion

# True if all active robots are within goal_tolerance of final waypoint
bool goal_reached

# True only when the run satisfies configured success criteria
bool experiment_success

# One-word or compact reason (e.g. running, success, timeout_before_goal)
string completion_reason
