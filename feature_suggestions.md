# Feature Suggestions & Technical Improvements

Low-to-medium difficulty enhancements for the swarm flocking project.

---

## 🚀 Quick Wins (Low Effort)

### 1. Increase Robot Speed
Currently `max_linear_vel` is 0.20 m/s. The TurtleBot3 Burger supports up to 0.22 m/s.
- **Change**: Bump `max_linear_vel` to `0.22` in `flocking_params.yaml`
- **Also**: Increase `w_migration` from 0.3 → 0.5–0.8 so robots move toward goals more aggressively
- **Impact**: Noticeably faster flock movement without breaking stability

### 2. Reduce Low-Pass Filter Lag
`LPF_ALPHA = 0.4` in `boid_node.py` makes the robots sluggish and slow to react.
- **Change**: Increase to `0.6` or `0.7` for snappier response
- **Impact**: Robots respond faster to force changes while still filtering jitter

### 3. Add Logging / Console Output for Flock Metrics
`flock_monitor_node.py` publishes to topics but has no console output.
- **Change**: Add periodic `self.get_logger().info(...)` every ~5 seconds printing centroid, avg_speed, cohesion_radius, and collision_count
- **Impact**: Much easier to observe flock behavior without needing `ros2 topic echo`

### 4. Color-Coded RViz Markers for Split Detection
When the flock splits, the convex hull stays a single green color.
- **Change**: Color the hull marker red when `is_split=True`, green when cohesive
- **Impact**: Immediate visual feedback on flock fragmentation in RViz

---

## 🔧 Medium Effort Improvements

### 5. Dynamic Parameter Reconfiguration (Callback)
Currently parameters are read once at startup. `ros2 param set` changes the parameter store but the node doesn't re-read the values.
- **Change**: Add a `ParameterEventHandler` or `add_on_set_parameters_callback` to `BoidNode` so weight changes take effect immediately
- **Impact**: True live-tuning of flocking weights while the simulation runs

### 6. Speed-Dependent Separation
Right now separation uses a fixed radius. Fast-moving robots need more clearance.
- **Change**: Scale `separation_radius` by current linear velocity: `effective_sep_r = sep_r + k * speed`
- **Impact**: Fewer collisions when robots are moving quickly

### 7. Obstacle Threshold Scaling
`obstacle_threshold` is 0.6m — quite aggressive, can cause oscillations near walls.
- **Change**: Increase to 0.8–1.0m, and add a proportional ramp so force builds gradually (the ramp exists in the code but threshold is too low to see the gradient)
- **Impact**: Smoother wall following and fewer oscillation-jitter scenarios

### 8. Per-Robot RViz Markers (Arrow + ID Label)
Hard to tell which robot is which in RViz.
- **Change**: Publish an `ARROW` marker and a `TEXT_VIEW_FACING` marker per robot showing its ID and heading direction
- **Node**: Add this to `flock_monitor_node.py`
- **Impact**: Visual debugging becomes much easier

### 9. Waypoint Visualization
Waypoints are invisible in Gazebo and RViz.
- **Change**: Publish SPHERE markers at each waypoint position, green for current target, grey for future ones, and remove completed ones
- **Impact**: Visually track the flock's progress toward goals

### 10. Goal Arrival Detection & Stop
Robots keep moving after reaching the final waypoint (migration returns (0,0) but other forces still apply).
- **Change**: In `boid_node.py`, when `current_wp >= len(waypoints)`, publish a zero `cmd_vel` and log "goal reached"
- **Also**: Make `flock_monitor_node` detect when all robots have reached the goal and publish a flag in `FlockState`
- **Impact**: Clean simulation end state

### 11. Simple Data Logging to CSV
No built-in way to record metrics over time for plotting.
- **Change**: Add a `--log-csv` option to `flock_monitor_node` that appends timestamp, centroid, avg_speed, cohesion_radius, collision_count, is_split to a CSV file
- **Impact**: Ready-made data for analysis plots and reports

### 12. Stagger Boid Start with Random Delay
All boid nodes start at the same delay offset — if Gazebo is slow, they all fail together.
- **Change**: Add a small random jitter (0.0–0.5s) to each boid's TimerAction period
- **Impact**: More robust startup and less contention

---

## 📊 Analysis & Evaluation Improvements

### 13. Add `open_field.world` to Launch Options
The `open_field.world` exists but there's no launch parameter to select it.
- **Change**: Add a `world_name` launch argument to `full_sim.launch.py` (default: `obstacle_course`, option: `open_field`)
- **Impact**: Easy switching between worlds for testing flocking vs. obstacle scenarios

### 14. Simple Collision Rate Metric
`FlockState` has `collision_count` but no rate indicator.
- **Change**: Add `float32 collision_rate` (collisions per second over a sliding window of ~10s) to `FlockState.msg`
- **Impact**: Better metric for evaluating parameter tuning

### 15. Boid Force Debug Publisher
No visibility into what individual force components look like per robot.
- **Change**: Publish a `MarkerArray` of colored arrows from each robot showing separation (red), alignment (blue), cohesion (green), obstacle (yellow), and migration (purple) forces
- **Impact**: Essential for debugging why robots behave unexpectedly

---

## ⚡ Performance Improvements

### 16. Switch to `BEST_EFFORT` QoS for cmd_vel
Currently `cmd_vel` uses `RELIABLE` QoS which can cause message queuing.
- **Change**: Switch `cmd_pub` to `SENSOR_QOS` (best-effort, depth=1)
- **Impact**: Lower latency for velocity commands, especially with many robots

### 17. Reduce Neighbour Search from O(N²) to Grid-Based
With 10+ robots, the double loop in `_get_valid_neighbours` and `flock_monitor` is expensive.
- **Change**: Implement a simple spatial grid (cell size = neighbor_radius). Only check adjacent cells.
- **Impact**: Scales better for 15–20+ robots

### 18. Reduce FlockMonitor Rate for Large Swarms
2 Hz with N² distance checks is expensive.
- **Change**: Make the timer period configurable (parameter `monitor_rate_hz`, default 2.0)
- **Impact**: Can reduce to 1 Hz for large swarms without losing much insight
