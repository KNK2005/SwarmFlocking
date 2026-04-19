# Swarm Flocking: Architecture Upgrades & Testing Guide

This document outlines the major architectural upgrades implemented in the `new-features` branch. The system has transitioned from a visually-driven reactive simulation in Gazebo to a **measurable, adaptive, and headless experimental control system**.

---

## 🚀 1. What Changes Were Implemented

### 1.1 Strictly Bounded Adaptive Weights (`boid_node.py`)
Static parameter weights (`w_separation = 1.5`) fail in edge cases. The `boid_node.py` now implements real-time, context-aware weight modulation.
*   **Crowding Response**: As robots get squeezed together, the separation weight mathematically scales up.
*   **Fragmentation Response**: As the flock's cohesion radius expands beyond a safe threshold, the cohesive pull scales up multiplicatively to snap them back together.
*   **Mathematical Stability**: Every adaptive multiplier is strictly wrapped in `clamp(value, min, max)` functions. This ensures that nonlinear feedback loops (like getting infinitely close to a wall) do not result in infinite force vectors that explode the simulation.

### 1.2 The Experiment Manager (`flock_monitor_node.py`)
The monitor node no longer just passively gathers metrics—it actively enforces success criteria and manages the experiment lifecycle.
*   **Goal Completion**: It detects when *all* `N` robots are within 1.2m of the final waypoint. 
*   **Safe Termination**: Upon completion, it instantly broadcasts a universal `Twist(0, 0)` command to halt the swarm, writes the final statistical metrics to a CSV, waits 1.0 second to guarantee OS buffer flushes, and invokes a graceful `sys.exit(0)`.
*   **New Metrics**: Tracks total elapsed time, collision rate (collisions per second), and tracks the mean cohesion radius over the life of the run.

---

## 🛠 2. What New Parts Were Added

### 2.1 The 2D Kinematics Engine (`physics_node.py`)
Testing Swarm scalability or running 100 parameter sweeps in Gazebo is astronomically slow and CPU-heavy. To solve this, we bypassed Gazebo entirely by writing a proprietary physics node.
*   **Zero-Order Hold Mechanics**: The node subscribes to all `/robot_i/cmd_vel` topics and caches their inputs.
*   **Synchronous Execution**: Every 0.1s (10 Hz), the engine computes exact differential drive kinematics (`x += vx * cos(theta) * dt`) and absolute angle normalizations.
*   **Output**: It publishes standard `nav_msgs/Odometry` and injects mock `/scan` arrays natively back into the ROS2 network so the `boid_node` logic remains completely unchanged. 

### 2.2 Headless Execution (`headless_sim.launch.py`)
A brand new launch configuration that spins up:
1.  The `physics_node`
2.  The `flock_monitor_node`
3.  An array of `N` decentralized `boid_node` instances.

**Impact**: Executing experiments no longer requires rendering Gazebo or launching heavy robot state publishers. The code runs deterministically off the CPU, effectively resulting in faster-than-real-time execution. To prevent desync issues in the zero-order hold, `use_sim_time` is strictly enforced to `False` across the entire stack.

---

## 🧪 3. How to Test the New System (Ubuntu)

For teammates executing these features on Ubuntu machines, follow the protocol below to validate the system.

### Step 1: Build the Upgraded Interfaces
Because we added robust telemetry fields (`total_time`, `collision_rate`, `mean_cohesion`) to the custom ROS2 messages, you **must rebuild the workspace** first.
```bash
cd ~/swarm_flocking_ws
colcon build --packages-select swarm_interfaces swarm_flocking
source install/setup.bash
```

### Step 2: Run the Headless Simulation
Execute the new headless launch file. Because it doesn't wait on Gazebo grids to load, the robots will instantly begin flocking mathematically in the background.
```bash
ros2 launch swarm_flocking headless_sim.launch.py
```

### Step 3: Observe the Automated Teardown
Watch the terminal output closely. 
1.  The monitor will log the real-time cohesion statuses.
2.  Once the final waypoint is physically reached by all bots in the `physics_node` space, you will see an `EXPERIMENT COMPLETE!` log.
3.  The system will gracefully shut down its own nodes automatically.

### Step 4: Validate the Data
Open the generated CSV in the directory you launched from:
```bash
cat experiment_results.csv
```
You should see a clean record mapping out exactly how many robots ran, how fast they finished, their formal collision rate, and how well the flock maintained cohesion!

### 📊 Recommended Experimental Methodology
To generate statistically significant academic data:
1.  Run `K = 5 to 10` trials using the standard base node parameters.
2.  Run `K = 5 to 10` trials heavily tweaking the Adaptive Scaling constants (e.g., `k_sep`, `alpha_coh`) in the launch file.
3.  Compare the results to mathematically prove the trade-off tension between tight Cohesion and high Collision Rates.
