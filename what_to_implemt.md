# Swarm Flocking: Architecture & Roadmap

This document outlines the system architecture of our **Adaptive Decentralized Swarm Navigation System**, defining exact metrics, adaptive logic, closed-loop simulation models, and formal performance constraints.

---

## 🧠 Core System Prototype (What is Built)

The foundational prototype is a **reactive, decentralized control system** driven by local interaction rules. It features no central brain, memory, or predictive planning.

### Control & Communication
*   **Decentralized**: Each robot runs an independent `boid_node`.
*   **Peer-to-Peer**: Robots broadcast position and velocity globally over `/robot_i/pose_share` and `/robot_i/velocity_share`.
*   **Reactive**: Boid forces (Separation, Alignment, Cohesion, Obstacle Repulsion, Migration) sum strictly using current sensor/peer states.

### Observability
*   `flock_monitor_node.py` aggregates state data without interfering in control.
*   Calculates: Centroid, cohesion radius, real-time collision counts, and graph-based split detection.

---

## 🔥 Roadmap: From Prototype to Adaptive System

To elevate this project into a robust engineering system, we will implement the following mathematically rigorous evaluations, adaptive rules, and decoupled execution environments.

### 🥇 Priority 1: Define Explicit Success Criteria
Logging is not an objective. The system must natively classify the outcome of an experiment mathematically, with computable constraints.

**Success Criteria Equations:**
A run is classified as a *Success* internally **only if**:
1.  **Completion**: `∀i ∈ robots: dist(robot_i, goal) < ε`
2.  **Time Constraint**: `completion_time < T_max` (seconds)
3.  **Safety Constraint**: `collision_rate < X` 
    *   *Where* `collision event = dist(robot_i, robot_j) < d_collision`
    *   *And* `collision_rate = total_collisions / total_time`
4.  **Cohesion Constraint**: `mean_cohesion < Y` (meters)
    *   *Where* `mean_cohesion = sum(cohesion_radius(t)) / steps`

*(These metrics will be aggressively logged via CSV at run termination).*

### 🥈 Priority 2: Goal Completion Logic
The swarm must act as a task-oriented system rather than an endless loop simulation.
*   **Event Generation**: `flock_monitor_node` monitors individual arrived states.
*   **Termination Sequence**: Once all robots reach the ultimate waypoint, the system broadcasts a universal `Twist(0,0)` kill command.
*   **Accounting**: `flock_monitor_node` calculates final success criteria (above), outputs results, and safely tears down the experiment.

### 🥉 Priority 3: Adaptive Behavior (Stable & Bounded)
Static parameter weights fail under varied edge cases. We will shift from static weights to a contextual, real-time weight modulation system. Crucially, weights will be strictly bounded and properly scaled to prevent system divergence or unbounded explosions.

**Adaptive Logic (Exact Equations):**
```python
# 1. Crowding Response (Bounded Separation Increase)
crowding_factor = max(0, 1.0 - (avg_neighbor_distance / ideal_distance))
w_separation = clamp(base_sep + k_sep * crowding_factor, min_w, max_w)

# 2. Fragmentation Response (Multiplicative Cohesion Scaling)
spread_factor = max(0, local_cohesion_radius - threshold_spread)
w_cohesion = base_coh * (1.0 + alpha_coh * spread_factor)

# 3. Threat-Proximity Response (Safe Obstacle Scaling)
# Protect against divide-by-zero or infinite scaling using min_laser_dist and epsilon
safe_dist = max(min_laser_dist, epsilon)
scale = min(max_scale, hazard_threshold / safe_dist)
w_obstacle = base_obs * scale
```

### 🏅 Priority 4: Remove Gazebo Dependency (Closed-Loop Simulation)
Running large parameter sweeps in Gazebo is astronomically slow. We will build a headless simulator loop to decouple the logic.

**System Execution Flow (No Gazebo):**
1.  `boid_node` computes forces and publishes continuous `cmd_vel` requests.
2.  `physics_node` (New! A lightweight 2D kinematics engine) reads all synchronous velocity commands.
    *   *Assumption*: All `cmd_vel` inputs are sampled at the fixed timestep `dt` using last-known values (zero-order hold delay handling).
3.  **Kinematic Update Model**: At a strict synchronous update rate of `dt = 0.1s` (10 Hz):
    *   `x[t+1] = x[t] + v * cos(θ) * dt`
    *   `y[t+1] = y[t] + v * sin(θ) * dt`
    *   `θ[t+1] = θ[t] + ω * dt`
4.  `physics_node` publishes the new simulated `/odom` and `/tf` states safely back to each robot.
5.  `obstacle_generator` (Optional) creates synthetic wall constraints, injecting mock `/scan` geometries.
6.  *The simulation runs faster-than-real-time due to the absence of rendering and physics engine overhead.*

---

## ⚡ Stability & Trade-Off Analysis

This system operates as a **nonlinear, feedback-driven control system**. As such, stability and performance trade-offs are central to the evaluation.

### Stability Considerations
*   **Oscillations & Competing Forces**: Because collision avoidance (separation) and goal-seeking (migration/cohesion) directly oppose each other at bottleneck entries, the system is prone to oscillatory jitter.
*   **LPF Stabilization**: A tuned Low-Pass Filter (LPF) is applied to the final output velocity commands. This dampens high-frequency noise and sudden force spikes, acting as a critical stabilizing factor to prevent diverging control loops.
*   **Bounded Parameters**: As defined in the adaptive logic, strict clamping and maximum scale bounds are enforced. Without these bounds, exponential feedback loops (e.g., getting infinitely close to a wall causing an infinite obstacle force) would immediately destabilize the swarm kinematics.

---

## 📊 Experimental Methodology

To generate statistically significant evaluation data, the system will be subjected to the following experimental protocol:

**For each parameter configuration:**
1.  Run **K trials** using random initial robot positions (within a bounded spawn zone).
2.  Record metrics strictly at the conclusion of each trial:
    *   `completion_time`
    *   `collision_rate`
    *   `mean_cohesion`
3.  Report all results formally as **mean ± variance** to demonstrate system reliability.