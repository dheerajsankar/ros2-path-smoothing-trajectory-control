## Path Smoothing & Trajectory Control

This package implements  *“Path Smoothing and Trajectory Control in 2D Space”* using ROS 2 and Python. It contains:

- **Path smoothing** of discrete waypoints using a cubic B-spline.
- **Time-parameterised trajectory generation** with constant linear velocity.
- **Trajectory tracking controller** for a differential-drive (unicycle) robot.
- **Simple 2D simulator** that integrates `cmd_vel` and publishes odometry + RViz marker.

The architecture is deliberately modular: each stage (waypoints → smoothing → trajectory → control → simulation) is a separate ROS 2 node connected by standard messages.

---

### 1. Setup Instructions

- **Dependencies (system / ROS2):**
  - ROS 2 (tested with `rclpy`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`)
  - Python 3
  - Python libraries: `numpy`, `scipy`, `matplotlib` (for plotting)

- **Install package into your ROS 2 workspace**

From your workspace root (e.g. `/home/dsn/nav_ws`):

```bash
colcon build --packages-select navigation
source install/setup.bash
```

If `numpy` / `scipy` / `matplotlib` are missing, install them, for example:

```bash
sudo apt-get install python3-numpy python3-scipy python3-matplotlib
```

---

### 2. Execution Instructions

1. **Build and source the workspace.**

   ```bash
   cd /home/dsn/nav_ws
   colcon build --packages-select navigation
   source install/setup.bash
   ```

2. **Run the full navigation pipeline.**

   ```bash
   ros2 launch navigation navigation_launch.py
   ```

   This launches:
   - `waypoints` – publishes a small set of 2D waypoints.
   - `path_smoothening` – smooths the waypoints into a dense path.
   - `trajectory_gen` – assigns timestamps along the smoothed path.
   - `controller` – tracks the trajectory and publishes `cmd_vel`.
   - `sim_robot` – simulates a differential‑drive robot and publishes `odom` + a marker.

3. **Visualise in RViz (optional but recommended).**

   In another terminal:

   ```bash
   rviz2
   ```

   Configure displays:
   - Fixed frame: `map`
   - Add `Path` displays for topics:
     - `/waypoints`
     - `/smoothed_path`
     - `/trajectory`
   - Add `Odometry` or `Pose` display for `/odom`.
   - Add `Marker` display for `/robot_marker`.

4. **Configuring behaviour (example parameters).**

   You can override parameters on the command line, e.g.:

   ```bash
   ros2 run navigation path_smoothening path_smoothening --ros-args -p smoothing:=1.0 -p num_points:=200
   ros2 run navigation trajectory_gen trajectory_gen --ros-args -p velocity:=0.4
   ros2 run navigation controller controller --ros-args -p k_lin:=1.0 -p k_ang:=2.5 -p lookahead_points:=8
   ```

   or use a YAML parameter file and include it in your own launch file.

---

### 3. Architecture & Node Responsibilities

The system is organised as a **linear pipeline of ROS 2 nodes**:

1. **`waypoints` (`navigation/waypoints.py`)**
   - Publishes a `nav_msgs/Path` on the `waypoints` topic.
   - Currently uses a **hard-coded list of (x, y) waypoints**:
     `(0, 0), (1, 1), (2, 1), (3, 2)`.
   - In a real application this could be:
     - Loaded from configuration / map,
     - Produced by a global planner (e.g., A* or sampling‑based planner),
     - Provided by a mission planner / user interface.

2. **`path_smoothening` (`navigation/path_smoothening.py`)**
   - Subscribes: `waypoints` (`nav_msgs/Path`).
   - Publishes: `smoothed_path` (`nav_msgs/Path`).
   - Implements the **path smoothing** step using a *cubic B-spline*.
   - The logic is isolated in the method:
     - `smooth_path(path_msg, smoothing, num_points) -> Path`
   - Design choice: treat smoothing as **purely geometric** (only x/y positions, no timing), making this node reusable for other planners.

3. **`trajectory_gen` (`navigation/trajectory_gen.py`)**
   - Subscribes: `smoothed_path` (`nav_msgs/Path`).
   - Publishes: `trajectory` (`nav_msgs/Path`).
   - Implements **time parameterisation** assuming a **constant linear velocity**.
   - The method:
     - `_build_trajectory(path_msg) -> Path`
     - Accumulates arc length along the smoothed path; for each segment:
       $\Delta t = \frac{d_{\mathrm{seg}}}{v}$.
     - Encodes the desired time in each `PoseStamped.header.stamp`.
   - Design choice: keep the message type as `nav_msgs/Path` to stay ROS‑native and simple, instead of defining a custom trajectory message.

4. **`controller` (`navigation/controller.py`)**
   - Subscribes:
     - `trajectory` (`nav_msgs/Path`),
     - `odom` (`nav_msgs/Odometry`).
   - Publishes:
     - `cmd_vel` (`geometry_msgs/Twist`).
   - Implements a **lookahead-based trajectory tracker**:
     - Conceptually similar to *pure pursuit*:
       - Find a point ahead along the path,
       - Drive towards it while controlling heading error.
     - Uses proportional gains on:
       - Distance to target (`k_lin`),
       - Heading error (`k_ang`).
   - Additional features:
     - **Path change detection** (`path_change_eps`): 
       - Do not reset progress for every re‑publish,
       - Only reset when the new path is significantly different at its ends.
     - **Windowed nearest‑point search**:
       - Search around the current index in a window on the trajectory
         (efficient vs. full scan).
       - From the closest point, jump ahead by `lookahead_points` indices.
     - **Rotate‑in‑place behaviour**:
       - If `|yaw_error| > rotate_in_place_yaw`, linear velocity is suppressed (robot turns until mostly aligned).
     - **Goal detection**:
       - When the final point is reached within `goal_tolerance`, commands are set to zero.

5. **`sim_robot` (`navigation/sim_robot.py`)**
   - Subscribes: `cmd_vel` (`geometry_msgs/Twist`).
   - Publishes:
     - `odom` (`nav_msgs/Odometry`) in `map` frame,
     - `robot_marker` (`visualization_msgs/Marker`) for RViz.
   - Implements a **unicycle kinematic model**:
     - State: $(x, y, \theta)$.
     - Dynamics:
       - $\dot{x} = v \cos\theta$
       - $\dot{y} = v \sin\theta$
       - $\dot{\theta} = \omega$

     - Integrated forward with a fixed time step (0.05 s).
   - Design choice: keep the simulator **deterministic and ideal** (no noise, no slip, no dynamics), which makes controller behaviour easier to interpret and plot.

6. **Launch file (`launch/navigation_launch.py`)**
   - Starts all the above nodes in a single command and keeps the interfaces clean:
     - No node depends on the launch file implementation details; it only wires together topics.

This modular architecture makes it trivial to:
- Swap in a different smoother,
- Replace the controller,
- Replace simulation with a real robot node, without changing the rest.

---

### 4. Algorithms and Design Choices

#### 4.1 Path Smoothing – Cubic B-spline (`PathSmoothening`)

- Input: discrete waypoints $\{(x_i, y_i)\}$ for $i = 0 \dots N-1$.
- We use `scipy.interpolate.splprep` / `splev`:
  - Build a cubic B-spline with global smoothing factor `s`:
    - `s = smoothing` parameter (larger → smoother, more deviation).
  - Sample `num_points` evenly spaced parameters $u \in [0, 1]$ to get a dense set of $(x, y)$ points.
- Edge cases:
  - If there are fewer than 4 points, or the spline fit fails (e.g. due to duplicate points), the node logs a warning and **falls back to publishing the original path**.
- Design rationale:
  - B-splines provide $C^2$ continuity (continuous curvature), which is appropriate for smooth differential-drive motion.
  - Using SciPy keeps the implementation concise and numerically well-tested.

#### 4.2 Trajectory Generation – Constant Velocity (`TrajectoryGen`)

- Objective: convert a geometric path into a **time-stamped trajectory**.
- Algorithm:
  1. Walk along the sequence of smoothed points.
  2. For each segment between consecutive points:
     - Compute distance $d = \sqrt{\Delta x^2 + \Delta y^2}$.
     - Compute $\Delta t = d / v$ where $v$ is the configured constant velocity.
     - Accumulate time: $t \leftarrow t + \Delta t$.
  3. Set `PoseStamped.header.stamp = base_time + t`.

#### 4.3 Trajectory Tracking Controller (`Controller`)

- The controller implements a **lookahead policy**:

  1. **Find a nearby point on the trajectory:**
     - Within a window [i_min, i_max] around the current index, pick the index with the minimum squared distance to the robot.
  2. **Look ahead** by a configurable number of indices (`lookahead_points`) to anticipate motion.
  3. Compute target point $(x_t, y_t)$ and current pose $(x, y, \theta)$.
  4. Compute:
     - Position error: $d = \sqrt{(x_t - x)^2 + (y_t - y)^2}$.
     - Desired heading: $\theta_d = \mathrm{atan2}(y_t - y, x_t - x)$.
     - Heading error: $e_\theta = \mathrm{wrapToPi}(\theta_d - \theta)$.
  5. Control laws:
     - Angular velocity: $\omega = \mathrm{sat}(k_{\text{ang}} \cdot e_\theta, \pm \omega_{\max})$

     - Linear velocity: $v = \max(0, \min(k_{\text{lin}} \cdot d, v_{\max}))$

       If $|e_\theta| > \theta_{\text{rotate}}$, set $v = 0$ (rotate in place).

  6. When the robot is close to the final goal (distance < `goal_tolerance`), command $v = 0$, $\omega = 0$.

- Design considerations:
  - Lookahead avoids “orbiting” behaviour around a single target waypoint.
  - Gains and thresholds are exposed as **ROS parameters**, so they can be tuned without code changes.
  - Controller is purely kinematic; it assumes low‑level wheel controllers track `cmd_vel`.

---

### 5. Plots and Evaluation

To evaluate tracking performance, you can log and plot:
- The **reference trajectory** (`/trajectory`),
- The **actual robot path** reconstructed from `/odom`.

Two straightforward workflows:

#### 5.1 Built-in plotting node (`traj_plotter`)

1. **Run the navigation pipeline** (as above), then in another terminal:

   ```bash
   ros2 run navigation traj_plotter
   ```

2. Let the system run until the robot finishes the trajectory, then press **Ctrl-C** in the `traj_plotter` terminal.

3. A PNG file named `trajectory_tracking.png` is saved in the current working directory, containing:
   - Reference trajectory from `/trajectory`,
   - Executed path reconstructed from `/odom`.

This PNG can be used directly in your submission report or slides.

#### 5.2 Using rosbag + offline plotting

1. **Record a rosbag**

   ```bash
   ros2 bag record /trajectory /odom
   ```

2. **Offline plotting script (conceptual outline)**

   Use Python + `rosbag2_py` or export to CSV and then:

   ```python
   import matplotlib.pyplot as plt

   # xs_ref, ys_ref from /trajectory
   # xs_odom, ys_odom from /odom
   plt.figure()
   plt.plot(xs_ref, ys_ref, 'b-', label='Reference trajectory')
   plt.plot(xs_odom, ys_odom, 'r--', label='Robot path')
   plt.xlabel('x [m]')
   plt.ylabel('y [m]')
   plt.axis('equal')
   plt.legend()
   plt.title('Trajectory tracking')
   plt.show()
   ```

