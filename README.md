# TorQ-R1 Autonomous Simulation & SLAM Mapping System

A dedicated, industrial-grade ROS 2 Jazzy workspace for the **TorQ-R1** mobile robot, featuring high-fidelity Gazebo Harmonic simulation, true coaxial differential drive kinematics, real-time 2D Pose Graph SLAM (Ceres Solver), and noise-free RViz2 mapping.

### 📚 Dedicated Technical Documentation
- **[SLAM Technical Specification & Algorithm Manual](docs/SLAM_README.md)**: Scan matching, Ceres Solver formulations, filter stack, and loop closure gating.
- **[Autonomous Navigation & Exploration Manual](docs/AUTONOMOUS_NAVIGATION_README.md)**: Yamauchi frontier exploration, BFS grid planning, obstacle dilation, pure pursuit, and anti-looping blacklisting.

---

## 1. System Architecture & Package Hierarchy

The workspace strictly adheres to the **ROS 2 REP-144 / REP-2000** standard, isolating only navigation, SLAM, physical kinematics, and Gazebo simulation models:

```
TorQ-R1/
├── README.md                          # Master TorQ-R1 documentation
├── docs/                              # Dedicated engineering manuals
│   ├── SLAM_README.md                 # Deep-dive on Ceres SLAM & Filters
│   └── AUTONOMOUS_NAVIGATION_README.md# Deep-dive on Yamauchi Exploration
├── scripts/                           # Workspace automation & operational scripts
│   ├── run_slam_demo.sh              # One-click SLAM demo launcher
│   ├── launch_teleop.sh              # Dedicated arrow-key teleoperation launcher
│   ├── save_map.sh                   # Map saver CLI wrapper (.pgm + .yaml)
│   ├── stop_mapping.sh               # Graceful map save & shutdown script
│   └── setup_workspace.sh            # Environment setup & GZ resource path exporter
├── maps/                              # Default output directory for saved maps
└── src/
    ├── torq_r1_description/          # [ament_cmake] Physical kinematics, meshes & URDF
    │   ├── meshes/acker/              # High-fidelity CAD STL meshes
    │   ├── urdf/
    │   │   ├── torq_r1.urdf.xacro     # Master robot URDF/xacro entry point
    │   │   ├── ack_diff.xacro         # Coaxial differential drive & decoupled contact friction
    │   │   ├── gazebo_plugins.xacro   # DiffDrive & rigid-body OdometryPublisher plugins
    │   │   ├── sensors.xacro          # LiDAR ray-tracing & camera definitions
    │   │   ├── imu.urdf.xacro         # 6-DOF IMU sensor definition
    │   │   ├── inertial_matrix.xacro  # Standard inertia calculation macros
    │   │   └── materials.xacro        # Visual color materials
    │   ├── rviz/                      # Robot model verification displays
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── torq_r1_gazebo/               # [ament_cmake] Gazebo Harmonic simulation stack
    │   ├── config/
    │   │   └── ros_gz_bridge.yaml     # Bidirectional ROS 2 <-> Gazebo Sim bridge config
    │   ├── launch/
    │   │   ├── torq_r1_slam.launch.py # Master orchestration pipeline (Gazebo + SLAM + RViz)
    │   │   └── torq_r1_sim.launch.py  # Standalone Gazebo simulation launcher
    │   ├── worlds/
    │   │   ├── maze_world.sdf         # 16m x 16m walled maze environment
    │   │   ├── obstacle_world.sdf     # Obstacle course environment
    │   │   └── torq_arena.sdf         # Boundary arena environment
    │   ├── rviz/
    │   │   ├── torq_r1_slam.rviz      # High-visibility, noise-filtered RViz2 profile
    │   │   └── torq_r1.rviz           # General robot inspection profile
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── torq_r1_navigation/           # [ament_python] SLAM & Navigation intelligence
        ├── config/
        │   └── slam_toolbox_params.yaml # Tuned Ceres Graph-SLAM parameter set
        ├── launch/
        │   ├── slam.launch.py         # Standalone SLAM node launcher
        │   └── teleop.launch.py       # Standalone teleoperation launcher
        ├── maps/
        │   ├── obstacle_world_map.pgm
        │   └── obstacle_world_map.yaml
        ├── torq_r1_navigation/
        │   ├── arrow_teleop_node.py   # Non-snapping smooth arrow-key controller
        │   ├── path_publisher_node.py # Odometry-driven trajectory path generator
        │   ├── obstacle_avoidance_node.py # Reactive LiDAR obstacle avoidance node
        │   └── telemetry_node.py      # Telemetry & status monitor node
        ├── setup.py
        ├── setup.cfg
        └── package.xml
```

---

## 2. Technical Specifications & Method Names

| Parameter | Specification | Details / Standard |
| :--- | :--- | :--- |
| **Robot Model** | **TorQ-R1** | Differential-drive mobile robot |
| **Actuation Method** | **Coaxial Differential Drive** | Mid-axis dual drive wheels ($x=0.0\text{ m}, y=\pm 0.070\text{ m}$) with 4 rotating visual wheels |
| **Kinematic Model** | **Unicycle / Dual-Wheel Kinematics** | Track width $L = 0.140\text{ m}$, Wheel radius $R = 0.033\text{ m}$ |
| **SLAM Method** | **Asynchronous 2D Pose Graph SLAM** | `slam_toolbox` asynchronous online mapping |
| **Graph Optimization** | **Sparse Pose Graph Optimization (PGO)** | Karto-based non-linear factor graph with rigid transformation constraints |
| **Optimization Solver** | **Google Ceres Solver** (`solver_plugins::CeresSolver`) | Levenberg-Marquardt trust-region with Sparse Normal Cholesky linear decomposition |
| **Visualization Platform** | **RViz2 (ROS 2 Visualization Engine)** | Fixed Frame: `map` |
| **Odometry Source** | **Rigid-Body Physics Odometry** | `gz-sim-odometry-publisher-system` at 50 Hz |

---

## 3. Algorithms Used in SLAM

1. **Ceres Scan Matcher (`CeresScanMatcher`)**:
   - Performs non-linear least-squares scan registration between incoming planar laser scans and the current occupancy submap.
   - Solves:
     $$\min_{(x, y, \theta)} \sum_{i} \left( 1 - M\left( T(x, y, \theta) \cdot p_i \right) \right)^2 + w_{\text{dist}} \Delta d^2 + w_{\text{ang}} \Delta \theta^2$$
   - Uses **Sparse Normal Cholesky** linear solver with **Schur-Jacobi** preconditioner and **Traditional Dogleg** step calculation.
2. **Multi-Resolution Correlative Matching**:
   - Evaluates scan likelihood over a $0.5\text{ m} \times 0.5\text{ m}$ search window with $0.01\text{ m}$ spatial resolution and $0.01745\text{ rad}$ ($1^\circ$) coarse angular resolution.
3. **Loop Closure Detection & Factor Graph Optimization**:
   - Branch-and-bound scan search within a $7.0\text{ m}$ loop search radius.
   - Enforces **Mahalanobis Distance & Chi-Square ($\chi^2$) Gating** to discard false-positive loop closures and eliminate ghost walls.

---

## 4. Filter Names & Noise Mitigation Stack

1. **Ray-Trace Free-Space Carving Filter (Bresenham Ray Casting)**:
   - Dynamic clearing of phantom obstacles along laser rays up to `max_laser_range: 12.0m`.
2. **Near-Field Blindzone Geometric Filter**:
   - Physical clipping configured in `sensors.xacro` (`<min_ray_range>0.16</min_ray_range>`) and `slam_toolbox_params.yaml` (`min_laser_range: 0.18`), eliminating self-reflections off the robot bumper and wheel arches.
3. **Contact Friction & Surface Decoupling Filter**:
   - Central drive wheels: High rubber friction ($\mu_1 = 2.5, \mu_2 = 2.5, k_p = 10^5, k_d = 10^2$).
   - Front & rear support axles: Friction-free frictionless gliders ($\mu_1 = 0.0, \mu_2 = 0.0$), eliminating ground-scrub friction locks and lateral drift.
4. **Odometry Ingestion & Covariance Filter**:
   - Direct integration of 50 Hz Gazebo rigid-body state estimation (`gz-sim-odometry-publisher-system`), completely eliminating wheel-slip integration drift during tight in-place turns.
5. **Scan Rate Decimation & Temporal Throttling**:
   - Scan throttling (`throttle_scans: 1`, `minimum_time_interval: 0.04s`, 25 Hz) and map publisher throttling (`map_update_interval: 0.25s`, 4 Hz), preventing CPU starvation and message queue backpressure.

---

## 5. Visualization Setup & RViz2 Displays

The RViz2 display configuration (`torq_r1_gazebo/rviz/torq_r1_slam.rviz`) has been calibrated for industrial clarity:

| Display Name | RViz Plugin Type | Topic / Frame | Configuration / Style |
| :--- | :--- | :--- | :--- |
| **Grid** | `rviz_default_plugins/Grid` | `map` (Reference) | Plane: XY, Cell Size: 1.0m, Subdivisions: 1 |
| **Map** | `rviz_default_plugins/Map` | `/map`, `/map_updates` | Scheme: `map`, Durability: Transient Local |
| **LaserScan** | `rviz_default_plugins/LaserScan` | `/scan` | Style: Flat Squares, Size: 0.04m, Color: Neon Green (`rgba(0, 255, 120, 1.0)`) |
| **Path Trajectory** | `rviz_default_plugins/Path` | `/trajectory_path` | Style: Lines, Line Width: 0.03m, Color: Cyan (`rgba(0, 220, 255, 1.0)`) |
| **RobotModel** | `rviz_default_plugins/RobotModel` | `/robot_description` | Visual: Enabled, Collision: Disabled (no red bounding boxes) |
| **TF Frames** | `rviz_default_plugins/TF` | `/tf`, `/tf_static` | **Disabled** (clears messy yellow/red arrow clutter) |

---

## 6. Execution & Operation Guide

### Step 1: Source the Workspace
```bash
source /opt/ros/jazzy/setup.bash
source /home/akszayan/TorQ-R1/install/setup.bash
```

### Step 2: Launch the Master Simulation & SLAM Stack
```bash
/home/akszayan/TorQ-R1/scripts/run_slam_demo.sh
```
*This starts Gazebo Harmonic with the 16m x 16m maze, SLAM Toolbox, RViz2, and launches the dedicated arrow teleoperation terminal.*

### Step 3: Interactive Teleoperation Controls
If running manually in a separate terminal:
```bash
/home/akszayan/TorQ-R1/scripts/launch_teleop.sh
```
- **$\uparrow$ Up Arrow**: Drive Forward
- **$\downarrow$ Down Arrow**: Drive Reverse
- **$\leftarrow$ Left Arrow**: Turn Left (Smooth coaxial pivot)
- **$\rightarrow$ Right Arrow**: Turn Right (Smooth coaxial pivot)
- **`+` / `-`**: Increase / Decrease Speed
- **`1` / `2` / `3`**: Steering Presets (1: Precision `0.9 rad/s`, 2: Standard `1.4 rad/s`, 3: Quick `2.2 rad/s`)
- **Spacebar**: Emergency Stop

### Step 4: Save the Map
To save the completed 2D occupancy grid at any time:
```bash
/home/akszayan/TorQ-R1/scripts/save_map.sh my_torq_r1_map
```
*Outputs: `maps/my_torq_r1_map.pgm` (high-res occupancy raster) and `maps/my_torq_r1_map.yaml` (metadata & resolution).*

### Step 5: Stop Mapping & Clean Shutdown
To save the map and cleanly stop all Gazebo, RViz, and ROS nodes:
```bash
/home/akszayan/TorQ-R1/scripts/stop_mapping.sh final_torq_r1_map
```

---

## 7. Fully Autonomous Exploration & Mapping (Zero-Human Exploration)

To autonomously map the entire maze without any manual teleoperation:

```bash
/home/akszayan/TorQ-R1/scripts/run_autonomous_mapping.sh
```

### How the Autonomous Exploration Engine Works:
1. **Yamauchi Frontier Detection:** Continuously detects boundaries between known free space and unknown space across the SLAM occupancy grid.
2. **Obstacle Inflation:** Dynamically dilates walls by $0.24\text{ m}$ to guarantee collision-free travel for TorQ-R1's chassis.
3. **BFS Shortest-Path Planning:** Calculates topological grid shortest-paths to all reachable frontier clusters.
4. **Pure Pursuit Path Execution:** Tracks waypoints towards the optimal frontier with differential in-place pivots at sharp turns.
5. **Full Room Coverage Guarantee:** Runs until **all reachable frontiers are exhausted** (0 frontiers remaining), ensuring every corridor and chamber is fully scanned.
6. **Automatic Map Output:** On completion, automatically writes:
   - `maps/torq_r1_autonomous_map.pgm`
   - `maps/torq_r1_autonomous_map.yaml`

