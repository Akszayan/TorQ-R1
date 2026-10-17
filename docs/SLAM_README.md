# TorQ-R1 SLAM System Specification & Architecture

This document provides a comprehensive technical breakdown of the Simultaneous Localization and Mapping (SLAM) pipeline implemented for the **TorQ-R1** autonomous mobile robot.

---

## 1. System Overview & Method Name

- **Method Name:** **Asynchronous 2D Pose Graph SLAM with Ceres Least Squares Optimization (Graph-SLAM)**
- **ROS 2 Implementation:** `slam_toolbox` (Online Asynchronous Mapping Node)
- **Actuation Model:** Coaxial Differential Drive with Decoupled Contact Damping
- **Odometry Integration:** Ground-Truth 50 Hz Rigid-Body Physics Odometry (`gz-sim-odometry-publisher-system`)
- **LiDAR Sensor:** 2D Planar GPU LiDAR ($360^\circ$ FOV, 720 samples, 20 Hz, range $0.16\text{ m} - 12.0\text{ m}$)

---

## 2. Core SLAM Algorithms & Mathematical Formulations

The SLAM pipeline utilizes a dual-tier scan registration and global factor graph optimization architecture:

### A. Ceres Non-Linear Least Squares Scan Matcher (`CeresScanMatcher`)
The local scan registration matches incoming planar laser scans directly against the accumulated occupancy submap by minimizing the non-linear least-squares cost:

$$\min_{(x, y, \theta)} \sum_{i=1}^{N} \left[ 1 - M\left( T(x, y, \theta) \cdot p_i \right) \right]^2 + w_{\text{dist}} \|\Delta d\|^2 + w_{\text{ang}} \|\Delta \theta\|^2$$

Where:
- $p_i = [x_i, y_i]^T$ is the $i$-th endpoint from the laser scan.
- $T(x, y, \theta)$ is the rigid 2D Euclidean transformation matrix in $\text{SE}(2)$:
  $$T(x, y, \theta) = \begin{bmatrix} \cos\theta & -\sin\theta & x \\ \sin\theta & \cos\theta & y \\ 0 & 0 & 1 \end{bmatrix}$$
- $M(q) \in [0, 1]$ represents the occupancy probability of world coordinate $q$ evaluated using bilinear interpolation on the map.
- $w_{\text{dist}}$ and $w_{\text{ang}}$ are variance penalty weights constraining the optimization deviation from the odometry motion prior ($w_{\text{dist}} = 0.5$, $w_{\text{ang}} = 0.5$).

### B. Numerical Optimization Configuration (Google Ceres Solver)
- **Linear Solver:** `SPARSE_NORMAL_CHOLESKY` (sparse Cholesky decomposition of normal equations $(J^T J)$).
- **Preconditioner:** `SCHUR_JACOBI` (block-diagonal Schur complement preconditioning).
- **Trust-Region Strategy:** `LEVENBERG_MARQUARDT` with dynamic damping parameter $\lambda$.
- **Step Controller:** `TRADITIONAL_DOGLEG` interpolation between Gauss-Newton and steepest-descent steps.
- **Thread Count:** Bounded to hardware core availability to prevent OpenMP over-subscription.

### C. Multi-Resolution Correlative Scan Matcher
Before invoking the continuous Ceres optimizer, a multi-resolution correlative grid search evaluates scan likelihood to seed the non-linear solver with an optimal initial basin of attraction:
- **Search Space Dimensions:** $0.5\text{ m} \times 0.5\text{ m}$ window.
- **Spatial Resolution:** $0.01\text{ m}$ (fine resolution).
- **Coarse Angular Resolution:** $0.01745\text{ rad}$ ($1.0^\circ$).
- **Fine Angular Offset:** $0.00349\text{ rad}$ ($0.2^\circ$).

### D. Sparse Pose Graph Optimization (PGO)
- Trajectory poses $x_k \in \text{SE}(2)$ form factor graph nodes.
- Sequential odometry and scan-match constraints form relative transformation edges:
  $$e_{ij} = T_j^{-1} T_i \Delta T_{ij}^{-1}$$
- When loop closures are identified, global graph relaxation updates the entire pose history, completely eliminating cumulative trajectory drift across the $16\text{ m} \times 16\text{ m}$ maze.

### E. Loop Closure Detection with $\chi^2$ Gating
- Candidate loop closures are detected within a spatial radius of $R_{\text{loop}} = 7.0\text{ m}$.
- Requires a minimum trajectory chain length of 10 nodes before attempting closure.
- **Outlier Rejection:** Applies Mahalanobis distance gating with a Chi-Square ($\chi^2$) threshold:
  $$D_{\text{Mahalanobis}}^2 = e_{ij}^T \Sigma_{ij}^{-1} e_{ij} \le \chi_{3, 0.95}^2$$
  Matches failing the hypothesis test are rejected as false loop closures, preventing rotational ghosting or duplicated walls.

---

## 3. Filters & Noise Mitigation Stack

| Filter Name | Type / Implementation | Location | Functional Purpose |
| :--- | :--- | :--- | :--- |
| **Bresenham Ray-Trace Free-Space Carving Filter** | Dynamic Ray Casting (`max_laser_range: 12.0m`) | `slam_toolbox` | Traverses line-of-sight cells along each laser beam and decrements occupancy values, dynamically scrubbing phantom obstacles. |
| **Near-Field Blindzone Geometric Filter** | Physical Ray Clipping (`min_ray_range: 0.16m`, `min_laser_range: 0.18m`) | [`sensors.xacro`](file:///home/akszayan/TorQ-R1/src/torq_r1_description/urdf/sensors.xacro) & `slam_toolbox_params.yaml` | Strips LiDAR returns originating within the robot's perimeter, eliminating self-reflections off the bumper and wheels. |
| **Contact Friction & Surface Decoupling Filter** | Gazebo ODE/Bullet Coulomb Differential | [`ack_diff.xacro`](file:///home/akszayan/TorQ-R1/src/torq_r1_description/urdf/ack_diff.xacro) | Decouples the drive axle ($\mu=2.5$) from passive corner gliders ($\mu=0.0$), eliminating contact bounce and lateral drift. |
| **Odometry Ingestion & Rigid-Body Filter** | 50 Hz Direct SE(2) Rigid-Body Integration | `gz-sim-odometry-publisher-system` | Ingests physics-verified robot displacement, preventing wheel-slip encoder errors from perturbing Ceres scan priors. |
| **Scan Decimation & Temporal Throttling Filter** | Rate Decimation (`throttle_scans: 1`, $25\text{ Hz}$ limit; $4\text{ Hz}$ map publisher) | `slam_toolbox_params.yaml` | Throttles message processing to match solver execution time, preventing queue buffer backlog and RViz rendering lag. |

---

## 4. Visualization & RViz2 Setup

The RViz2 display configuration ([`torq_r1_slam.rviz`](file:///home/akszayan/TorQ-R1/src/torq_r1_gazebo/rviz/torq_r1_slam.rviz)) is calibrated for industrial clarity:

| Display Name | RViz Plugin Type | Subscribed Topic | Visual Presentation & Settings |
| :--- | :--- | :--- | :--- |
| **Grid** | `rviz_default_plugins/Grid` | `map` (Reference) | Plane: XY, Cell Size: $1.0\text{ m}$, Subdivisions: 1 |
| **Map** | `rviz_default_plugins/Map` | `/map`, `/map_updates` | Scheme: `map`, Transient Local durability, $0.05\text{ m}$ resolution |
| **LaserScan** | `rviz_default_plugins/LaserScan` | `/scan` | Style: Flat Squares, Size: $0.04\text{ m}$, Color: **Vivid Neon Green (`#00FF78`)** |
| **Path Trajectory** | `rviz_default_plugins/Path` | `/robot_path` | Style: Lines, Line Width: $0.04\text{ m}$, Color: **Cyan (`#00DCFF`)** |
| **Frontier Path** | `rviz_default_plugins/Path` | `/exploration_path` | Style: Lines, Line Width: $0.035\text{ m}$, Color: **Gold/Yellow (`#FFC800`)** |
| **RobotModel** | `rviz_default_plugins/RobotModel` | `/robot_description` | Visual: Enabled, Collision: Disabled (no red bounding boxes) |
| **TF Frames** | `rviz_default_plugins/TF` | `/tf`, `/tf_static` | **Disabled** (clears messy yellow/red arrow clutter) |

---

## 5. Verified Output Map

A complete mapping mission across the $16\text{ m} \times 16\text{ m}$ maze generated razor-sharp, orthogonal boundaries with full loop-closure alignment:

- **PGM Raster:** `maps/torq_r1_autonomous_map.pgm` (Resolution: $0.05\text{ m/pixel}$, Dimensions: $355 \times 328$)
- **Metadata YAML:** `maps/torq_r1_autonomous_map.yaml`
- **Preview PNG:** `maps/torq_r1_autonomous_map.png`

---

## 6. How to Run SLAM Manually

```bash
# 1. Source workspace
source /opt/ros/jazzy/setup.bash
source /home/akszayan/TorQ-R1/install/setup.bash

# 2. Launch Gazebo Simulation, SLAM Toolbox & RViz2
ros2 launch torq_r1_gazebo torq_r1_slam.launch.py world:=maze_world gui:=true rviz:=true

# 3. Teleoperate the robot (in separate terminal)
ros2 run torq_r1_navigation arrow_teleop_node

# 4. Save map on completion
ros2 run nav2_map_server map_saver_cli -f /home/akszayan/TorQ-R1/maps/my_map --ros-args -p use_sim_time:=true
```
