#!/usr/bin/env bash
# ==============================================================================
# Master Runner: TorQ-R1 Autonomous SLAM & Maze Exploration Mapping
# - Cleans up any stale simulator instances to prevent desync
# - Launches Gazebo Sim 8 with 16mx16m Maze World
# - Spawns TorQ-R1 Differential-Drive Robot
# - Starts ROS 2 - Gazebo Parameter Bridge
# - Starts SLAM Toolbox (Real-time Asynchronous 2D Mapping)
# - Starts Path Publisher Node (smooth trajectory line, zero arrow clutter)
# - Starts Autonomous Explorer Node (Yamauchi Frontier-Based Exploration)
# - Opens RViz2 with high-visibility Map, Scan, and Trajectory
# - Automatically saves completed map to maps/torq_r1_autonomous_map (.pgm + .yaml)
# ==============================================================================

set -e

# Pre-launch: Clean up any stale background simulator or ROS processes to prevent desync
echo "Cleaning up any stale simulation processes..."
pkill -9 -f "gz sim|parameter_bridge|slam_toolbox|rviz2|arrow_teleop|path_publisher|autonomous_explorer" 2>/dev/null || true
sleep 1

# Setup exit trap to guarantee all background processes terminate on exit
trap 'echo "Terminating simulation stack..."; pkill -9 -f "gz sim|parameter_bridge|slam_toolbox|rviz2|arrow_teleop|path_publisher|autonomous_explorer" 2>/dev/null || true' EXIT SIGINT SIGTERM

WS_DIR="/home/akszayan/TorQ-R1"
source "${WS_DIR}/scripts/setup_workspace.sh"

echo "==================================================================="
echo "  🚀 STARTING TORQ-R1 AUTONOMOUS MAZE SLAM & MAPPING"
echo "==================================================================="
echo "  1. Gazebo Sim 8 (Harmonic) GUI with Maze World"
echo "  2. TorQ-R1 Differential Drive Robot"
echo "  3. Real-Time SLAM Toolbox (Industrial-Grade Ceres Scan Matching)"
echo "  4. RViz2 Visual Mapping Display (Map, LaserScan, Path & Frontiers)"
echo "  5. Autonomous Explorer Node (Yamauchi Frontier Exploration)"
echo "  6. Mission Goal: Full Maze Coverage (Runs until 0 frontiers remain)"
echo "  7. Output Map: ${WS_DIR}/maps/torq_r1_autonomous_map"
echo "==================================================================="

# Launch master autonomous simulation, SLAM, and RViz stack
ros2 launch torq_r1_gazebo torq_r1_autonomous_slam.launch.py \
    world:=maze_world \
    gui:=true \
    rviz:=true

