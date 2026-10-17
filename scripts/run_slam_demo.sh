#!/usr/bin/env bash
# ==============================================================================
# Master Runner: TorQ-R1 Maze SLAM & Visual Mapping Demo
# - Cleans up any prior zombie simulator instances to prevent desync
# - Launches Gazebo Sim 8 with 16mx16m Maze World
# - Spawns TorQ-R1 4WD Differential-Drive Robot
# - Starts ROS 2 - Gazebo Parameter Bridge
# - Starts SLAM Toolbox (Real-time Asynchronous 2D Mapping)
# - Starts Path Publisher Node (smooth trajectory line, zero arrow clutter)
# - Opens RViz2 with high-visibility Map, Scan, and Trajectory
# - Spawns interactive Arrow-Key Teleop Window
# ==============================================================================

set -e

# Pre-launch: Clean up any stale background simulator or ROS processes to prevent desync
echo "Cleaning up any stale simulation processes..."
pkill -9 -f "gz sim|parameter_bridge|slam_toolbox|rviz2|arrow_teleop|path_publisher" 2>/dev/null || true
sleep 1

# Setup exit trap to guarantee all background processes terminate on exit
trap 'echo "Terminating simulation stack..."; pkill -9 -f "gz sim|parameter_bridge|slam_toolbox|rviz2|arrow_teleop|path_publisher" 2>/dev/null || true' EXIT SIGINT SIGTERM

WS_DIR="/home/akszayan/TorQ-R1"
source "${WS_DIR}/scripts/setup_workspace.sh"

echo "==================================================================="
echo "  🚀 STARTING TORQ-R1 MAZE SLAM SIMULATION DEMO"
echo "==================================================================="
echo "  1. Gazebo Sim 8 (Harmonic) GUI with Maze World"
echo "  2. TorQ-R1 Differential Drive Robot (Single Active Instance)"
echo "  3. Real-Time SLAM Toolbox (Industrial-Grade Ceres Scan Matching)"
echo "  4. RViz2 Visual Mapping Display (Map, LaserScan, Path Line)"
echo "  5. Interactive Arrow-Key Teleoperation Window"
echo "==================================================================="

# Launch interactive arrow teleop in a separate gnome-terminal window if available
if command -v gnome-terminal &> /dev/null && [ -n "$DISPLAY" ]; then
    echo "Spawning Arrow Teleoperation window in dedicated terminal..."
    gnome-terminal --title="🎮 TorQ-R1 Arrow Teleop (Drive to Map)" -- bash -c "${WS_DIR}/scripts/launch_teleop.sh; exec bash" &
else
    echo "-------------------------------------------------------------------"
    echo "  NOTE: In a separate terminal window, drive the robot with:"
    echo "    ${WS_DIR}/scripts/launch_teleop.sh"
    echo "-------------------------------------------------------------------"
fi

# Launch the master simulation, SLAM, and RViz stack
ros2 launch torq_r1_gazebo torq_r1_slam.launch.py world:=maze_world gui:=true rviz:=true
