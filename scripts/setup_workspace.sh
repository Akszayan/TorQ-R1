#!/usr/bin/env bash
# ==============================================================================
# TorQ-R1 Simulation & Navigation Environment Setup
# Workspace: ~/TorQ-R1
# ROS 2 Jazzy & Gazebo Sim 8 (Harmonic)
# ==============================================================================

WS_DIR="/home/akszayan/TorQ-R1"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

# Export Gazebo Sim resource path for 3D meshes & models
DESC_SHARE="${WS_DIR}/install/torq_r1_description/share"
GZ_SHARE="${WS_DIR}/install/torq_r1_gazebo/share"
export GZ_SIM_RESOURCE_PATH="${DESC_SHARE}:${GZ_SHARE}:${WS_DIR}/install:${GZ_SIM_RESOURCE_PATH}"

echo "==================================================================="
echo "  🚀 TorQ-R1 SLAM & Simulation Environment Ready! [ROS 2 Jazzy]"
echo "==================================================================="
echo "  Available Quick Launch Commands:"
echo "    ${WS_DIR}/scripts/run_slam_demo.sh"
echo "    ${WS_DIR}/scripts/launch_teleop.sh"
echo "    ${WS_DIR}/scripts/save_map.sh <map_name>"
echo "    ${WS_DIR}/scripts/stop_mapping.sh <map_name>"
echo ""
echo "  Direct ROS 2 Launch Commands:"
echo "    ros2 launch torq_r1_gazebo torq_r1_slam.launch.py world:=maze_world gui:=true rviz:=true"
echo "    ros2 launch torq_r1_gazebo torq_r1_sim.launch.py world:=obstacle_world gui:=true rviz:=true"
echo "    ros2 run torq_r1_navigation arrow_teleop_node"
echo "    ros2 run torq_r1_navigation obstacle_avoidance_node"
echo "==================================================================="
