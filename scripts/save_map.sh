#!/usr/bin/env bash
# ==============================================================================
# Helper Script: Save Completed TorQ-R1 SLAM Map
# Usage: ./scripts/save_map.sh [map_name]
# ==============================================================================
set -e

WS_DIR="/home/akszayan/TorQ-R1"
source "${WS_DIR}/scripts/setup_workspace.sh"

MAP_NAME="${1:-torq_r1_map}"
MAP_DIR="${WS_DIR}/maps"
mkdir -p "${MAP_DIR}"

echo "==================================================================="
echo "  💾 SAVING COMPLETED SLAM MAP TO: ${MAP_DIR}/${MAP_NAME}"
echo "==================================================================="

ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" --ros-args -p use_sim_time:=true

echo "==================================================================="
echo "  ✅ Map successfully saved!"
echo "  Image : ${MAP_DIR}/${MAP_NAME}.pgm"
echo "  YAML  : ${MAP_DIR}/${MAP_NAME}.yaml"
echo "==================================================================="
