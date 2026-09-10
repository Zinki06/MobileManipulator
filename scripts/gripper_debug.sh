#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
run_dir="$PWD/gripper_debug/manual_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run_dir"
export ROS_LOG_DIR="$run_dir/ros"
python3 -u scripts/gripper_debug.py "$run_dir" 2>&1 | tee "$run_dir/console.log"
