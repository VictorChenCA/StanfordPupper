#!/bin/bash
# Simple script to run basic foxglove setup

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Project root is one level up from scripts directory
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Change to project directory
cd "$PROJECT_ROOT"

source ~/.bashrc
ros2 launch "$PROJECT_ROOT/lab_7.launch.py" &
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
python "$PROJECT_ROOT/hailo_detection.py"


