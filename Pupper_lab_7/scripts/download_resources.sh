#!/bin/bash
# Download required model resources

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Project root is one level up from scripts directory
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Change to project directory and download
cd "$PROJECT_ROOT"

echo "Downloading YOLOv5 model for Hailo..."
wget https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.11.0/hailo8l/yolov5m_wo_spp_60p.hef

echo "✅ Download complete!"
echo "Model saved to: $PROJECT_ROOT/yolov5m_wo_spp_60p.hef"


