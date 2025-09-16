#!/bin/bash
# Launch MASt3R-SLAM ROS2 Node with Visualization for Stretch3 Robot
set -e

echo "🚀 Starting MASt3R-SLAM with Visualization for Stretch3 Robot"
echo "===================================================="

# Check if we're in the container
if [ ! -d "/workspace/MASt3R-SLAM" ]; then
    echo "❌ Error: Must be run inside the mast3r-slam container"
    echo "Run: cd workspace/docker && ./run_mast3r.sh"
    exit 1
fi

# Change to MASt3R-SLAM directory
cd /workspace/MASt3R-SLAM

# Setup ROS2 environment  
echo "🔧 Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash

# Clean up any remaining processes and CUDA memory
echo "🧹 Cleaning up previous processes..."
pkill -f python3 2>/dev/null || true
pkill -f mast3r 2>/dev/null || true

# Clear CUDA cache if available
echo "🔧 Clearing CUDA cache..."
python3 -c "import torch; torch.cuda.empty_cache(); print('CUDA cache cleared')" 2>/dev/null || echo "CUDA cache clear failed (non-critical)"
export ROS_DOMAIN_ID=0

# Setup Python paths for MASt3R
export PYTHONPATH="/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:${PYTHONPATH:-}"

# Fix GUI resources (required for visualization)
echo "🖥️  Setting up visualization resources..."
mkdir -p /usr/local/lib/python3.10/dist-packages/resources || true

# Skip camera connectivity check for faster testing
echo "📷 Skipping camera connectivity check (for testing)"
echo "🔄 Node will wait for topics when started..."

# Check if the visualization node exists
if [ ! -f "mast3r_slam_visual_ros2.py" ]; then
    echo "❌ Error: mast3r_slam_visual_ros2.py not found"
    echo "Make sure the file is properly mounted in the container"
    exit 1
fi

echo "🎥 Starting MASt3R-SLAM with VISUALIZATION enabled..."
echo "📺 A visualization window should appear showing the SLAM process"
echo "🔴 Press Ctrl+C to stop and save final reconstruction"
echo ""

# Launch the visualization node with parameters
# Use enable_visualization parameter to control visualization (default: true for backward compatibility)
ENABLE_VIZ=${ENABLE_VIZ:-true}
echo "🔧 Visualization enabled: $ENABLE_VIZ"

python3 mast3r_slam_visual_ros2.py \
    --ros-args \
    -p config_file:=config/base.yaml \
    -p save_as:=stretch3_slam \
    -p image_topic:=/camera/camera/color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info \
    -p device:=cuda:0 \
    -p enable_visualization:=$ENABLE_VIZ

echo ""
echo "✅ MASt3R-SLAM with visualization completed"
echo "📁 Check logs/ directory for saved reconstructions"
