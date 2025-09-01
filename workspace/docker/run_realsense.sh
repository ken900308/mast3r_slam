#!/usr/bin/env bash
set -euo pipefail

echo "🚀 Starting MASt3R-SLAM with RealSense camera..."

# Make sure container is running
echo "📦 Ensuring container is running..."
docker compose up -d

# Check if RealSense is connected
echo "📹 Checking RealSense camera..."
CAMERA_CHECK=$(docker exec mast3r-slam bash -c "
cd /workspace/MASt3R-SLAM
python3 -c 'import pyrealsense2 as rs; ctx = rs.context(); print(len(ctx.query_devices()))'
")

if [ "$CAMERA_CHECK" = "0" ]; then
    echo "❌ No RealSense camera detected! Please connect your camera."
    exit 1
fi

echo "✅ RealSense camera detected!"

# Run MASt3R-SLAM
echo "🎯 Starting MASt3R-SLAM..."
docker exec -it mast3r-slam bash -c "
cd /workspace/MASt3R-SLAM
export PYTHONPATH=/workspace/MASt3R-SLAM:/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM/thirdparty/in3d:\$PYTHONPATH
python main.py --dataset realsense --config config/base.yaml \$@
" -- "$@"
