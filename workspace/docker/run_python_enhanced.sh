#!/bin/bash

# Enhanced MASt3R-SLAM with periodic saving
# This script runs MASt3R-SLAM with automatic .ply file saving every 30 seconds
# and graceful shutdown handling (Ctrl+C will save before exiting)

# Display help information
echo "=== MASt3R-SLAM with Periodic Saving ==="
echo "Features:"
echo "  - Saves .ply files every 30 seconds automatically"
echo "  - Graceful shutdown: Press Ctrl+C to save and exit safely"
echo "  - Files saved to: logs/realsense_live/"
echo "  - Partial saves: realsense_live_partial_HHMMSS.ply"
echo "  - Final save: realsense_live.ply"
echo ""
echo "Controls:"
echo "  - Ctrl+C: Save current reconstruction and exit gracefully"
echo "  - Close GUI window: Natural exit with final save"
echo ""

# Check if container is running (try both naming schemes)
CONTAINER_NAME=$(docker ps --format "{{.Names}}" | grep -E "(docker-mast3r-run|mast3r-slam)" | head -1)
if [ -z "$CONTAINER_NAME" ]; then
    echo "Error: MASt3R container is not running!"
    echo "Please run './run_mast3r.sh' or 'docker compose up -d' first to start the container."
    exit 1
fi
echo "Using container: $CONTAINER_NAME"

# Automatically fix GUI resources if running with visualization
if [[ "$@" != *"--no-viz"* ]]; then
    echo "Setting up GUI resources..."
    docker exec "$CONTAINER_NAME" bash -c "
        if [ ! -d '/usr/local/lib/python3.10/dist-packages/resources' ] || [ -z \"\$(ls -A /usr/local/lib/python3.10/dist-packages/resources 2>/dev/null)\" ]; then
            echo 'GUI resources missing, fixing automatically...'
            mkdir -p /usr/local/lib/python3.10/dist-packages/resources
            
            if [ -d '/workspace/MASt3R-SLAM/thirdparty/in3d/resources' ]; then
                cp -r /workspace/MASt3R-SLAM/thirdparty/in3d/resources/* /usr/local/lib/python3.10/dist-packages/resources/ 2>/dev/null || true
                echo '  ✓ in3d resources copied'
            fi
            
            if [ -d '/workspace/MASt3R-SLAM/resources' ]; then
                cp -r /workspace/MASt3R-SLAM/resources/* /usr/local/lib/python3.10/dist-packages/resources/ 2>/dev/null || true
                echo '  ✓ MASt3R-SLAM resources copied'
            fi
            
            echo '✓ GUI resources setup complete'
        else
            echo 'GUI resources already configured ✓'
        fi
    "
fi

# Check for RealSense camera only if using realsense dataset
if [[ "$@" == *"realsense"* ]]; then
    echo "Checking for RealSense camera..."
    CAMERA_CHECK=$(docker exec "$CONTAINER_NAME" bash -c 'cd /workspace/MASt3R-SLAM && python3 -c "import pyrealsense2 as rs; ctx = rs.context(); devices = ctx.query_devices(); print(len(devices))"' 2>/dev/null || echo "0")

    echo "Debug: CAMERA_CHECK='$CAMERA_CHECK'"
    if [ "$CAMERA_CHECK" = "0" ]; then
        echo "Warning: No RealSense camera detected!"
        echo "Make sure your RealSense camera is connected and try again."
        exit 1
    fi
    echo "RealSense camera detected (found $CAMERA_CHECK devices)."
else
    echo "Using pre-recorded dataset (not requiring RealSense camera)."
fi

echo "Starting MASt3R-SLAM..."

# Set up X11 forwarding for GUI
export DISPLAY=:0
if command -v xhost >/dev/null 2>&1; then
    echo "Setting up X11 permissions..."
    xhost +local:docker >/dev/null 2>&1 || true
fi

# Determine mode based on arguments
if [ "$1" = "--no-viz" ]; then
    echo "Running in headless mode (no visualization)"
    VIZ_FLAG="--no-viz"
else
    echo "Running with 3D visualization GUI"
    VIZ_FLAG=""
fi

# Set up signal handling to properly forward SIGINT to container
cleanup() {
    echo ""
    echo "Signal received! Terminating MASt3R-SLAM processes..."
    docker exec "$CONTAINER_NAME" pkill -f "python main_fixed.py" 2>/dev/null || true
    docker exec "$CONTAINER_NAME" pkill -f "multiprocessing" 2>/dev/null || true
    echo "Cleanup completed."
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM
trap cleanup SIGINT SIGTERM

# Run MASt3R-SLAM with enhanced saving
docker exec "$CONTAINER_NAME" bash -c "
cd /workspace/MASt3R-SLAM
export PYTHONPATH=\"/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:\${PYTHONPATH:-}\"
echo 'Starting MASt3R-SLAM with periodic saving...'
echo 'Press Ctrl+C to save and exit gracefully'
python main.py --dataset realsense --config config/base.yaml $VIZ_FLAG
" &

# Store the background process PID
DOCKER_PID=$!

# Wait for the docker exec process to complete
wait $DOCKER_PID

echo ""
echo "=== Session Complete ==="
echo "Check logs/realsense_live/ for saved .ply files:"
echo "  - Partial saves: realsense_live_partial_*.ply (saved every 30s)"
echo "  - Final save: realsense_live.ply (saved on exit)"
