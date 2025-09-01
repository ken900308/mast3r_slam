# Enhanced MASt3R-SLAM with Periodic Saving

This repository contains an enhanced version of MASt3R-SLAM with automatic .ply file saving, graceful shutdown handling, and improved RealSense camera integration.

## 🚀 Quick Start

### 1. Build and Start Container
```bash
cd docker/
./run_mast3r.sh  # Builds container and installs all dependencies
```

### 2. Run Enhanced MASt3R-SLAM
```bash
./run_python_enhanced.sh           # With GUI visualization
./run_python_enhanced.sh --no-viz  # Headless mode
```

### 3. Access Container Directly
```bash
docker exec -it mast3r-slam bash
source /workspace/container_startup.sh
run_enhanced  # or check_camera, list_saves, etc.
```

## ✨ Enhanced Features

### **Automatic Periodic Saving**
- ✅ Saves .ply files **every 5 seconds** automatically
- ✅ **Graceful Ctrl+C handling** - saves before exit
- ✅ Multiple file types:
  - `realsense_live_partial_HHMMSS.ply` (periodic saves)
  - `realsense_live.ply` (final save on exit)

### **Improved Camera Integration**
- ✅ Fixed RealSense dataset path issues
- ✅ Automatic camera detection and validation
- ✅ Proper error handling for camera busy states

### **Enhanced Visualization**
- ✅ Fixed missing OpenGL shader files (`lines.glsl`)
- ✅ Stable 3D visualization at ~4-5 FPS
- ✅ Real-time keyframe and FPS monitoring

### **Docker Integration**
- ✅ All enhanced files automatically mounted via docker-compose
- ✅ No manual file copying required
- ✅ Persistent logs and datasets storage

## 📁 File Structure

```
docker/
├── docker-compose.yml          # Enhanced with auto-mounting
├── dockerfile                  # Base container setup
├── run_mast3r.sh              # Container setup script
├── run_python_enhanced.sh     # Enhanced SLAM with periodic saving
├── run_python.sh              # Basic SLAM execution
└── container_startup.sh       # In-container helper script

MASt3R-SLAM/
├── main.py                    # Enhanced with signal handling & periodic saving
├── mast3r_slam/
│   └── dataloader.py          # Fixed RealSense dataset_path issue
└── resources/programs/
    └── lines.glsl             # Fixed OpenGL shader for visualization
```

## 🎮 Usage Examples

### **From Host System:**
```bash
# Basic enhanced run with periodic saving
cd docker/
./run_python_enhanced.sh

# Headless mode (no GUI)
./run_python_enhanced.sh --no-viz

# Original functionality (if needed)
./run_python.sh
```

### **Inside Container:**
```bash
docker exec -it mast3r-slam bash
cd /workspace/MASt3R-SLAM

# Load helper functions
source /workspace/container_startup.sh

# Check camera
check_camera

# Run enhanced SLAM
run_enhanced

# List saved files
list_saves
```

## 💾 File Saving Behavior

### **Automatic Saves (Every 5 seconds)**
- **Location**: `logs/realsense_live_partial_HHMMSS.ply`
- **Trigger**: Automatic timer
- **Content**: Current 3D reconstruction state

### **Final Save (On Exit)**
- **Location**: `logs/realsense_live.ply`  
- **Trigger**: Natural exit or Ctrl+C
- **Content**: Complete final reconstruction

### **Save Locations**
```bash
# From host system
ls MASt3R-SLAM/logs/

# From container
ls /workspace/MASt3R-SLAM/logs/
```

## 🛠 Technical Improvements

### **Signal Handling**
```python
# Graceful Ctrl+C handling
def signal_handler(sig, frame):
    global should_exit
    print("\nReceived interrupt signal. Saving reconstruction and exiting...")
    should_exit = True
```

### **Periodic Saving**
```python
# Automatic saving every 5 seconds
save_interval = 5  # seconds
if i > 0 and len(keyframes) > 0:
    save_reconstruction_now(args, dataset, keyframes, last_msg, force=False)
```

### **Fixed Dataset Path**
```python
# RealsenseDataset fix
self.dataset_path = pathlib.Path("realsense_live")
```

## 🔧 Docker Compose Enhancements

The `docker-compose.yml` now automatically mounts all enhanced files:

```yaml
volumes:
  # Enhanced MASt3R-SLAM files - automatically sync changes
  - ../MASt3R-SLAM/main.py:/workspace/MASt3R-SLAM/main.py:rw
  - ../MASt3R-SLAM/mast3r_slam/dataloader.py:/workspace/MASt3R-SLAM/mast3r_slam/dataloader.py:rw
  - ../MASt3R-SLAM/resources/programs/lines.glsl:/workspace/MASt3R-SLAM/resources/programs/lines.glsl:rw
  
  # Runtime scripts for enhanced functionality
  - ./run_python.sh:/workspace/scripts/run_python.sh:ro
  - ./run_python_enhanced.sh:/workspace/scripts/run_python_enhanced.sh:ro
  - ./container_startup.sh:/workspace/container_startup.sh:ro
```

## 🎯 Performance

- **Camera FPS**: ~4-5 FPS with RealSense D435
- **GPU Utilization**: ~94% during operation
- **Save Frequency**: Every 5 seconds (configurable)
- **Memory Usage**: Stable with GUI visualization

## 🐛 Troubleshooting

### **Camera Issues**
```bash
# Check camera status
docker exec mast3r-slam bash -c "source /workspace/container_startup.sh && check_camera"

# If camera busy, restart container
docker compose restart
```

### **No .ply Files Generated**
- Ensure `save_results = True` in dataset configuration
- Check that periodic saving is enabled (save_interval = 5)
- Verify logs directory permissions

### **GUI Issues**
```bash
# Ensure X11 forwarding is enabled
xhost +local:docker

# Check display variable
echo $DISPLAY
```

## 🎉 Success Indicators

When running properly, you should see:
```
RealSense camera detected. Starting MASt3R-SLAM...
Starting MASt3R-SLAM with periodic saving...
Press Ctrl+C to save and exit gracefully
...
Saved reconstruction to logs/realsense_live_partial_105827.ply
FPS: 4.0869601284580535, Keyframes: 3
...
```

## 📝 Notes

- All changes are automatically synced between host and container
- No manual file copying required after setup
- Ctrl+C now saves before exiting (graceful shutdown)
- .ply files are generated continuously during operation
- Container persists between restarts for easy development
