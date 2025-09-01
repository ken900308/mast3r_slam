# MASt3R-SLAM Docker Setup

## Quick Start

### 1. Setup Container (One-time or when you need fresh environment)
```bash
./run_mast3r.sh
```
This will:
- Install all dependencies (MAST3R, lietorch, in3d, etc.)
- Setup the environment 
- Keep the container running in background

### 2. Run MASt3R-SLAM with RealSense
```bash
./run_python.sh
```
This will:
- Check RealSense device connection
- Run `python main.py --dataset realsense --config config/base.yaml`

### 3. Stop Container (when done)
```bash
./stop_mast3r.sh
```

### Custom Usage
```bash
# Run with different config
./run_python.sh realsense config/eth3d.yaml

# Run with extra arguments  
./run_python.sh realsense config/base.yaml "--some-extra-arg"
```

## Development Workflow

1. **First time setup:**
   ```bash
   ./run_mast3r.sh  # Install everything, container stays running
   ```

2. **During development:**
   ```bash
   ./run_python.sh  # Run the script as many times as needed
   ```

3. **When finished:**
   ```bash
   ./stop_mast3r.sh  # Stop container to free resources
   ```

## Scripts Overview

- `run_mast3r.sh` - Sets up container with all dependencies, keeps running
- `run_python.sh` - Runs the main Python script in the existing container  
- `stop_mast3r.sh` - Stops the container
- `run_realsense.sh` - Legacy script for quick RealSense testing

## Requirements

- Docker with GPU support (nvidia-docker2)
- Intel RealSense camera (D435 or similar)
- X11 forwarding for GUI (xhost +local:root)

## GitHub Distribution

For sharing this setup on GitHub, users can:

1. Clone the repository
2. Run `./run_mast3r.sh` once to install everything
3. Run `./run_python.sh` to execute the SLAM system
4. Run `./stop_mast3r.sh` when done
