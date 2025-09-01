#!/usr/bin/env bash
set -euo pipefail

# 使用方法：
# ./run_python.sh [dataset] [config] [extra_args] [no_gui]
# 
# 範例：
# ./run_python.sh                                    # 使用預設設定
# ./run_python.sh realsense config/base.yaml         # 指定 dataset 和 config
# ./run_python.sh realsense config/base.yaml "" true # 無 GUI 模式

# 預設參數（可改）
DATASET_DEFAULT="realsense"
CONFIG_DEFAULT="config/base.yaml"

DATASET="${1:-$DATASET_DEFAULT}"
CONFIG="${2:-$CONFIG_DEFAULT}"
EXTRA_ARGS="${3:-}"    # 你想傳給 main.py 的額外參數（可留空）
NO_GUI="${4:-false}"   # 設定為 true 可以禁用 GUI

# 若要 GUI，先允許 X11（只需在本機跑一次）
if command -v xhost >/dev/null 2>&1; then
  echo ">>> 設定 X11 權限"
  xhost +local:root >/dev/null 2>&1 || true
  xhost +local:docker >/dev/null 2>&1 || true
fi

echo ">>> 執行 MASt3R-SLAM with RealSense"
echo ">>> Dataset: ${DATASET}"
echo ">>> Config: ${CONFIG}"
echo ">>> Extra args: ${EXTRA_ARGS}"

# 在已經運行的容器中執行 Python 腳本
docker compose exec mast3r bash -lc "
  set -euo pipefail
  
  echo '>>> 進到專案根目錄'
  cd /workspace/MASt3R-SLAM

  echo '>>> 檢查顯示環境'
  echo \"DISPLAY=\$DISPLAY\"
  echo \"X11 socket exists: \$(ls -la /tmp/.X11-unix/ 2>/dev/null || echo 'No X11 socket')\"
  
  echo '>>> 檢查 RealSense 設備'
  python3 -c '
import pyrealsense2 as rs
ctx = rs.context()
devices = ctx.query_devices()
print(f\"Found {len(devices)} RealSense device(s):\")
for i, dev in enumerate(devices):
    print(f\"  Device {i}: {dev.get_info(rs.camera_info.name)} - Serial: {dev.get_info(rs.camera_info.serial_number)}\")
if len(devices) == 0:
    print(\"WARNING: No RealSense devices detected!\")
    exit(1)
'

  echo '>>> 執行程式'
  if [ "$NO_GUI" = "true" ]; then
    PYTHONPATH=\"/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:\${PYTHONPATH:-}\" python main.py --dataset '${DATASET}' --config '${CONFIG}' ${EXTRA_ARGS} --no-viz
  else
    PYTHONPATH=\"/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:\${PYTHONPATH:-}\" python main.py --dataset '${DATASET}' --config '${CONFIG}' ${EXTRA_ARGS}
  fi
"
