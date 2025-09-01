#!/usr/bin/env bash
set -euo pipefail

# 使用方法：
# ./run_python_save.sh [dataset] [config] [extra_args] [no_gui]
# 
# 範例：
# ./run_python_save.sh                                    # 使用預設設定，強制儲存 .ply
# ./run_python_save.sh realsense config/base.yaml         # 指定 dataset 和 config，強制儲存 .ply
# ./run_python_save.sh realsense config/base.yaml "" true # 無 GUI 模式，強制儲存 .ply

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
  xhost +local:docker >/dev/null 2>&1 || true
fi

echo ">>> 執行 MASt3R-SLAM with RealSense (強制儲存 .ply)"
echo ">>> Dataset: ${DATASET}"
echo ">>> Config: ${CONFIG}"
echo ">>> Extra args: ${EXTRA_ARGS}"

docker compose exec mast3r_slam bash -c "
  echo '>>> 進到專案根目錄'
  cd /workspace/MASt3R-SLAM

  echo '>>> 檢查顯示環境'
  echo \"DISPLAY=\${DISPLAY}\"
  ls -la /tmp/.X11-unix/ | head -5

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

  echo '>>> 修改 RealSense 資料集以啟用儲存'
  # 臨時修改 dataloader.py 以啟用 save_results
  sed -i 's/self.save_results = False/self.save_results = True/g' /workspace/MASt3R-SLAM/mast3r_slam/dataloader.py
  
  echo '>>> 執行程式'
  if [ \"$NO_GUI\" = \"true\" ]; then
    PYTHONPATH=\"/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:\${PYTHONPATH:-}\" python main.py --dataset '${DATASET}' --config '${CONFIG}' ${EXTRA_ARGS} --no-viz
  else
    PYTHONPATH=\"/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:\${PYTHONPATH:-}\" python main.py --dataset '${DATASET}' --config '${CONFIG}' ${EXTRA_ARGS}
  fi
"
