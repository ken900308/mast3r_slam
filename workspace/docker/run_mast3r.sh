#!/usr/bin/env bash
set -euo pipefail

# 預設參數（可改）
DATASET_DEFAULT="datasets/tum/rgbd_dataset_freiburg1_desk"
CONFIG_DEFAULT="config/base.yaml"

DATASET="${1:-$DATASET_DEFAULT}"
CONFIG="${2:-$CONFIG_DEFAULT}"
EXTRA_ARGS="${3:-}"    # 你想傳給 main.py 的額外參數（可留空）

# 若要 GUI，先允許 X11（只需在本機跑一次）
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

# 用 docker compose 啟動工作容器，在容器內做安裝＋執行（不自動刪除）
docker compose run mast3r bash -lc "
  set -euo pipefail

  echo '>>> Fixing GUI Resources for MASt3R-SLAM'
  echo 'Creating resources directory...'
  mkdir -p /usr/local/lib/python3.10/dist-packages/resources
  
  echo 'Copying in3d resources...'
  if [ -d '/workspace/MASt3R-SLAM/thirdparty/in3d/resources' ]; then
    cp -r /workspace/MASt3R-SLAM/thirdparty/in3d/resources/* /usr/local/lib/python3.10/dist-packages/resources/ 2>/dev/null || true
    echo '  ✓ in3d resources copied'
  fi
  
  echo 'Copying MASt3R-SLAM resources...'
  if [ -d '/workspace/MASt3R-SLAM/resources' ]; then
    cp -r /workspace/MASt3R-SLAM/resources/* /usr/local/lib/python3.10/dist-packages/resources/ 2>/dev/null || true
    echo '  ✓ MASt3R-SLAM resources copied'
  fi
  
  echo '✓ GUI Resources setup complete'

  echo '>>> Python/環境檢查'
  python -V
  nvcc --version || true
  python -c 'import torch, ctypes.util; print(\"torch:\", torch.__version__); print(\"torch.cuda:\", torch.version.cuda); print(\"is_available:\", torch.cuda.is_available()); print(\"cudart:\", ctypes.util.find_library(\"cudart\"))'

  echo '>>> 進到專案根目錄'
  cd /workspace/MASt3R-SLAM

  echo '>>> 升級打包工具鏈 + 編譯器（避免 packaging 相容性問題，ninja 加速）'
  python -m pip install -U pip 'setuptools>=70' 'packaging>=24.1' 'wheel>=0.43' ninja

  echo '>>> 預先安裝 imgui 的 wheel（避免從 sdist 編譯失敗）'
  python -m pip install --only-binary=:all: "imgui==2.0.0"

  echo '>>> 安裝 Cython 和 OpenGL 依賴（為了 custom pyimgui 編譯）'
  python -m pip install 'Cython>=0.24,<0.30' PyOpenGL

  echo '>>> 安裝 thirdparty 套件（非 editable，使用 no-build-isolation）'
  python -m pip install --no-build-isolation thirdparty/mast3r
  python -m pip install --no-build-isolation thirdparty/in3d

  echo '>>> 安裝專案本體（非 editable，使用 no-build-isolation）'
  python -m pip install --no-build-isolation . --no-cache-dir

  echo '>>> 最終檢查 torch/cuda 可用性（執行期應該是 True）'
  python - <<'PY'
import torch, sys
print('torch:', torch.__version__)
print('torch.version.cuda:', torch.version.cuda)
print('is_available:', torch.cuda.is_available())
if not torch.cuda.is_available():
    print('WARNING: torch.cuda.is_available() == False; 請確認你以 --gpus all 啟動容器，且主機有安裝驅動與 nvidia-container-toolkit')
PY

  echo '>>> 安裝完成！容器已準備就緒'
  echo '>>> 要執行程式，請使用: ./run_python.sh'
  echo '>>> 容器將保持運行狀態...'
  tail -f /dev/null
"