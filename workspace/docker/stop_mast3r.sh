#!/usr/bin/env bash
set -euo pipefail

echo ">>> 停止 MASt3R-SLAM 容器"
docker compose down

echo ">>> 容器已停止"
