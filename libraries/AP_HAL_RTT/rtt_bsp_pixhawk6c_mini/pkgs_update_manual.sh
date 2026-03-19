#!/bin/bash
# 当 RT-Thread Env 的 pkgs 无法使用时，用此脚本手动拉取 H7 BSP 依赖包
# 使用：在 BSP 目录下执行 ./pkgs_update_manual.sh

set -e
BSP_ROOT="$(cd "$(dirname "$0")" && pwd)"
PKGS_DIR="$BSP_ROOT/packages"
mkdir -p "$PKGS_DIR"

PACKAGES=(
    "CMSIS-Core-latest|https://github.com/RT-Thread-packages/CMSIS-Core.git"
    "stm32h7_cmsis_driver-latest|https://github.com/RT-Thread-packages/cmsis-device-h7.git"
    "stm32h7_hal_driver-latest|https://github.com/RT-Thread-packages/stm32h7xx-hal-driver.git"
)

for entry in "${PACKAGES[@]}"; do
    name="${entry%%|*}"
    url="${entry#*|}"
    dest="$PKGS_DIR/$name"
    if [ -d "$dest" ]; then
        echo "[OK] 已存在: $name"
    else
        echo "[拉取] $name ..."
        git clone --depth 1 "$url" "$dest"
        echo "[OK] $name"
    fi
done

echo ""
echo "依赖包已就绪，可执行: scons"
