#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXPECTED_DIR="${HOME}/delta_4dof"

clear
echo "========================================"
echo "       DELTA 4DOF AUTO LAUNCHER"
echo "========================================"
echo "Current folder: ${ROOT_DIR}"
echo

# Launch file hien tai dang mac dinh workspace o ~/delta_4dof.
# Neu nguoi dung tai folder nay o noi khac, tu tao symlink ve ~/delta_4dof.
if [ "${ROOT_DIR}" != "${EXPECTED_DIR}" ]; then
  if [ ! -e "${EXPECTED_DIR}" ]; then
    echo "[INFO] Tao lien ket ${EXPECTED_DIR} -> ${ROOT_DIR}"
    ln -s "${ROOT_DIR}" "${EXPECTED_DIR}"
    ROOT_DIR="${EXPECTED_DIR}"
  elif [ "$(readlink -f "${EXPECTED_DIR}")" != "$(readlink -f "${ROOT_DIR}")" ]; then
    echo "[ERROR] Thu muc ${EXPECTED_DIR} da ton tai va khong phai repo hien tai."
    echo "Hay clone repo vao dung ~/delta_4dof hoac xoa/sua thu muc cu."
    echo
    read -rp "Nhan Enter de dong..."
    exit 1
  fi
fi

cd "${ROOT_DIR}"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[ERROR] Chua cai ROS 2 Jazzy."
  echo "Hay cai ROS 2 Jazzy truoc roi chay lai file nay."
  echo
  read -rp "Nhan Enter de dong..."
  exit 1
fi

source /opt/ros/jazzy/setup.bash

echo "[INFO] Kiem tra cac goi can thiet..."

MISSING_PKGS=()

check_pkg() {
  if ! dpkg -s "$1" >/dev/null 2>&1; then
    MISSING_PKGS+=("$1")
  fi
}

check_pkg python3-colcon-common-extensions
check_pkg python3-tk
check_pkg python3-numpy
check_pkg python3-scipy
check_pkg python3-matplotlib
check_pkg ros-jazzy-ros-gz
check_pkg ros-jazzy-ros-gz-sim
check_pkg ros-jazzy-ros-gz-bridge
check_pkg gz-harmonic
check_pkg libgz-sim8-dev
check_pkg libgz-plugin2-dev
check_pkg libgz-transport13-dev
check_pkg libgz-msgs10-dev

if [ "${#MISSING_PKGS[@]}" -gt 0 ]; then
  echo
  echo "[INFO] May dang thieu cac goi:"
  printf '  - %s\n' "${MISSING_PKGS[@]}"
  echo
  echo "[INFO] Se cai cac goi con thieu. Co the can nhap mat khau sudo."
  sudo apt update
  sudo apt install -y "${MISSING_PKGS[@]}"
else
  echo "[OK] Da du goi can thiet."
fi

chmod +x run_delta_control.sh 2>/dev/null || true
chmod +x run_gazebo.sh 2>/dev/null || true

NEED_BUILD=0

if [ ! -f install/setup.bash ]; then
  NEED_BUILD=1
fi

if [ ! -f install/gz_delta_controller2/lib/gz_delta_controller2/libgz_delta_controller2.so ]; then
  NEED_BUILD=1
fi

if [ "${1:-}" = "--rebuild" ]; then
  NEED_BUILD=1
fi

if [ "${NEED_BUILD}" -eq 1 ]; then
  echo
  echo "[INFO] Dang build workspace..."
  colcon build --symlink-install
else
  echo "[OK] Workspace da build san."
fi

source install/setup.bash

echo
echo "[INFO] Dang chay Delta 4DOF..."
echo "[INFO] Se mo Gazebo + bridge + feedback + GUI."
echo

exec ros2 launch delta_control delta_control.launch.py
