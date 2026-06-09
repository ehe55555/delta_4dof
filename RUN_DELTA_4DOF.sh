#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DESKTOP_FILE="${HOME}/Desktop/Delta_4DOF_Control.desktop"

pause_window() {
  echo
  read -rp "Nhan Enter de dong cua so nay..."
}

echo "========================================"
echo "       DELTA 4DOF FIRST-RUN SETUP"
echo "========================================"
echo "Workspace: ${ROOT_DIR}"
echo

cd "${ROOT_DIR}"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[ERROR] Chua cai ROS 2 Jazzy."
  echo "Hay cai ROS 2 Jazzy truoc roi chay lai file nay."
  pause_window
  exit 1
fi

echo "[INFO] Source ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "[INFO] Kiem tra cac package can thiet..."

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
  echo "[INFO] May dang thieu cac goi sau:"
  printf '  - %s\n' "${MISSING_PKGS[@]}"
  echo
  echo "[INFO] Se cai bang apt. Co the can nhap mat khau sudo."
  sudo apt update
  sudo apt install -y "${MISSING_PKGS[@]}"
else
  echo "[OK] Da du package can thiet."
fi

echo
echo "[INFO] Cap quyen chay cho script..."
chmod +x "${ROOT_DIR}/run_delta_control.sh"
chmod +x "${ROOT_DIR}/run_gazebo.sh" 2>/dev/null || true

NEED_BUILD=0

if [ ! -f "${ROOT_DIR}/install/setup.bash" ]; then
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
  echo
  echo "[OK] Workspace da build san, bo qua buoc build."
fi

echo
echo "[INFO] Tao shortcut ngoai Desktop..."
mkdir -p "${HOME}/Desktop"

cat > "${DESKTOP_FILE}" <<DESKTOP_EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Delta 4DOF Control
Comment=Run Delta 4DOF Gazebo simulation and control GUI
Path=${ROOT_DIR}
Exec=gnome-terminal -- bash -lc "cd '${ROOT_DIR}' && ./run_delta_control.sh; echo; read -rp 'Nhan Enter de dong...'"
Icon=utilities-terminal
Terminal=false
Categories=Development;Robotics;
DESKTOP_EOF

chmod +x "${DESKTOP_FILE}"
gio set "${DESKTOP_FILE}" metadata::trusted true 2>/dev/null || true

echo "[OK] Da tao shortcut:"
echo "     ${DESKTOP_FILE}"

echo
echo "[INFO] Source workspace..."
source "${ROOT_DIR}/install/setup.bash"

echo
echo "[INFO] Dang mo chuong trinh Delta 4DOF..."
echo

"${ROOT_DIR}/run_delta_control.sh"

echo
echo "[INFO] Chuong trinh da ket thuc."
pause_window
