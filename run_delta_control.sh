#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "${ROOT_DIR}/install/setup.bash"

set -u

exec ros2 launch delta_control delta_control.launch.py
