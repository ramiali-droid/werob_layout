#!/usr/bin/env bash
# Isolated validation session: never kills or joins a user's running simulation.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${WEROB_VALIDATION_DOMAIN:-87}"
export GZ_PARTITION="werob_validation_$$"
export ROS_LOG_DIR="$PWD/log/validation_ros"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

if [[ $# -eq 0 ]]; then
    set -- --suite
fi
exec xvfb-run -a -s '-screen 0 1280x720x24' python3 scripts/live_validation.py "$@"
