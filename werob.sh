#!/usr/bin/env bash
# Source this file for a terminal environment, or execute it to build and launch.
WEROB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo 'ROS 2 Jazzy is required at /opt/ros/jazzy.' >&2
    return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$WEROB_ROOT/src/patrol_simulation/models:/opt/ros/jazzy/share:${GZ_SIM_RESOURCE_PATH:-}"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    set -e
    cd -- "$WEROB_ROOT"
    colcon build --packages-select patrol_simulation --symlink-install
    source install/setup.bash
    exec ros2 launch patrol_simulation bringup.launch.py "$@"
elif [[ -f "$WEROB_ROOT/install/setup.bash" ]]; then
    source "$WEROB_ROOT/install/setup.bash"
fi
