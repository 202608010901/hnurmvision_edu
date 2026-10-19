#!/usr/bin/env bash
set -eo pipefail
REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_ROOT="$REPO_ROOT/simulation"
source /opt/ros/humble/setup.bash
cd "$SIM_ROOT"
colcon --log-base log build --base-paths ros2 --build-base build_interfaces --install-base install_interfaces --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$SIM_ROOT/install_interfaces/setup.bash"
cmake -S "$REPO_ROOT" -B "$REPO_ROOT/build" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$REPO_ROOT/build" -j2
