#!/usr/bin/env bash
set -eo pipefail
REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_ROOT="$REPO_ROOT/simulation"
source /opt/ros/humble/setup.bash
source "$SIM_ROOT/install_interfaces/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89}
cd "$REPO_ROOT"
exec ./build/training configs/simulation.yaml
