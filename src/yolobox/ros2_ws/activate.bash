#!/usr/bin/env bash
# Source this file from the workspace root.
source /opt/ros/jazzy/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_merged/local_setup.bash"
export AMENT_PREFIX_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_merged:${AMENT_PREFIX_PATH}"
