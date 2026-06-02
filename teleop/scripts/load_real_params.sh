#!/usr/bin/env bash
# Load teleop ROS parameters from teleop/config/real_robot.yaml
# Usage:
#   ./teleop/scripts/load_real_params.sh
# Requires: roscore running
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG="${REPO_ROOT}/teleop/config/real_robot.yaml"
if [[ ! -f "$CFG" ]]; then
  echo "Config not found: $CFG" >&2
  exit 1
fi
rosparam load "$CFG" /
echo "Loaded ROS params from: $CFG"
