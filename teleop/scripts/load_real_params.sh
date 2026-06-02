#!/usr/bin/env bash
# Load teleop ROS parameters from a YAML file into the parameter server.
# Usage:
#   ./teleop/scripts/load_real_params.sh [path/to.yaml]
# Default config: teleop/config/real_robot.local.yaml (copy from real_robot.example.yaml)
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEFAULT_CFG="${REPO_ROOT}/teleop/config/real_robot.local.yaml"
CFG="${1:-$DEFAULT_CFG}"
if [[ ! -f "$CFG" ]]; then
  echo "Config not found: $CFG" >&2
  echo "Copy ${REPO_ROOT}/teleop/config/real_robot.example.yaml to teleop/config/real_robot.local.yaml and edit IPs, then retry." >&2
  exit 1
fi
rosparam load "$CFG" /
echo "Loaded ROS params from: $CFG"
