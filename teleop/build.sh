#!/usr/bin/env bash
# Build teleop C++ nodes (teleop_msgs + teleop_arm).
# Run on x86_64 Linux with ROS Noetic sourced.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "Source ROS first:  source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

echo "==> catkin_make in teleop/ws ..."
catkin_make -C teleop/ws

echo ""
echo "==> Build OK. Binaries:"
ls -la teleop/ws/devel/lib/teleop_arm/ 2>/dev/null || ls -la teleop/ws/devel/lib/*/teleop_arm/ 2>/dev/null || true
echo ""
echo "Before rosrun, in EVERY terminal:"
echo "  source ${REPO_ROOT}/source_all.sh"
echo ""
echo "Then:"
echo "  rosrun teleop_arm master_arm"
echo "  rosrun teleop_arm slave_arm"
echo "  rosrun teleop_arm communication _role:=master"
