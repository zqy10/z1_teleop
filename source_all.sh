#!/bin/bash

# ROS Multi-Workspace Source Script
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash
echo "[1/3] Sourced /opt/ros/noetic"

source "$BASE_DIR/teleop/ws/devel/setup.bash"
MERGED_ROS_PATH=$ROS_PACKAGE_PATH
MERGED_CMAKE_PATH=$CMAKE_PREFIX_PATH
MERGED_PYTHON_PATH=$PYTHONPATH
echo "[2/3] Sourced teleop/ws"

source "$BASE_DIR/unitree_ws/devel/setup.bash"
MERGED_ROS_PATH=$MERGED_ROS_PATH:$ROS_PACKAGE_PATH
MERGED_CMAKE_PATH=$MERGED_CMAKE_PATH:$CMAKE_PREFIX_PATH
MERGED_PYTHON_PATH=$MERGED_PYTHON_PATH:$PYTHONPATH
echo "[3/3] Sourced unitree_ws"

export ROS_PACKAGE_PATH=$MERGED_ROS_PATH
export CMAKE_PREFIX_PATH=$MERGED_CMAKE_PATH
export PYTHONPATH=$MERGED_PYTHON_PATH
export Z1_SDK_LIB="$BASE_DIR/unitree_ws/src/z1_sdk/lib"
export LD_LIBRARY_PATH="$Z1_SDK_LIB:${LD_LIBRARY_PATH:-}"

echo ""
echo "[OK] All workspaces sourced successfully!"
echo ""
echo "ROS_PACKAGE_PATH:"
echo $ROS_PACKAGE_PATH | tr ':' '\n' | sed 's/^/  /'
echo ""
echo "CMAKE_PREFIX_PATH:"
echo $CMAKE_PREFIX_PATH | tr ':' '\n' | sed 's/^/  /'
echo ""
echo "PYTHONPATH:"
echo $PYTHONPATH | tr ':' '\n' | sed 's/^/  /'
echo ""
echo "Z1_SDK_LIB:"
echo "  $Z1_SDK_LIB"
echo ""
echo "LD_LIBRARY_PATH:"
echo $LD_LIBRARY_PATH | tr ':' '\n' | sed 's/^/  /'