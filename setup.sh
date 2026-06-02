#!/bin/bash
set -e

cd "$(dirname "$0")"

echo "==> [1/3] Building Docker image (VNC mode)..."
docker compose build --no-cache

echo "==> [2/3] Starting container..."
docker compose up -d

echo "==> [3/3] Waiting for VNC server to start..."
sleep 5

echo ""
echo "✅ Done! Now:"
echo "   1. Open 'Screen Sharing' on your Mac (Cmd+Space → 'Screen Sharing')"
echo "      OR install: brew install --cask vnc-viewer"
echo "   2. Connect to:  localhost:5901"
echo "   3. Password:    ros1234"
echo ""
echo "   Inside the VNC desktop, open a terminal and run:"
echo "   source /opt/ros/noetic/setup.bash"
echo "   rosrun rviz rviz"
echo "   # or"
echo "   roslaunch gazebo_ros empty_world.launch"
echo ""
echo "   To open a shell in the container anytime:"
echo "   docker exec -it ros1_teleop bash"