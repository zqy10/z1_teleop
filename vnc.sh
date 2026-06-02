docker exec -d ros1_teleop bash -c "
vncserver -kill :1 2>/dev/null || true
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true
sleep 1
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no
"