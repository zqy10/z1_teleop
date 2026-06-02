# TeleopFrame.msg
std_msgs/Header header
uint32 seq_num        # 序列号
float64[6] pose       # [x, y, z, rx, ry, rz]
float64[6] velocity   # [vx, vy, vz, wx, wy, wz]
float64[6] wrench     # [fx, fy, fz, tx, ty, tz]