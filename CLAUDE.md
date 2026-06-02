# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo does

Master–slave bilateral teleoperation for Unitree Z1 arms. A master arm measures the operator's motion and force; a slave arm mirrors it with admittance control. Communication uses a custom `teleop_msgs/TeleopFrame` ROS message. Everything is C++/ROS Noetic running on x86_64 Linux. 

Note: The development process of this project is on Mac so there's no need to try to compile or run this code locally. Just focus on code structure and the logic.

Hardware: In real test, there are two robotic arms, two force sensors and only one computer to connect every thing. The force sensors will send data through ros topic from USB ports. The two robotic arms are connnected to the computer using network cable,with slave at `192.168.120.110`, port 8073 and 8074; master at `192.168.123.110`, port 8071 and 8072 (using two ports to differ input and output).

## Build

Requires Ubuntu + ROS Noetic (x86\_64 only — SDK is a prebuilt `.so`).

```bash
source /opt/ros/noetic/setup.bash

# First-time or after editing any .cpp
./teleop/build.sh          # wraps: catkin_make -C teleop/ws
```

Binaries land at `teleop/ws/devel/lib/teleop_arm/{master_arm,slave_arm,communication,admittance_debug_logger}`.

The `unitree_ws/` catkin workspace contains SDK packages (`z1_sdk`, `z1_controller`, `unitree_legged_msgs`). **Do not modify SDK source.**

## Running

Every terminal needs the workspaces and SDK library path:

```bash
source ./source_all.sh     # sources Noetic + teleop/ws + unitree_ws, sets LD_LIBRARY_PATH
```

Then (in order):

```bash
# Terminal 1
roscore

# Terminal 2 (after roscore)
./teleop/scripts/load_real_params.sh   # loads teleop/config/real_robot.yaml into ROS param server

# Terminal 3 — launch master 
rosrun teleop_arm master_arm

# Terminal 4 — launch slave
rosrun teleop_arm slave_arm

# Terminal 5 — launch communication bridge on the master side
rosrun teleop_arm communication _role:=master

# Terminal 6 — launch communication bridge on the slave side
rosrun teleop_arm communication _role:=slave
```

## Code layout

```
teleop/scripts/          # C++ source (compiled by teleop/ws)
  master_arm.cpp         # admittance control loop; publishes /master_arm/{tool_pose,tool_velocity,wrench}
  slave_arm.cpp          # mirrors master pose via admittance; subscribes /slave/teleop_frame
  communication.cpp      # ROS bridge: packs TeleopFrame, runs jitter buffer
  admittance_debug_logger.cpp  # optional debug subscriber
  load_real_params.sh    # rosparam load teleop/config/real_robot.yaml

teleop/config/
  real_robot.yaml        # all tunable ROS params (admittance gains, timing, workspace limits)

teleop/ws/src/
  teleop_msgs/           # TeleopFrame.msg: [pose6, velocity6, wrench6]
  teleop_arm/            # catkin package; CMakeLists pulls sources from teleop/scripts/

unitree_ws/src/z1_sdk/   # prebuilt SDK (libZ1_SDK_x86_64.so + headers)
source_all.sh            # one-shot env setup
teleop/build.sh          # one-shot build script
```

## Key architecture points

- **UDP IPs and ports are compile-time `#define`s**, not ROS params. Edit the `★★★` block at the top of `master_arm.cpp` / `slave_arm.cpp`, then rebuild:
  - Master: `MASTER_ARM_IP`, `MASTER_LOCAL_PORT` (8071), `MASTER_REMOTE_PORT` (8072)
  - Slave: `SLAVE_ARM_IP`, `SLAVE_LOCAL_PORT` (8073), `SLAVE_REMOTE_PORT` (8074)

- **Session lifecycle** (both arms): `backToStart()` → wait `init_wait_sec` → admittance teleop for `run_duration_sec` → `backToStart()` → `PASSIVE` → node exits. Tune these in `real_robot.yaml`.

- **`communication` node** handles jitter buffering (`buffer_depth` in YAML, default 30 frames ≈ 300 ms latency). It does not touch the SDK — it only bridges ROS topics across the network.

- **Force feedback** is off by default on the master (`enable_force_feedback: false`) and on by default on the slave. When enabled, the admittance input is `local_force_sensor − remote_wrench`.

- **Workspace soft limits** are enforced in Cartesian space; configure `workspace_pos_min/max` and `workspace_rpy_abs_max` in YAML.
