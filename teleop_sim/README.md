# teleop_sim

所有命令在仓库根目录执行：`cd /你的路径/z1_teleop`。  
生成的曲线图保存在 **`teleop_sim/figures/`**（运行后打开 PNG 即可）。

---

## 1. 通信验证

**1.1** 打开第一个终端：

```bash
cd /你的路径/z1_teleop
source /opt/ros/noetic/setup.bash
roscore
```

**1.2** 打开第二个终端：

```bash
cd /你的路径/z1_teleop
source /opt/ros/noetic/setup.bash
source ./teleop/ws/devel/setup.bash
python3 ./teleop_sim/sim_verify_communication.py --mode buffer
```

用图片查看器打开 **`teleop_sim/figures/comm_buffer.png`**。  
左上：缓冲前注入的 `seq` 与时间；右上：缓冲输出 `seq` 与时间；下面两图为 `pose[0]` 对应关系。

**1.3** 仍在第二个终端（或再开一个终端并重复 `cd` 与两次 `source`）：

```bash
python3 ./teleop_sim/sim_verify_communication.py --mode packager
```

打开 **`teleop_sim/figures/comm_packager.png`**。  
蓝线：`/master/desired_pose` 的 `position.x`；橙线：`/network/master_frame` 里的 `pose[0]`。

---

## 2. 导纳验证

**2.1 离线曲线（不启动 Gazebo）**

第一个终端：

```bash
cd /你的路径/z1_teleop
source /opt/ros/noetic/setup.bash
roscore
```

第二个终端：

```bash
cd /你的路径/z1_teleop
source /opt/ros/noetic/setup.bash
source ./teleop/ws/devel/setup.bash
python3 ./teleop_sim/sim_verify_admittance.py
```

打开 **`teleop_sim/figures/admittance_offline.png`**。  
上图：仿真力 `Fz`；下图：导纳输出的目标位姿 `z` 与 `x`。

**2.2 Gazebo 曲线（机械臂 + 橙色目标球）**

第一个终端：

```bash
cd /你的路径/z1_teleop
source /opt/ros/noetic/setup.bash
source ./unitree_ws/devel/setup.bash
source ./teleop/ws/devel/setup.bash
export Z1_SDK_LIB=/你的路径/z1_teleop/unitree_ws/src/z1_sdk/lib
export LD_LIBRARY_PATH="$Z1_SDK_LIB:${LD_LIBRARY_PATH:-}"
roslaunch unitree_gazebo z1.launch paused:=false gui:=true
```

第二个终端（再执行一遍上面的 `cd`、三次 `source`、两条 `export`）：g

```bash
python3 ./teleop_sim/gazebo_admittance_visual.py _plot_duration:=12.0
```

Gazebo 里看橙色小球（导纳目标）和机械臂运动。  
脚本在约 12 秒后自动退出，并写出 **`teleop_sim/figures/admittance_gazebo.png`**：上图目标 `z` 与末端 `z`（FK），下图仿真 `Fz`。  
不写 `_plot_duration` 则一直运行、不生成该 PNG。
