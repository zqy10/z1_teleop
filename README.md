# 富接触遥操作

主从导纳 + `teleop_msgs/TeleopFrame` 通信；仿真说明见 [`teleop_sim/README.md`](teleop_sim/README.md)。

---

## 1. 目录结构

```bash
z1_teleop/
├── README.md
├── teleop/
│   ├── scripts/          # 主要代码
│   ├── config/           # 配置参数
│   └── ws/               # 自定义消息 teleop_msgs
├── teleop_sim/           # 仿真脚本与结果
└── unitree_ws/
```

---

## 2. 编译 & source
最好重新编译一下 ./teleop/ws 和 ./unitree_ws

source 的话可以直接用 `source ./source_all.sh`，这个脚本能把所有环境都包括进去，或者手动source + export:
```bash
source /opt/ros/noetic/setup.bash 
source ./teleop/ws/devel/setup.bash #自定义的message
source ./unitree_ws/devel/setup.bash  
export Z1_SDK_LIB=/你的路径/z1_teleop/unitree_ws/src/z1_sdk/lib
export LD_LIBRARY_PATH="$Z1_SDK_LIB:${LD_LIBRARY_PATH:-}"
```
有可能会出现source 互相覆盖的问题，可以echo一下 PYTHONPATH 和 ROS_PACKAGE_PATH 看看是不是真的把路径添加进去了，如果没有就手动添加路径：
```bash
export PYTHONPATH=[path]:$PYTHONPATH #类似这样的指令可手动添加对应路径不会覆盖
```
把整个文件夹z1_teleop复制到主从两端。

---

## 3. 参数
参数在./teleop/config里快速设置

在 `real_robot.local.yaml` 里改这几个：

- `master.arm_ip`、`slave.arm_ip`：两台 Z1 的 IP  
- `master_arm.force_sensor_topic`、`slave_arm.force_sensor_topic`：力传感器的话题名称（例如 `/master_force` 和 `/slave_force`）  
- `master_arm` / `slave_arm` 里的 `mass`、`damping` 等导纳数  

每次修改参数，保存后要在有roscore的时候，执行参数加载的脚本才能在当前roscore生效：

```bash
./teleop/scripts/load_real_params.sh
```
---

## 4. 实机部署：主端电脑

接下来的步骤中把 **`192.168.1.10`** 当作主端 IP；把 **`192.168.1.20`** 当作从端 IP，实际需替换。每开一个终端都记得：
```bash
source ./source_all.sh
#配置ip和端口的操作，我不太清楚实机怎么做的，还是以实机真实情况为准
export ROS_IP=192.168.1.10 #主端ip
export ROS_HOSTNAME=$ROS_IP 
```

**终端 1 — roscore**

```bash
roscore
```

**终端 2 — 主程序**
主端控制代码
```bash
python3 ./teleop/scripts/master_arm.py
```

**终端 3 — 通信（主）**
主端通信代码（和从端共用的，但是要指定角色用以区分）
```bash
python3 ./teleop/scripts/communication.py --role master
```

---

## 5. 实机部署：从端电脑
每开一个终端都记得：
```bash
source ./source_all.sh
#配置ip和端口的操作，我不太清楚实机怎么做的，还是以实机真实情况为准
export ROS_MASTER_URI=http://192.168.1.10:11311 #主端ip(因为roscore在主端)
export ROS_IP=192.168.1.20 #从端ip
export ROS_HOSTNAME=$ROS_IP 
```

**终端 1 — 从程序**
葱段控制代码
```bash
python3 ./teleop/scripts/slave_arm.py
```

**终端 2 — 通信（从）**
从端通信
```bash
python3 ./teleop/scripts/communication.py --role slave
```

---

## 6. Docker

实机不需要考虑`setup.sh`和`vnc.sh`都是我运行容器时用的脚本，`teleop_sim`是仿真代码。
