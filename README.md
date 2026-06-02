# Z1 富接触遥操作（实机）

主从双臂通过 `teleop_msgs/TeleopFrame` 通信，C++ 节点调用 `unitree_arm_sdk` 控制机械臂。
slave: 192.168.120.110
master: 192.168.123.110
---

## 1. 目录结构

```
z1_teleop/
├── README.md
├── source_all.sh              # 一键 source ROS 工作空间
├── teleop/
│   ├── scripts/               # C++ 源码（由 teleop/ws 编译为可执行文件）
│   │   ├── master_arm.cpp     # 主端完整实现（MASTER_ARM_IP / 8071/8072）
│   │   ├── slave_arm.cpp      # 从端完整实现（SLAVE_ARM_IP / 8073/8074）
│   │   ├── communication.cpp
│   │   └── load_real_params.sh
│   ├── config/                # ROS 参数 YAML
│   └── ws/                    # teleop_msgs + teleop_arm 编译
└── unitree_ws/                # z1_sdk、z1_controller 等（勿改 SDK）
```

---

## 2. 运行流程（每个臂节点）

启动后自动执行：

1. **回零竖直**：`backToStart()`，机械臂到 SDK 定义的竖直 home 位
2. **静止等待**：默认 10 s（`~init_wait_sec`），保持该位姿、不启用导纳
3. **导纳遥操作**：默认持续 300 s（5 分钟，`~run_duration_sec`）
4. **自动结束**：再次 `backToStart()`，切 `PASSIVE`，节点退出

修改等待/时长：编辑 `teleop/config/real_robot.yaml` 中 `init_wait_sec`、`run_duration_sec`（改后重新 `load_real_params.sh`），或改 `master_arm.cpp` 顶部默认值后重新编译。

---

## 3. 编译与运行（必读）

`teleop/scripts/*.cpp` 是**源码**，不能 `./master_arm.cpp` 直接跑。  
流程是：**catkin 编译 → source → rosrun**（或运行生成的二进制）。

### 3.1 环境要求

- Ubuntu + **ROS Noetic**（x86_64）
- 已存在 `unitree_ws/src/z1_sdk`（含 `libZ1_SDK_x86_64.so`）

### 3.2 一次性编译

```bash
cd /path/to/z1_teleop
source /opt/ros/noetic/setup.bash

# 方式 A（推荐）
./teleop/build.sh

# 方式 B（等价）
catkin_make -C teleop/ws
```

成功后会生成可执行文件（路径固定）：

```
teleop/ws/devel/lib/teleop_arm/master_arm
teleop/ws/devel/lib/teleop_arm/slave_arm
teleop/ws/devel/lib/teleop_arm/communication
```

可用下面命令确认是否已编译：

```bash
ls teleop/ws/devel/lib/teleop_arm/
```

若目录不存在或为空，说明**还没编译**，此时 `rosrun teleop_arm ...` 会报 `package 'teleop_arm' not found`。

### 3.3 每个终端运行前必须 source

```bash
source /path/to/z1_teleop/source_all.sh
```

该脚本会 source `teleop/ws/devel/setup.bash`（注册 `teleop_arm` 包）、设置 `LD_LIBRARY_PATH` 指向 `z1_sdk` 动态库。  
**不 source 就 rosrun** 常见报错：找不到包、或 `libZ1_SDK_x86_64.so: cannot open shared object file`。

### 3.4 用 rosrun 启动（编译 + source 之后）

```bash
rosrun teleop_arm master_arm
rosrun teleop_arm slave_arm
rosrun teleop_arm communication _role:=master   # 或 _role:=slave
```

`rosrun` 只是启动上面 `devel/lib/teleop_arm/` 里编好的程序，**前提是第 3.2、3.3 步已完成**。

### 3.5 不用 rosrun 的等价方式（调试用）

```bash
source /path/to/z1_teleop/source_all.sh
/path/to/z1_teleop/teleop/ws/devel/lib/teleop_arm/master_arm
```

### 3.6 修改 C++ 后

改 `teleop/scripts/*.cpp` 或 `master_arm.cpp` 里 UDP 配置后，必须重新编译：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C teleop/ws
source teleop/ws/devel/setup.bash   # 或 source_all.sh
```

---

## 3b. SDK 接口核对（均已存在于 z1_sdk）

| 代码调用 | 头文件 / 示例依据 |
|----------|-------------------|
| `CtrlComponents`, `UDPPort`, `RECVSTATE_LENGTH` | `ctrlComponents.h`, `udp.h`, `arm_common.h` |
| `Z1Model()`, `addLoad()` | `ArmModel.h` |
| `unitreeArm(CtrlComponents*)` | `unitreeArm.h` |
| `sendRecvThread->start/shutdown` | `unitreeArm.h` |
| `backToStart()`, `startTrack(CARTESIAN)`, `cartesianCtrlCmd` | `unitreeArm.h`, `examples/highcmd_basic.cpp` |
| `lowstate->endPosture` | `LowlevelState.h` |
| `setFsm(PASSIVE)` | `unitreeArm.h` |

未使用已废弃的 Python 接口（`unitree_arm_sdk`、`UnitreeArm(ip)`、`getF_T_EE` 等）。

---

## 4. 实机前置条件

### 4.1 修改 UDP（必做）

与官方例程一致，主从 **分开配置**：

**主端** `teleop/scripts/master_arm.cpp`：

```cpp
#define MASTER_ARM_IP "127.0.0.1"
#define MASTER_LOCAL_PORT 8071
#define MASTER_REMOTE_PORT 8072
```

**从端** `teleop/scripts/slave_arm.cpp`：

```cpp
#define SLAVE_ARM_IP "127.0.0.1"
#define SLAVE_LOCAL_PORT 8073
#define SLAVE_REMOTE_PORT 8074
```

改完后：`catkin_make -C teleop/ws && source ./source_all.sh`

### 4.2 启动 z1_controller

| 臂   | 程序              | ARMSDK 端口（controller 侧） |
|------|-------------------|------------------------------|
| 主臂 | `z1_controller`   | 8072 / 8071                  |
| 从臂 | `z1_controller_111` | 8074 / 8073                |

两台 controller 与遥操作节点之间的 UDP 端口必须与上表、与 `master_arm.cpp` 中定义一致。

### 4.3 力传感器

力传感器节点需发布 `geometry_msgs/Wrench`（或 `WrenchStamped` 经 relay）。在 YAML 中配置：

- 主端：`master_arm/force_sensor_topic`（如 `/master_force`）
- 从端：`slave_arm/force_sensor_topic`（如 `/slave_force`）

### 4.4 ROS 参数

编辑 `teleop/config/real_robot.yaml`（每个参数含义见文件内注释）。

在 **roscore 已运行** 时加载：

```bash
./teleop/scripts/load_real_params.sh
```

---

## 5. 主端电脑部署

将 `192.168.1.10` 换成你的主端 IP。每个终端先执行：

```bash
source /path/to/z1_teleop/source_all.sh
export ROS_IP=192.168.1.10
export ROS_HOSTNAME=$ROS_IP
```

**终端 1 — roscore**

```bash
roscore
```

**终端 2 — 加载参数（每次改 YAML 后执行一次）**

```bash
./teleop/scripts/load_real_params.sh
```

**终端 3 — 主臂控制**

```bash
rosrun teleop_arm master_arm
```

**终端 4 — 通信（主）**

```bash
rosrun teleop_arm communication _role:=master
```

---

## 6. 从端电脑部署

将 `192.168.1.10` / `192.168.1.20` 换成主端 / 从端 IP。每个终端：

```bash
source /path/to/z1_teleop/source_all.sh
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.20
export ROS_HOSTNAME=$ROS_IP
```

**终端 1 — 从臂控制**

```bash
rosrun teleop_arm slave_arm
```

**终端 2 — 通信（从）**

```bash
rosrun teleop_arm communication _role:=slave
```

---

## 7. 建议启动顺序

1. 两台机械臂上电，分别启动 `z1_controller` / `z1_controller_111`
2. 主端 `roscore`
3. `load_real_params.sh`
4. 主端 `master_arm` → 从端 `slave_arm`（等待各自完成回零与 10 s 静止）
5. 主端 `communication _role:=master` → 从端 `communication _role:=slave`
6. 约 5 分钟后两臂节点自动回零退出；关闭 controller

---

## 8. 常见问题

| 现象 | 检查 |
|------|------|
| SDK 连不上 | `ARM_UDP_IP`、端口是否与 controller 一致；防火墙 |
| 从臂不动 | 是否已启动 `communication`；`rostopic echo /slave/teleop_frame` 是否有数据 |
| 力无反馈 | `force_sensor_topic` 与实机话题名是否一致 |
| 编译找不到 SDK | `unitree_ws/src/z1_sdk` 是否存在；是否在 x86_64 Linux 上编译 |
| `rosrun teleop_arm` 找不到包 | 先 `catkin_make -C teleop/ws`，再 `source source_all.sh` |
| 找不到 `libZ1_SDK_x86_64.so` | 运行前必须 `source source_all.sh`（设置 `LD_LIBRARY_PATH`） |
