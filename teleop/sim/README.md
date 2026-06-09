# FT2 遥操作 — 纯 Python 仿真验证

不依赖 Gazebo / ROS。用离散时间模型复现本分支（`main_pro`）的 **FT2 位置-力 2 通道**
控制与通信逻辑，并用 matplotlib 出图，验证「通信缓冲」与「控制」两方面的效果。

## 运行

```bash
/opt/miniconda3/bin/python3 teleop/sim/ft2_sim.py
```

依赖：`numpy`、`matplotlib`（已装在 `/opt/miniconda3`）。输出图见 `teleop/sim/figures/`。

## 模型对应关系（代码 → 仿真）

| 真实代码 | 仿真实现 |
|----------|----------|
| `master_arm.cpp` `AdmittanceController` + `commandCartesianToward` | `Admittance1D` + 笛卡尔伺服（导纳给目标，伺服限速趋向）→ 力驱动的 leader |
| `slave_arm.cpp` 刚性位置跟踪（`target = clamp(ref)` + `cartesianCtrlCmd`） | `CART_DIR_GAIN/POS_SPEED` 限速跟踪，无导纳柔顺 |
| `communication.cpp` `JitterBufferNode`（按 seq 入堆、填到 `buffer_depth` 再播放） | `JitterBuffer` 类，逐位复刻 |
| 主→从 只传 pose+vel；从→主 只传 wrench | 两条独立 `JitterBuffer` 通道 + 网络时延/抖动/丢包 |
| 力反馈 `admittanceForce = 本端力 ± 远端力, *4.5` | 主端导纳输入 `F_op - 4.5·f_fb` |

参数取自 `teleop/config/real_robot.yaml` 与各 `.cpp` 顶部 `#define`。模型把笛卡尔
单自由度（x 轴）抽出做 1-DOF 化，足以体现通信与控制的耦合，不追求多关节精确动力学。

## 两张图说明了什么

**`ft2_overview.png`（控制 + 力反馈 + 时延）**
- 上：主端 leader 主导运动，从端刚性跟踪并**滞后约 300ms**（抖动缓冲时延），
  接触墙面后位置出现平台 —— 富接触下操作者「被挡住」。
- 中：操作者推力、从端实测接触力、回传到主端的力（明显延后约 300ms）——
  即从端把接触力反射给操作者，FT2 的核心。
- 下：主→从端到端时延 ≈ `buffer_depth×10ms + 网络`，约 300ms，量化缓冲代价。

**`ft2_comm_buffer.png`（抖动缓冲）**
- 上：发送 vs「无缓冲」（乱序/抖动）vs「有缓冲」（平滑但延迟）——缓冲用延迟换平滑有序。
- 下：缓冲占用先填到 `buffer_depth=30` 再进入稳态播放。

## 仿真得到的一个调参结论

出厂 `real_robot.yaml` 主端 `stiffness=0`，导纳是纯「力→速度」bang-bang 源：一旦推力
非零，leader 即以伺服全速前进。仿真显示，在 `buffer_depth=30`（≈300ms 单程、≈600ms
往返）下，全速 leader 撞向刚性墙会在反馈到达前**盲冲穿越、接触不可控**（接触力数百牛、
发散）。因此本仿真在演示「可控富接触」时取 `K_ADM=300`（见 `ft2_sim.py` 顶部注释）。

**建议**：做接触类任务时，给主端导纳加一定刚度（或减小 `buffer_depth`/网络时延），
以换取稳定可控的接触手感。可在 `ft2_sim.py` 顶部修改 `K_ADM / BUFFER_DEPTH / K_ENV /
F_PEAK` 复现不同区间。
