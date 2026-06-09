#!/usr/bin/env python3
"""FT2（位置-力 2 通道）遥操作的纯 Python 仿真与可视化。

不依赖 Gazebo / ROS：用离散时间模型复现本仓库 main_pro 分支的控制与通信逻辑，
并用 matplotlib 出图，直观体现两件事：

  1. 通信缓冲（communication.cpp 里的 JitterBuffer）——抖动缓冲如何用
     buffer_depth 帧的代价换取平滑、有序的播放，以及它引入的固定时延。
  2. 控制效果（master_arm.cpp / slave_arm.cpp 的 FT2 结构）——主端导纳 leader
     带动从端刚性跟踪，从端接触力经通道回传后让 leader 在「墙」前停住，
     即富接触遥操作中操作者「摸到」环境的过程。

本模型把笛卡尔某一个自由度（x 轴）抽出来做 1-DOF 化处理，足以体现通信与控制的
耦合效果；参数取自 teleop/config/real_robot.yaml 与各 .cpp 顶部的 #define。

运行：
    /opt/miniconda3/bin/python3 teleop/sim/ft2_sim.py
输出：
    teleop/sim/figures/ft2_overview.png   控制 + 力反馈 + 端到端时延
    teleop/sim/figures/ft2_comm_buffer.png 抖动缓冲平滑效果 + 缓冲占用
"""

import os
import heapq
import numpy as np
import matplotlib

matplotlib.use("Agg")  # 无显示环境下出图
import matplotlib.pyplot as plt


# =============================================================================
# 参数（对齐 real_robot.yaml 与 *.cpp 的 #define）
# =============================================================================
DT = 0.01                 # 仿真 / 打包 / 播放节拍 100Hz（communication.cpp 定时器 0.01s）
T_END = 8.0
N = int(T_END / DT)

# 主端导纳 leader（master_arm.cpp / yaml: master_arm）
M_ADM = 0.5               # mass（同 yaml）
B_ADM = 2.0               # damping（同 yaml）
# stiffness：出厂 yaml 为 0（纯「力->速度」bang-bang 源）。仿真发现：在 300ms 抖动缓冲
# 下，K=0 的全速 leader 撞向刚性墙会盲冲穿越、接触不可控（见 README）。这里取 K=300
# 演示「可控的富接触」区间——同时也是面向接触任务的调参建议（接触时给 leader 加刚度）。
K_ADM = 300.0             # stiffness（仿真演示值；出厂 yaml=0）
MAX_CORR = 0.2            # AdmittanceController::max_correction_
MAX_VEL = 0.4             # AdmittanceController::max_velocity_
FB_SCALE = 4.5            # admittanceForce() 里 out *= 4.5 的力反馈缩放

# 从端刚性跟踪（slave_arm.cpp: commandCartesianToward）
CART_DIR_GAIN = 10.0      # CARTESIAN_DIR_GAIN
CART_POS_SPEED = 0.8      # CARTESIAN_POS_SPEED

# 通信抖动缓冲（communication.cpp: JitterBufferNode, yaml buffer_depth）
BUFFER_DEPTH = 30         # 30 帧 ≈ 300ms
NET_BASE_DELAY = 0.02     # 网络基础单程时延 (s)
NET_JITTER = 0.03         # 附加抖动上限 (s)，造成乱序/late
DROP_PROB = 0.01          # 丢包概率

# 环境（仿真用）：x_wall 处有一面墙，接触刚度 k_env
X_WALL = 0.12
K_ENV = 120.0             # N/m，接触刚度（富接触）

RNG = np.random.default_rng(7)


# =============================================================================
# 通信：抖动缓冲，复刻 communication.cpp 的 JitterBufferNode 行为
# =============================================================================
class Network:
    """单程网络：固定基础时延 + 随机抖动（可乱序）+ 丢包。"""

    def __init__(self, base_delay, jitter, drop_prob):
        self.base_delay = base_delay
        self.jitter = jitter
        self.drop_prob = drop_prob

    def arrival_time(self, send_time):
        if RNG.random() < self.drop_prob:
            return None  # 丢包
        return send_time + self.base_delay + RNG.random() * self.jitter


class JitterBuffer:
    """对应 communication.cpp::JitterBufferNode。

    - 按 seq 入小顶堆，丢弃 seq <= last_seq 的旧帧；
    - 缓冲达到 target_depth 后开始播放（is_playing）；
    - 每个播放节拍弹出当前最小 seq 的一帧；空则保持上一帧（last_valid）。
    """

    def __init__(self, target_depth):
        self.target_depth = target_depth
        self.heap = []                # (seq, send_time, payload)
        self.is_playing = False
        self.last_seq = -1
        self.last_valid = None        # (payload, send_time)
        self.occupancy_log = []

    def deliver(self, seq, send_time, payload):
        if seq <= self.last_seq:
            return
        heapq.heappush(self.heap, (seq, send_time, payload))
        if not self.is_playing and len(self.heap) >= self.target_depth:
            self.is_playing = True

    def playout(self):
        if not self.is_playing or not self.heap:
            out = self.last_valid
        else:
            seq, send_time, payload = heapq.heappop(self.heap)
            self.last_seq = seq
            self.last_valid = (payload, send_time)
            out = self.last_valid
        self.occupancy_log.append(len(self.heap))
        return out  # (payload, original_send_time) 或 None


# =============================================================================
# 主端导纳 leader，复刻 AdmittanceController::computeTargetPose（1-DOF）
# =============================================================================
class Admittance1D:
    def __init__(self, m, b, k, dt):
        self.m_inv = 1.0 / max(m, 1e-6)
        self.b, self.k, self.dt = b, k, dt
        self.dx = 0.0
        self.dxd = 0.0

    def step(self, base_pose, force):
        corr = force - self.b * self.dxd - self.k * self.dx
        dxdd = self.m_inv * corr
        self.dxd += dxdd * self.dt
        self.dx += self.dxd * self.dt
        self.dx = float(np.clip(self.dx, -MAX_CORR, MAX_CORR))
        self.dxd = float(np.clip(self.dxd, -MAX_VEL, MAX_VEL))
        return base_pose + self.dx


# =============================================================================
# 仿真主循环：闭环 主端导纳 -> 通道 -> 从端刚性跟踪 -> 接触 -> 通道 -> 力反馈
# =============================================================================
F_PEAK = 3.0  # 操作者推力峰值 (N)


def operator_force(t):
    """操作者作用在主端力传感器上的推力剖面（+x 推、保持、回撤）。"""
    if t < 1.0:
        return F_PEAK * (t / 1.0)             # 0..1s 渐推到峰值
    if t < 5.0:
        return F_PEAK                         # 1..5s 持续推（压向墙）
    if t < 5.5:
        return F_PEAK * (1.0 - 2.0 * (t - 5.0))  # 5..5.5s 撤力
    return 0.0


def run_sim():
    t = np.arange(N) * DT

    adm = Admittance1D(M_ADM, B_ADM, K_ADM, DT)
    m2s = JitterBuffer(BUFFER_DEPTH)   # 主->从：位置通道
    s2m = JitterBuffer(BUFFER_DEPTH)   # 从->主：力通道
    net_m2s = Network(NET_BASE_DELAY, NET_JITTER, DROP_PROB)
    net_s2m = Network(NET_BASE_DELAY, NET_JITTER, DROP_PROB)

    # 网络在途队列：list of (arrival_time, seq, send_time, payload)
    inflight_m2s, inflight_s2m = [], []

    x_master = np.zeros(N)     # 主端 leader 位姿（命令/实测，1-DOF）
    x_slave = np.zeros(N)      # 从端实际位姿
    f_contact = np.zeros(N)    # 从端测得接触力
    f_fb = np.zeros(N)         # 主端收到的回传力
    f_op = np.zeros(N)         # 操作者力
    x_slave_recv = np.zeros(N) # 从端从缓冲拿到的主端位姿
    lat_m2s = np.full(N, np.nan)  # 主->从端到端时延（播放时刻 - 原始发送时刻）

    # 对照组：从端「不带抖动缓冲」时直接用最近到达帧（体现缓冲平滑作用）
    x_slave_recv_raw = np.zeros(N)
    raw_latest = (0.0, 0.0)    # (payload, send_time)

    xs = 0.0
    xm = 0.0
    f_fb_now = 0.0
    f_contact_now = 0.0
    force_in_prev = 0.0

    for k in range(N):
        tk = k * DT
        f_op[k] = operator_force(tk)

        # --- 主端导纳 leader：输入 = 操作者力 - 回传接触力（让操作者感到阻力） ---
        # 导纳给出目标位姿（自身位姿 + 受力位移 dX，dX 有界）；真实 leader 还有一层
        # 笛卡尔伺服（commandCartesianToward），实际位姿以有限速度趋向该目标，
        # 因此 leader 表现为「力->速度」源：持续推 -> 以伺服速度前进；回传力反向时停住/后退。
        target_m = adm.step(xm, force_in_prev)
        dir_m = np.clip((target_m - xm) * CART_DIR_GAIN, -1.0, 1.0)
        xm += dir_m * CART_POS_SPEED * DT
        x_master[k] = xm
        force_in_prev = f_op[k] - FB_SCALE * f_fb_now  # 下一拍导纳输入

        # --- 主->从 打包发送（位置通道，seq=k） ---
        at = net_m2s.arrival_time(tk)
        if at is not None:
            inflight_m2s.append((at, k, tk, xm))

        # 网络投递：到达时间 <= 当前 tk 的帧入抖动缓冲
        still = []
        for (a, seq, st, pl) in inflight_m2s:
            if a <= tk:
                m2s.deliver(seq, st, pl)
                if st >= raw_latest[1]:        # 对照：无缓冲，取最近到达
                    raw_latest = (pl, st)
            else:
                still.append((a, seq, st, pl))
        inflight_m2s = still

        # --- 从端抖动缓冲播放，取出主端位姿 ---
        out = m2s.playout()
        if out is not None:
            xm_recv, send_time = out
            lat_m2s[k] = tk - send_time
        else:
            xm_recv = 0.0
        x_slave_recv[k] = xm_recv
        x_slave_recv_raw[k] = raw_latest[0]

        # --- 从端刚性位置跟踪（cartesianCtrlCmd 模型，无导纳柔顺） ---
        direction = np.clip((xm_recv - xs) * CART_DIR_GAIN, -1.0, 1.0)
        xs += direction * CART_POS_SPEED * DT
        x_slave[k] = xs

        # --- 环境接触：从端压过墙面则产生接触力 ---
        f_contact_now = K_ENV * max(0.0, xs - X_WALL)
        f_contact[k] = f_contact_now

        # --- 从->主 打包发送（力通道，seq=k） ---
        at2 = net_s2m.arrival_time(tk)
        if at2 is not None:
            inflight_s2m.append((at2, k, tk, f_contact_now))
        still2 = []
        for (a, seq, st, pl) in inflight_s2m:
            if a <= tk:
                s2m.deliver(seq, st, pl)
            else:
                still2.append((a, seq, st, pl))
        inflight_s2m = still2

        out2 = s2m.playout()
        if out2 is not None:
            f_fb_now = out2[0]
        f_fb[k] = f_fb_now

    return dict(
        t=t, x_master=x_master, x_slave=x_slave, x_slave_recv=x_slave_recv,
        x_slave_recv_raw=x_slave_recv_raw, f_contact=f_contact, f_fb=f_fb,
        f_op=f_op, lat_m2s=lat_m2s,
        occ_m2s=np.array(m2s.occupancy_log), occ_s2m=np.array(s2m.occupancy_log),
    )


# =============================================================================
# 出图
# =============================================================================
def plot_overview(d, outpath):
    t = d["t"]
    fig, ax = plt.subplots(3, 1, figsize=(11, 11), sharex=True)

    # 1) 位置：主端 leader / 从端跟踪 / 墙
    ax[0].plot(t, d["x_master"], label="master (leader, commanded)", lw=2)
    ax[0].plot(t, d["x_slave"], label="slave (follower, actual)", lw=2)
    ax[0].axhline(X_WALL, color="k", ls="--", lw=1, label="wall (contact at x=%.2f)" % X_WALL)
    ax[0].set_ylabel("position x [m]")
    ax[0].set_title("FT2 control: leader-driven motion, stiff follower, contact plateau")
    ax[0].legend(loc="upper left")
    ax[0].grid(alpha=0.3)

    # 2) 力：操作者力 / 从端接触力 / 主端收到的回传力
    ax[1].plot(t, d["f_op"], label="operator force on master", lw=2)
    ax[1].plot(t, d["f_contact"], label="slave contact force (measured)", lw=2)
    ax[1].plot(t, d["f_fb"], label="force fed back to master (delayed)", lw=2, ls="--")
    ax[1].set_ylabel("force [N]")
    ax[1].set_title("FT2 force feedback: slave contact force returns to the operator")
    ax[1].legend(loc="upper left")
    ax[1].grid(alpha=0.3)

    # 3) 端到端时延（主->从），凸显抖动缓冲带来的固定时延
    lat_ms = d["lat_m2s"] * 1000.0
    ax[2].plot(t, lat_ms, label="master→slave end-to-end latency", lw=1.5)
    nominal = (BUFFER_DEPTH * DT + NET_BASE_DELAY) * 1000.0
    ax[2].axhline(nominal, color="r", ls=":", lw=1.5,
                  label="≈ buffer_depth×10ms + net = %.0f ms" % nominal)
    ax[2].set_ylabel("latency [ms]")
    ax[2].set_xlabel("time [s]")
    ax[2].set_title("Communication buffer cost: fixed playout latency")
    ax[2].legend(loc="upper right")
    ax[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(outpath, dpi=130)
    plt.close(fig)


def plot_comm_buffer(d, outpath):
    t = d["t"]
    fig, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    # 1) 抖动缓冲的平滑作用：原始发送 vs 无缓冲(乱序/抖动) vs 有缓冲(平滑、延迟)
    ax[0].plot(t, d["x_master"], label="sent by master", lw=2)
    ax[0].plot(t, d["x_slave_recv_raw"], label="received WITHOUT buffer (jittery/reordered)",
               lw=1, alpha=0.7)
    ax[0].plot(t, d["x_slave_recv"], label="received WITH jitter buffer (smooth, delayed)",
               lw=2, ls="--")
    ax[0].set_ylabel("position x [m]")
    ax[0].set_title("Jitter buffer smooths reordered/jittery packets at the cost of delay")
    ax[0].legend(loc="upper left")
    ax[0].grid(alpha=0.3)

    # 2) 缓冲占用：先填到 buffer_depth 再稳态播放
    n = len(d["occ_m2s"])
    tt = np.arange(n) * DT
    ax[1].plot(tt, d["occ_m2s"], label="master→slave buffer occupancy", lw=1.5)
    ax[1].axhline(BUFFER_DEPTH, color="r", ls=":", lw=1.5,
                  label="buffer_depth = %d frames" % BUFFER_DEPTH)
    ax[1].set_ylabel("frames in buffer")
    ax[1].set_xlabel("time [s]")
    ax[1].set_title("Buffer fills to depth, then plays out in steady state")
    ax[1].legend(loc="upper right")
    ax[1].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(outpath, dpi=130)
    plt.close(fig)


def print_summary(d):
    lat = d["lat_m2s"][~np.isnan(d["lat_m2s"])] * 1000.0
    print("==== FT2 simulation summary ====")
    print(" master->slave latency: mean=%.0f ms  max=%.0f ms (buffer_depth=%d, ~%.0fms nominal)"
          % (lat.mean(), lat.max(), BUFFER_DEPTH, (BUFFER_DEPTH * DT + NET_BASE_DELAY) * 1000))
    print(" slave-follows-master transport lag (buffer-induced): ~%.0f ms" % lat.mean())
    print(" max slave contact force: %.1f N at wall x=%.2f" % (d["f_contact"].max(), X_WALL))
    print(" max force fed back to operator: %.1f N" % d["f_fb"].max())
    print(" leader peak penetration past wall: %.3f m (delay-induced overshoot)"
          % (d["x_master"].max() - X_WALL))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    figdir = os.path.join(here, "figures")
    os.makedirs(figdir, exist_ok=True)

    d = run_sim()
    print_summary(d)

    plot_overview(d, os.path.join(figdir, "ft2_overview.png"))
    plot_comm_buffer(d, os.path.join(figdir, "ft2_comm_buffer.png"))
    print("figures written to %s" % figdir)


if __name__ == "__main__":
    main()
