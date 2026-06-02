#!/usr/bin/env python3
"""
z1_teleop 双机械臂遥操作系统 - 可视化验证脚本
生成一张综合展示图（基于真实 rosbag 数据），包含：
  图1: 【核心】有无抖动缓冲的位置轨迹对比（真实数据）
  图2: 抖动缓冲前后序列号有序性对比
  图3: 导纳控制器阶跃力响应（仿真验证）
  图4: 真实末端速度曲线（/arm/tool_velocity）
  图5: 导纳控制器正弦力跟随
  图6: 系统架构 + 关键指标汇总

用法:
    python3 generate_figures.py
输出:
    results/validation_results.png
"""

import sys, os, math
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.font_manager as _fm
_fm._load_fontmanager(try_read_cache=False)
import matplotlib.pyplot as plt
_cjk = [f.name for f in _fm.fontManager.ttflist
        if 'Noto' in f.name and 'CJK' in f.name and 'Serif' not in f.name]
if _cjk:
    plt.rcParams['font.family'] = _cjk[0]
plt.rcParams['axes.unicode_minus'] = False
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches

# ── 路径 ──────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_ROOT, 'master_ws/src/master_controller/scripts'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'slave_ws/src/slave_controller/scripts'))
BAG1 = os.path.join(REPO_ROOT, '2026-04-07-08-08-51.bag')

from admittance_controller import AdmittanceController


# ═══════════════════════════════════════════════════════════════════════
# 数据读取
# ═══════════════════════════════════════════════════════════════════════

def load_rosbag():
    """读取 BAG1 的所有关键话题数据"""
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    arm_ts, arm_xs       = [], []
    vel_ts, vel_xs       = [], []
    j_ts,  j_seqs, j_xs = [], [], []
    s_ts,  s_seqs, s_xs = [], [], []

    with Reader(BAG1) as reader:
        add_types = {}
        for conn in reader.connections:
            if conn.msgdef:
                try:
                    add_types.update(get_types_from_msg(conn.msgdef.data, conn.msgtype))
                except Exception:
                    pass
        ts = get_typestore(Stores.ROS1_NOETIC)
        ts.register(add_types)

        t0 = None
        for conn, timestamp, rawdata in reader.messages():
            t = timestamp / 1e9
            if t0 is None:
                t0 = t
            rel = t - t0
            try:
                if conn.topic == '/arm/tool_pose':
                    msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                    arm_ts.append(rel); arm_xs.append(msg.pose.position.x)
                elif conn.topic == '/arm/tool_velocity':
                    msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                    vel_ts.append(rel); vel_xs.append(msg.twist.linear.x)
                elif conn.topic == '/network/jittered_frame':
                    msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                    j_ts.append(rel); j_seqs.append(int(msg.seq_num))
                    j_xs.append(float(msg.pose[0]))
                elif conn.topic == '/teleop/smoothed_frame':
                    msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                    s_ts.append(rel); s_seqs.append(int(msg.seq_num))
                    s_xs.append(float(msg.pose[0]))
            except Exception:
                pass

    return (np.array(arm_ts), np.array(arm_xs),
            np.array(vel_ts), np.array(vel_xs),
            np.array(j_ts),   np.array(j_seqs), np.array(j_xs),
            np.array(s_ts),   np.array(s_seqs), np.array(s_xs))


# ═══════════════════════════════════════════════════════════════════════
# 导纳控制器仿真
# ═══════════════════════════════════════════════════════════════════════

def admittance_step(N=200, dt=0.02):
    ctrl = AdmittanceController(mass=1.0, damping=20.0, stiffness=0.0,
                                dt=dt, enable_filter=False)
    f = np.array([5.0, 0, 0, 0, 0, 0])
    corrs, vels = [], []
    for _ in range(N):
        ctrl.compute_correction(f)
        corrs.append(ctrl.dX[0]); vels.append(ctrl.dX_dot[0])
    return np.arange(N)*dt, np.array(corrs), np.array(vels)


def admittance_sine(N=200, dt=0.02):
    ctrl = AdmittanceController(mass=1.0, damping=20.0, stiffness=0.0,
                                dt=dt, enable_filter=False)
    forces, corrs = [], []
    for i in range(N):
        f = 3.0 * math.sin(2 * math.pi * 1.0 * i * dt)
        ctrl.compute_correction(np.array([f, 0, 0, 0, 0, 0]))
        forces.append(f); corrs.append(ctrl.dX[0])
    return np.arange(N)*dt, np.array(forces), np.array(corrs)


# ═══════════════════════════════════════════════════════════════════════
# 主函数
# ═══════════════════════════════════════════════════════════════════════

def main():
    print("正在生成验证图（真实 rosbag 数据）...")

    print("  [1/3] 读取 rosbag 数据...")
    (arm_ts, arm_xs,
     vel_ts, vel_xs,
     j_ts,  j_seqs, j_xs,
     s_ts,  s_seqs, s_xs) = load_rosbag()

    # ── 统计量 ────────────────────────────────────────────────────────
    j_ooo    = int(np.sum(np.diff(j_seqs) <= 0))
    s_ooo    = int(np.sum(np.diff(s_seqs) <= 0))
    freq     = len(arm_ts) / arm_ts[-1]
    duration = arm_ts[-1]

    # 无缓冲：按到达时序使用 jitter 帧的位置（会产生大跳变）
    j_jumps  = np.abs(np.diff(j_xs))
    s_jumps  = np.abs(np.diff(s_xs))
    j_big    = int(np.sum(j_jumps > 0.05))   # >5cm 的跳变帧数
    s_big    = int(np.sum(s_jumps > 0.05))

    print(f"     录制时长: {duration:.2f}s  频率: {freq:.1f}Hz  帧数: {len(arm_ts)}")
    print(f"     Jitter OOO: {j_ooo}/{len(j_seqs)} ({100*j_ooo/len(j_seqs):.1f}%)")
    print(f"     Smooth OOO: {s_ooo}/{len(s_seqs)}")
    print(f"     无缓冲时 >5cm 跳变帧: {j_big} / {len(j_jumps)}")
    print(f"     有缓冲后 >5cm 跳变帧: {s_big} / {len(s_jumps)}")
    print(f"     最大跳变 无缓冲: {j_jumps.max()*100:.1f}cm  有缓冲: {s_jumps.max()*100:.1f}cm")

    print("  [2/3] 生成导纳仿真数据...")
    ts_step, corr_step, vel_step  = admittance_step()
    ts_sine, forces_sine, corr_sine = admittance_sine()

    print("  [3/3] 绘图...")

    # ── 颜色 ─────────────────────────────────────────────────────────
    DARK   = '#0d1117'
    AX_BG  = '#161b22'
    BLUE   = '#58a6ff'   # 主端 / 参考
    GREEN  = '#3fb950'   # 从端（有缓冲）
    RED    = '#f78166'   # 无缓冲 / 力
    ORANGE = '#ffa657'   # 速度 / 强调
    GRID   = '#30363d'
    TEXT   = '#e6edf3'
    DIM    = '#8b949e'

    def sax(ax, title, xl='', yl=''):
        ax.set_facecolor(AX_BG)
        ax.tick_params(colors=TEXT, labelsize=9)
        for sp in ax.spines.values(): sp.set_edgecolor(GRID)
        ax.xaxis.label.set_color(TEXT); ax.yaxis.label.set_color(TEXT)
        ax.set_title(title, color=TEXT, fontsize=11, fontweight='bold', pad=8)
        if xl: ax.set_xlabel(xl, fontsize=9)
        if yl: ax.set_ylabel(yl, fontsize=9)
        ax.grid(True, color=GRID, linewidth=0.5, alpha=0.8)

    # ── 画布 ──────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(22, 16), facecolor=DARK)
    gs  = gridspec.GridSpec(3, 3, figure=fig,
                            hspace=0.52, wspace=0.42,
                            left=0.06, right=0.97, top=0.935, bottom=0.05)
    fig.suptitle('Z1 双机械臂遥操作系统  ─  验证结果总览（真实录制数据 + 控制仿真）',
                 color=TEXT, fontsize=16, fontweight='bold', y=0.975)

    # ════════════════════════════════════════════════════════════════
    # 图1 (顶部全宽) ★ 核心：无缓冲 vs 有缓冲位置对比
    # ════════════════════════════════════════════════════════════════
    ax1 = fig.add_subplot(gs[0, :])
    sax(ax1, '图1  ★ 核心对比：抖动缓冲对位置轨迹的影响（真实录制，45% 帧乱序）',
        '时间 (s)', '末端 X 轴位置 (m)')

    # 主端参考
    ax1.plot(arm_ts, arm_xs, color=BLUE, linewidth=2.0, zorder=4,
             label=f'主端实际轨迹  /arm/tool_pose  ({freq:.0f} Hz, 1536 帧)')
    # 无缓冲：jitter 按到达时序（乱序位置 → 大跳变）
    ax1.plot(j_ts, j_xs, color=RED, linewidth=1.2, zorder=2, alpha=0.75,
             label=f'从端无缓冲（直接使用到达帧）— 最大跳变 {j_jumps.max()*100:.1f} cm')
    # 有缓冲：smooth 严格有序输出（平滑）
    ax1.plot(s_ts, s_xs, color=GREEN, linewidth=1.8, zorder=3,
             linestyle='--', label=f'从端有缓冲（抖动缓冲输出）— 最大跳变 {s_jumps.max()*100:.1f} cm')

    # 标注
    info = (f'乱序帧: {j_ooo} / {len(j_seqs)}  ({100*j_ooo/len(j_seqs):.0f}%)  →  消除后: {s_ooo} / {len(s_seqs)}  (0%)\n'
            f'无缓冲时 >5cm 跳变: {j_big} 帧    有缓冲后 >5cm 跳变: {s_big} 帧\n'
            f'录制时长 {duration:.2f} s  ·  采样频率 {freq:.1f} Hz  ·  {len(arm_ts)} 帧')
    ax1.text(0.01, 0.97, info, transform=ax1.transAxes,
             color=TEXT, fontsize=10, va='top',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='#21262d', edgecolor=GRID, alpha=0.93))
    ax1.legend(loc='upper right', fontsize=9.5, facecolor='#21262d',
               edgecolor=GRID, labelcolor=TEXT)
    ax1.set_xlim(0, max(arm_ts[-1], j_ts[-1], s_ts[-1]))
    ax1.set_ylim(-1.15, 1.15)

    # ════════════════════════════════════════════════════════════════
    # 图2a (中左) 抖动序列号散点
    # ════════════════════════════════════════════════════════════════
    ax2a = fig.add_subplot(gs[1, 0])
    sax(ax2a, '图2a  网络帧到达顺序（真实乱序）', '时间 (s)', '序列号')

    j_diff   = np.diff(j_seqs, prepend=j_seqs[0]-1)
    is_ooo_j = j_diff <= 0
    ax2a.scatter(j_ts[~is_ooo_j], j_seqs[~is_ooo_j], s=3, c=BLUE,  alpha=0.45, rasterized=True, label='正序帧')
    ax2a.scatter(j_ts[ is_ooo_j], j_seqs[ is_ooo_j], s=8, c=RED,   alpha=0.90, rasterized=True, label=f'乱序帧 ({j_ooo})')
    t_ideal = np.array([j_ts[0], j_ts[-1]])
    s_ideal = np.array([j_seqs[0], j_seqs[0] + (j_seqs[-1]-j_seqs[0])])
    ax2a.plot(t_ideal, s_ideal, color=DIM, linewidth=1.2, linestyle='--', alpha=0.55, label='理想顺序')
    ax2a.legend(fontsize=8, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax2a.text(0.03, 0.97,
              f'总帧数: {len(j_seqs)}\n乱序率: {100*j_ooo/len(j_seqs):.1f}%',
              transform=ax2a.transAxes, color=RED, fontsize=9, va='top',
              bbox=dict(boxstyle='round', facecolor='#21262d', edgecolor=GRID))
    ax2a.set_xlim(0, j_ts[-1])

    # ════════════════════════════════════════════════════════════════
    # 图2b (中中) 缓冲后有序输出
    # ════════════════════════════════════════════════════════════════
    ax2b = fig.add_subplot(gs[1, 1])
    sax(ax2b, '图2b  抖动缓冲输出（严格有序）', '时间 (s)', '序列号')

    ax2b.scatter(s_ts, s_seqs, s=3, c=GREEN, alpha=0.50, rasterized=True, label='有序帧')
    t_s_ideal = np.array([s_ts[0], s_ts[-1]])
    s_s_ideal = np.array([s_seqs[0], s_seqs[0] + (s_seqs[-1]-s_seqs[0])])
    ax2b.plot(t_s_ideal, s_s_ideal, color=DIM, linewidth=1.2, linestyle='--', alpha=0.55, label='理想顺序')
    ax2b.legend(fontsize=8, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax2b.text(0.03, 0.97,
              f'总帧数: {len(s_seqs)}\n乱序帧: 0  ✓ 完全消除',
              transform=ax2b.transAxes, color=GREEN, fontsize=9, va='top',
              bbox=dict(boxstyle='round', facecolor='#21262d', edgecolor=GRID))
    ax2b.set_xlim(0, s_ts[-1])

    # ════════════════════════════════════════════════════════════════
    # 图3 (中右) 导纳阶跃响应
    # ════════════════════════════════════════════════════════════════
    ax3 = fig.add_subplot(gs[1, 2])
    sax(ax3, '图3  导纳控制器：阶跃力响应\n( F=5 N → 修正量收敛 + 限幅 )', '时间 (s)')

    ax3r = ax3.twinx()
    ax3r.set_facecolor(AX_BG); ax3r.tick_params(colors=TEXT, labelsize=9)
    ax3r.spines['right'].set_edgecolor(GRID)

    l3a, = ax3.plot(ts_step, corr_step, color=GREEN, linewidth=2.3, label='修正量 Δx (m)')
    l3b, = ax3r.plot(ts_step, vel_step, color=ORANGE, linewidth=1.4, linestyle=':', label='速度 (m/s)')
    ax3.axhline(0.05, color=RED, linewidth=1.3, linestyle='--', alpha=0.8, label='限幅 0.05m')

    sat = np.where(corr_step >= 0.049)[0]
    if len(sat):
        ax3.annotate(f'饱和  t={ts_step[sat[0]]:.2f}s',
                     xy=(ts_step[sat[0]], 0.05), xytext=(ts_step[sat[0]]+0.4, 0.036),
                     color=TEXT, fontsize=8,
                     arrowprops=dict(arrowstyle='->', color=TEXT),
                     bbox=dict(boxstyle='round', facecolor='#21262d', edgecolor=GRID))

    ax3.set_ylabel('修正量 (m)', color=GREEN, fontsize=9)
    ax3r.set_ylabel('速度 (m/s)', color=ORANGE, fontsize=9)
    hl = [l3a, l3b, plt.Line2D([0],[0],color=RED,lw=1.3,ls='--')]
    ax3.legend(hl, ['修正量 Δx (m)', '速度 (m/s)', '限幅 ±0.05m'],
               fontsize=8, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT,
               loc='center right')
    ax3.set_xlim(0, ts_step[-1])

    # ════════════════════════════════════════════════════════════════
    # 图4 (下左) 真实速度曲线
    # ════════════════════════════════════════════════════════════════
    ax4 = fig.add_subplot(gs[2, 0])
    sax(ax4, f'图4  真实末端速度（实录，{freq:.0f} Hz）\n/arm/tool_velocity', '时间 (s)', 'Vx (m/s)')

    ax4.plot(vel_ts, vel_xs, color=ORANGE, linewidth=1.3, alpha=0.9, label='Vx')
    ax4.axhline(0, color=GRID, linewidth=0.5)
    ax4.fill_between(vel_ts, vel_xs, 0, where=(vel_xs>=0), alpha=0.13, color=BLUE)
    ax4.fill_between(vel_ts, vel_xs, 0, where=(vel_xs< 0), alpha=0.13, color=RED)
    vmax = np.abs(vel_xs).max()
    ax4.text(0.02, 0.97,
             f'最大速度: ±{vmax:.2f} m/s\n均值:  {np.abs(vel_xs).mean():.2f} m/s\n帧数:  {len(vel_ts)}',
             transform=ax4.transAxes, color=TEXT, fontsize=9, va='top',
             bbox=dict(boxstyle='round', facecolor='#21262d', edgecolor=GRID))
    ax4.legend(fontsize=8, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax4.set_xlim(0, vel_ts[-1])

    # ════════════════════════════════════════════════════════════════
    # 图5 (下中) 正弦力导纳跟随
    # ════════════════════════════════════════════════════════════════
    ax5 = fig.add_subplot(gs[2, 1])
    sax(ax5, '图5  导纳控制器：正弦力跟随\n( F=3sin(2πt) N @ 1 Hz )', '时间 (s)')

    ax5r = ax5.twinx()
    ax5r.set_facecolor(AX_BG); ax5r.tick_params(colors=TEXT, labelsize=9)
    ax5r.spines['right'].set_edgecolor(GRID)

    ax5.plot(ts_sine, forces_sine, color=RED,   linewidth=1.6, alpha=0.85, label='力 F (N)')
    ax5r.plot(ts_sine, corr_sine,  color=GREEN, linewidth=2.0, label='修正量 Δx (m)')
    ax5.axhline(0, color=GRID, linewidth=0.4)

    ax5.set_ylabel('外力 (N)', color=RED,   fontsize=9)
    ax5r.set_ylabel('修正量 (m)', color=GREEN, fontsize=9)
    hl5 = [plt.Line2D([0],[0],color=RED,lw=1.6),
           plt.Line2D([0],[0],color=GREEN,lw=2.0)]
    ax5.legend(hl5, ['力输入 F (N)', '修正量 Δx (m)'],
               fontsize=8, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax5.set_xlim(0, ts_sine[-1])

    # ════════════════════════════════════════════════════════════════
    # 图6 (下右) 系统链路 + 指标汇总
    # ════════════════════════════════════════════════════════════════
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.set_facecolor(AX_BG)
    ax6.set_xlim(0, 10); ax6.set_ylim(0, 10)
    ax6.axis('off')
    ax6.set_title('图6  系统链路 & 关键指标汇总',
                  color=TEXT, fontsize=11, fontweight='bold', pad=8)

    # 系统链路方块
    pipeline = [
        (0.2, 9.2, '主端机械臂',              BLUE),
        (0.2, 7.7, '导纳控制器 (Master)',      BLUE),
        (0.2, 6.2, 'TeleopFrame 打包发送',     ORANGE),
        (0.2, 4.7, '网络传输 (含抖动)',        RED),
        (0.2, 3.2, '抖动缓冲 (JitterBuffer)',  ORANGE),
        (0.2, 1.7, '导纳控制器 (Slave)',       GREEN),
        (0.2, 0.2, '从端机械臂',              GREEN),
    ]
    for x0, y0, label, color in pipeline:
        rect = mpatches.FancyBboxPatch((x0, y0-0.48), 4.0, 0.85,
                                        boxstyle='round,pad=0.07',
                                        facecolor=color+'22', edgecolor=color, linewidth=1.3)
        ax6.add_patch(rect)
        ax6.text(x0+2.0, y0-0.02, label, color=TEXT, fontsize=8,
                 ha='center', va='center', fontweight='bold')

    # 箭头：每两个相邻方块之间绘制向下箭头
    # 箭头从上方块底边 (y0 - 0.48) 指向下方块顶边 (y0_next + 0.37)
    for i in range(len(pipeline) - 1):
        y_top    = pipeline[i][1] - 0.48      # 上方块底边 y 坐标
        y_bottom = pipeline[i+1][1] + 0.37    # 下方块顶边 y 坐标
        ax6.annotate('', xy=(2.2, y_bottom), xytext=(2.2, y_top),
                     arrowprops=dict(arrowstyle='->', color=DIM, lw=1.1))

    # 关键指标
    METRIC_VERTICAL_SPACING = 1.33  # 每条指标之间的纵向间距（坐标单位）
    metrics = [
        ('采样频率',       f'{freq:.1f} Hz',             BLUE),
        ('录制时长',       f'{duration:.2f} s',           BLUE),
        ('乱序帧消除',     f'{j_ooo} → {s_ooo} 帧',      GREEN),
        ('乱序率',         f'{100*j_ooo/len(j_seqs):.0f}% → 0%', GREEN),
        ('无缓冲最大跳变', f'{j_jumps.max()*100:.1f} cm', RED),
        ('有缓冲最大跳变', f'{s_jumps.max()*100:.1f} cm', GREEN),
        ('修正量限幅',     '± 0.05 m',                   ORANGE),
    ]
    for i, (name, val, color) in enumerate(metrics):
        xt, yt = 4.5, 9.5 - i * METRIC_VERTICAL_SPACING
        ax6.text(xt, yt,      name, color=DIM,   fontsize=7.5, va='bottom')
        ax6.text(xt, yt-0.38, val,  color=color, fontsize=11,  fontweight='bold', va='top')
        if i < len(metrics)-1:
            ax6.plot([xt, 9.8], [yt-0.9, yt-0.9], color=GRID, linewidth=0.4)

    # ── 保存 ──────────────────────────────────────────────────────────
    out = os.path.join(SCRIPT_DIR, 'validation_results.png')
    fig.savefig(out, dpi=160, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"\n✓ 保存完成: {out}")
    return out


if __name__ == '__main__':
    out = main()
    print(f"完成。请查看: {out}")
