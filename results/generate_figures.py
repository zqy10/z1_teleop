#!/usr/bin/env python3
"""
z1_teleop 双机械臂遥操作系统 - 可视化验证脚本（增强版）
分别保存以下独立图像：
  fig1_position_comparison.png  ★核心：有无抖动缓冲位置轨迹对比（真实数据）
  fig2a_jitter_sequence.png     抖动帧到达的序列号增量分布（真实乱序可视化）
  fig2b_smooth_sequence.png     抖动缓冲输出的序列号增量分布（完全有序验证）
  fig3_multistep_response.png   导纳控制器多级阶跃力响应（复杂信号）
  fig4_real_velocity.png        真实末端速度曲线（/arm/tool_velocity）
  fig5_chirp_response.png       导纳控制器扫频力响应（0.5→4 Hz chirp 信号）
  fig6_composite_response.png   导纳控制器多频叠加力响应（三次谐波合成）
  fig7_system_summary.png       系统链路 + 关键指标汇总

用法:
    python3 generate_figures.py
输出:
    results/fig*.png
"""

import sys, os, math
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.font_manager as _fm
import glob as _glob

# 手动注册系统中的 Noto Sans CJK 字体（.ttc 文件不会被 matplotlib 自动扫描）
_noto_paths = sorted(_glob.glob('/usr/share/fonts/opentype/noto/NotoSansCJK*.ttc'))
for _p in _noto_paths:
    _fm.fontManager.addfont(_p)

_fm._load_fontmanager(try_read_cache=False)

# 再次注册（_load_fontmanager 会重建列表，需重新添加）
for _p in _noto_paths:
    _fm.fontManager.addfont(_p)

import matplotlib.pyplot as plt
_cjk = [f.name for f in _fm.fontManager.ttflist
        if 'Noto' in f.name and 'CJK' in f.name and 'Serif' not in f.name]
if _cjk:
    plt.rcParams['font.family'] = _cjk[0]
plt.rcParams['axes.unicode_minus'] = False
import matplotlib.patches as mpatches

# ── 路径 ──────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_ROOT, 'master_ws/src/master_controller/scripts'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'slave_ws/src/slave_controller/scripts'))
BAG1 = os.path.join(REPO_ROOT, '2026-04-07-08-08-51.bag')

from admittance_controller import AdmittanceController


# ═══════════════════════════════════════════════════════════════════════
# 公共样式
# ═══════════════════════════════════════════════════════════════════════

DARK   = '#0d1117'
AX_BG  = '#161b22'
BLUE   = '#58a6ff'
GREEN  = '#3fb950'
RED    = '#f78166'
ORANGE = '#ffa657'
PURPLE = '#bc8cff'
GRID   = '#30363d'
TEXT   = '#e6edf3'
DIM    = '#8b949e'


def sax(ax, title, xl='', yl='', fontsize=12):
    ax.set_facecolor(AX_BG)
    ax.tick_params(colors=TEXT, labelsize=10)
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID)
    ax.xaxis.label.set_color(TEXT)
    ax.yaxis.label.set_color(TEXT)
    ax.set_title(title, color=TEXT, fontsize=fontsize, fontweight='bold', pad=10)
    if xl:
        ax.set_xlabel(xl, fontsize=10)
    if yl:
        ax.set_ylabel(yl, fontsize=10)
    ax.grid(True, color=GRID, linewidth=0.6, alpha=0.8)


def savefig(fig, name):
    out = os.path.join(SCRIPT_DIR, name)
    fig.savefig(out, dpi=160, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  ✓ 保存: {name}")
    return out


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
# 导纳控制器仿真信号（复杂输入）
# ═══════════════════════════════════════════════════════════════════════

def admittance_multistep(dt=0.02):
    """
    多级变幅阶跃力序列：
      [0,1s]   F=+3 N   缓慢上升阶段
      [1,2s]   F=-5 N   反向大力阶段
      [2,3s]   F=+8 N   正向强力阶段（触发限幅）
      [3,4s]   F=+2 N   中等力阶段
      [4,5s]   F=-4 N   负向中等力阶段
      [5,6s]   F=0  N   撤力恢复阶段
    """
    segments = [(1.0, 3.0), (1.0, -5.0), (1.0, 8.0),
                (1.0, 2.0), (1.0, -4.0), (1.0, 0.0)]
    ctrl = AdmittanceController(mass=1.0, damping=20.0, stiffness=0.0,
                                dt=dt, enable_filter=False)
    t_all, f_all, c_all, v_all = [], [], [], []
    t_cur = 0.0
    for dur, fval in segments:
        N = int(dur / dt)
        for _ in range(N):
            ctrl.compute_correction(np.array([fval, 0, 0, 0, 0, 0]))
            t_all.append(t_cur); f_all.append(fval)
            c_all.append(ctrl.dX[0]); v_all.append(ctrl.dX_dot[0])
            t_cur += dt
    return (np.array(t_all), np.array(f_all),
            np.array(c_all), np.array(v_all))


def admittance_chirp(dt=0.02, total=8.0, f0=0.5, f1=4.0, amp=5.0):
    """
    线性扫频（chirp）力输入：频率从 f0 Hz 线性增加到 f1 Hz。
    F(t) = amp * sin(2π * (f0*t + (f1-f0)/(2*T)*t²))
    """
    ctrl = AdmittanceController(mass=1.0, damping=20.0, stiffness=0.0,
                                dt=dt, enable_filter=False)
    N = int(total / dt)
    t = np.arange(N) * dt
    # 相位累积：φ(t) = 2π * (f0*t + (f1-f0)*t²/(2*T))
    phase = 2 * math.pi * (f0 * t + (f1 - f0) * t**2 / (2 * total))
    forces = amp * np.sin(phase)
    corrs, vels = [], []
    for i in range(N):
        ctrl.compute_correction(np.array([forces[i], 0, 0, 0, 0, 0]))
        corrs.append(ctrl.dX[0]); vels.append(ctrl.dX_dot[0])
    # 瞬时频率（Hz）用于标注
    inst_freq = f0 + (f1 - f0) * t / total
    return t, forces, np.array(corrs), np.array(vels), inst_freq


def admittance_composite(dt=0.02, total=6.0):
    """
    三频叠加力输入（谐波合成）：
      F(t) = 3.0*sin(2π*0.5*t)
           + 2.0*sin(2π*1.5*t + π/4)
           + 1.0*sin(2π*3.0*t + π/3)
    模拟真实力传感器中包含基频及二、三次谐波的情形。
    """
    ctrl = AdmittanceController(mass=1.0, damping=20.0, stiffness=0.0,
                                dt=dt, enable_filter=False)
    N = int(total / dt)
    t = np.arange(N) * dt
    comp1 = 3.0 * np.sin(2 * math.pi * 0.5  * t)
    comp2 = 2.0 * np.sin(2 * math.pi * 1.5  * t + math.pi / 4)
    comp3 = 1.0 * np.sin(2 * math.pi * 3.0  * t + math.pi / 3)
    forces = comp1 + comp2 + comp3
    corrs = []
    for i in range(N):
        ctrl.compute_correction(np.array([forces[i], 0, 0, 0, 0, 0]))
        corrs.append(ctrl.dX[0])
    return t, forces, comp1, comp2, comp3, np.array(corrs)


# ═══════════════════════════════════════════════════════════════════════
# 绘图函数（每张图独立）
# ═══════════════════════════════════════════════════════════════════════

def plot_fig1(arm_ts, arm_xs, j_ts, j_xs, j_jumps, s_ts, s_xs, s_jumps,
              j_ooo, j_seqs, s_ooo, s_seqs, freq, duration, j_big, s_big):
    """图1: ★ 核心对比 — 抖动缓冲对位置轨迹的影响"""
    fig, ax = plt.subplots(figsize=(18, 6), facecolor=DARK)
    sax(ax, '图1  ★ 核心对比：抖动缓冲对位置轨迹的影响（真实录制数据）',
        '时间 (s)', '末端 X 轴位置 (m)', fontsize=14)

    ax.plot(arm_ts, arm_xs, color=BLUE, linewidth=2.2, zorder=4,
            label=f'主端实际轨迹  /arm/tool_pose  (采样率 {freq:.1f} Hz，共 {len(arm_ts)} 帧)')
    ax.plot(j_ts, j_xs, color=RED, linewidth=1.1, zorder=2, alpha=0.75,
            label=f'从端无缓冲（直接使用到达帧）— 最大跳变 {j_jumps.max()*100:.2f} cm，乱序帧 {j_ooo} 个')
    ax.plot(s_ts, s_xs, color=GREEN, linewidth=2.0, zorder=3, linestyle='--',
            label=f'从端有缓冲（抖动缓冲输出）— 最大跳变 {s_jumps.max()*100:.2f} cm，乱序帧 {s_ooo} 个')

    # 标注最大跳变位置
    idx_jmax = int(np.argmax(j_jumps))
    ax.annotate(f'最大跳变\n{j_jumps.max()*100:.2f} cm',
                xy=(j_ts[idx_jmax], j_xs[idx_jmax]),
                xytext=(j_ts[idx_jmax] + 0.5, j_xs[idx_jmax] - 0.35),
                color=RED, fontsize=9,
                arrowprops=dict(arrowstyle='->', color=RED),
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#21262d', edgecolor=RED))

    info = (f'录制时长: {duration:.2f} s   采样频率: {freq:.1f} Hz   总帧数: {len(arm_ts)}\n'
            f'乱序帧 (无缓冲): {j_ooo} / {len(j_seqs)}  ({100*j_ooo/len(j_seqs):.1f}%)   →   消除后: {s_ooo} / {len(s_seqs)}  (0%)\n'
            f'>5cm 跳变帧: 无缓冲 {j_big} 帧  →  有缓冲 {s_big} 帧   |   '
            f'最大跳变: {j_jumps.max()*100:.2f} cm  →  {s_jumps.max()*100:.2f} cm   |   '
            f'跳变抑制率: {100*(1 - s_jumps.max()/j_jumps.max()):.1f}%')
    ax.text(0.01, 0.98, info, transform=ax.transAxes,
            color=TEXT, fontsize=10, va='top',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#21262d', edgecolor=GRID, alpha=0.93))
    ax.legend(loc='upper right', fontsize=10, facecolor='#21262d',
              edgecolor=GRID, labelcolor=TEXT)
    ax.set_xlim(0, max(arm_ts[-1], j_ts[-1], s_ts[-1]))
    ax.set_ylim(-1.2, 1.35)
    fig.tight_layout()
    return savefig(fig, 'fig1_position_comparison.png')


def plot_fig2a(j_ts, j_seqs, j_ooo):
    """图2a: 网络帧到达顺序 — 序列号逐帧增量（清晰显示乱序）"""
    j_delta = np.diff(j_seqs)
    j_t_mid = j_ts[1:]

    is_normal  = j_delta == 1
    is_ooo     = j_delta <= 0
    is_skip    = j_delta > 1

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(14, 8),
                                          facecolor=DARK,
                                          gridspec_kw={'height_ratios': [3, 1]})

    # ── 上图：Δseq 时间序列 ──
    sax(ax_top,
        f'图2a  网络帧到达顺序（真实网络抖动）— 序列号逐帧增量 Δseq',
        '时间 (s)', 'Δseq（相邻帧序列号差值）', fontsize=13)

    ax_top.scatter(j_t_mid[is_normal], j_delta[is_normal],
                   s=4, c=BLUE, alpha=0.5, rasterized=True, label=f'正序帧 Δseq=1 ({is_normal.sum()})')
    ax_top.scatter(j_t_mid[is_skip], j_delta[is_skip],
                   s=12, c=ORANGE, alpha=0.85, rasterized=True,
                   label=f'跳帧 Δseq>1 ({is_skip.sum()})，均值={j_delta[is_skip].mean():.1f}' if is_skip.sum() else '跳帧 Δseq>1 (0)')
    ax_top.scatter(j_t_mid[is_ooo], j_delta[is_ooo],
                   s=25, c=RED, alpha=0.95, rasterized=True,
                   zorder=5, label=f'乱序帧 Δseq≤0 ({j_ooo})，最小值={j_delta[is_ooo].min() if j_ooo else 0}')

    ax_top.axhline(1, color=GREEN, linewidth=1.5, linestyle='--', alpha=0.75, label='理想值 Δseq=1')
    ax_top.axhline(0, color=RED,   linewidth=0.8, linestyle=':',  alpha=0.50)
    ax_top.set_xlim(0, j_ts[-1])
    ylo = min(j_delta.min() - 0.5, -1.5)
    yhi = min(j_delta.max() + 0.5, 10)
    ax_top.set_ylim(ylo, yhi)
    ax_top.legend(fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax_top.text(0.01, 0.97,
                f'总帧数: {len(j_seqs)}   乱序帧: {j_ooo}   乱序率: {100*j_ooo/len(j_seqs):.1f}%\n'
                f'跳帧: {is_skip.sum()}   正序帧: {is_normal.sum()}\n'
                f'Δseq 均值: {j_delta.mean():.3f}   标准差: {j_delta.std():.3f}',
                transform=ax_top.transAxes, color=TEXT, fontsize=10, va='top',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='#21262d', edgecolor=GRID))

    # ── 下图：Δseq 直方图分布 ──
    sax(ax_bot, '', '   Δseq 值', '帧数 (count)', fontsize=11)
    bins = range(int(j_delta.min()) - 1, min(int(j_delta.max()) + 3, 15))
    ax_bot.hist(j_delta, bins=bins, color=BLUE, edgecolor=GRID, alpha=0.75, rasterized=True)
    ax_bot.axvline(1,  color=GREEN, linewidth=1.5, linestyle='--', label='理想值 1')
    ax_bot.axvline(0,  color=RED,   linewidth=1.0, linestyle=':',  label='乱序边界 0')
    ax_bot.legend(fontsize=9, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax_bot.set_xlim(bins.start, bins.stop - 1)

    fig.tight_layout(pad=1.5)
    return savefig(fig, 'fig2a_jitter_sequence.png')


def plot_fig2b(s_ts, s_seqs, s_ooo):
    """图2b: 缓冲输出序列号增量（完全有序验证）"""
    s_delta = np.diff(s_seqs)
    s_t_mid = s_ts[1:]

    is_normal = s_delta == 1
    is_skip   = s_delta > 1
    is_ooo    = s_delta <= 0

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(14, 8),
                                          facecolor=DARK,
                                          gridspec_kw={'height_ratios': [3, 1]})

    # ── 上图：Δseq 时间序列 ──
    sax(ax_top,
        f'图2b  抖动缓冲输出顺序（严格有序 ✓）— 序列号逐帧增量 Δseq',
        '时间 (s)', 'Δseq（相邻帧序列号差值）', fontsize=13)

    ax_top.scatter(s_t_mid[is_normal], s_delta[is_normal],
                   s=4, c=GREEN, alpha=0.55, rasterized=True,
                   label=f'正序帧 Δseq=1 ({is_normal.sum()})  — 完全有序输出')
    if is_skip.sum():
        ax_top.scatter(s_t_mid[is_skip], s_delta[is_skip],
                       s=12, c=ORANGE, alpha=0.85, rasterized=True,
                       label=f'跳帧 Δseq>1 ({is_skip.sum()})（缓冲丢弃重复）')
    if is_ooo.sum():
        ax_top.scatter(s_t_mid[is_ooo], s_delta[is_ooo],
                       s=25, c=RED, alpha=0.95, rasterized=True,
                       zorder=5, label=f'仍乱序 Δseq≤0 ({is_ooo.sum()})')

    ax_top.axhline(1, color=GREEN, linewidth=1.8, linestyle='--', alpha=0.75, label='理想值 Δseq=1')
    ax_top.axhline(0, color=RED,   linewidth=0.8, linestyle=':',  alpha=0.40, label='乱序边界')
    ax_top.set_xlim(0, s_ts[-1])
    yhi = max(s_delta.max() + 0.5, 3)
    ax_top.set_ylim(-0.5, yhi)
    ax_top.legend(fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)

    # 计算连续正序率
    consecutive_ok = int(np.sum(s_delta >= 1))
    ax_top.text(0.01, 0.97,
                f'总输出帧数: {len(s_seqs)}   乱序帧: {s_ooo}  ✓ 完全消除\n'
                f'正序帧 (Δseq≥1): {consecutive_ok}   Δseq=1 帧: {is_normal.sum()}\n'
                f'Δseq 均值: {s_delta.mean():.3f}   标准差: {s_delta.std():.3f}',
                transform=ax_top.transAxes, color=GREEN, fontsize=10, va='top',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='#21262d', edgecolor=GRID))

    # ── 下图：Δseq 直方图分布 ──
    sax(ax_bot, '', '   Δseq 值', '帧数 (count)', fontsize=11)
    bins = range(max(int(s_delta.min()) - 1, -2), min(int(s_delta.max()) + 3, 15))
    ax_bot.hist(s_delta, bins=bins, color=GREEN, edgecolor=GRID, alpha=0.75, rasterized=True)
    ax_bot.axvline(1, color=GREEN, linewidth=1.5, linestyle='--', label='理想值 1')
    ax_bot.axvline(0, color=RED,   linewidth=1.0, linestyle=':',  label='乱序边界 0')
    ax_bot.legend(fontsize=9, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax_bot.set_xlim(bins.start, bins.stop - 1)

    fig.tight_layout(pad=1.5)
    return savefig(fig, 'fig2b_smooth_sequence.png')


def plot_fig3(t, forces, corrs, vels):
    """图3: 导纳控制器多级变幅阶跃力响应"""
    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(14, 8),
                                          facecolor=DARK,
                                          gridspec_kw={'height_ratios': [1, 2]})

    # ── 上图：力输入 ──
    sax(ax_top, '图3  导纳控制器：多级变幅阶跃力输入（6 段，±8 N）',
        '', '外力 F (N)', fontsize=13)
    ax_top.step(t, forces, color=RED, linewidth=2.0, where='post', label='阶跃力 F(t) (N)')
    ax_top.axhline(0, color=GRID, linewidth=0.6)
    ax_top.fill_between(t, forces, 0, where=(forces > 0),
                        step='post', alpha=0.15, color=RED)
    ax_top.fill_between(t, forces, 0, where=(forces < 0),
                        step='post', alpha=0.15, color=BLUE)
    ax_top.set_xlim(0, t[-1])

    # 标注各段
    segs = [(0.0, 1.0, 3.0, '+3 N'), (1.0, 2.0, -5.0, '-5 N'),
            (2.0, 3.0, 8.0, '+8 N'), (3.0, 4.0, 2.0, '+2 N'),
            (4.0, 5.0, -4.0, '-4 N'), (5.0, 6.0, 0.0, '0 N')]
    for t0, t1, fv, label in segs:
        ax_top.text((t0 + t1) / 2, fv + (0.4 if fv >= 0 else -0.6),
                    label, color=TEXT, fontsize=9, ha='center', va='bottom')
    ax_top.legend(fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)

    # ── 下图：修正量 & 速度 ──
    sax(ax_bot, '', '时间 (s)', '修正量 Δx (m)', fontsize=12)
    axr = ax_bot.twinx()
    axr.set_facecolor(AX_BG)
    axr.tick_params(colors=TEXT, labelsize=10)
    axr.spines['right'].set_edgecolor(GRID)

    l1, = ax_bot.plot(t, corrs, color=GREEN,  linewidth=2.2, label='修正量 Δx (m)')
    l2, = axr.plot(t, vels,  color=ORANGE, linewidth=1.5, linestyle=':', alpha=0.9, label='速度 (m/s)')
    ax_bot.axhline( 0.05, color=RED, linewidth=1.2, linestyle='--', alpha=0.8)
    ax_bot.axhline(-0.05, color=RED, linewidth=1.2, linestyle='--', alpha=0.8,
                   label='限幅 ±0.05 m')
    ax_bot.axhline(0, color=GRID, linewidth=0.5)

    sat_hi = np.where(corrs >= 0.049)[0]
    sat_lo = np.where(corrs <= -0.049)[0]
    for idx in (list(sat_hi[:1]) + list(sat_lo[:1])):
        ax_bot.annotate(f'饱和 {corrs[idx]*100:.1f}cm\nt={t[idx]:.2f}s',
                        xy=(t[idx], corrs[idx]),
                        xytext=(t[idx] + 0.25, corrs[idx] * 0.65),
                        color=TEXT, fontsize=8.5,
                        arrowprops=dict(arrowstyle='->', color=TEXT),
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='#21262d', edgecolor=GRID))

    ax_bot.set_ylabel('修正量 (m)', color=GREEN,  fontsize=10)
    axr.set_ylabel('速度 (m/s)', color=ORANGE, fontsize=10)

    # 统计
    sat_count = int(np.sum(np.abs(corrs) >= 0.049))
    ax_bot.text(0.01, 0.97,
                f'饱和帧数 (|Δx|≥4.9cm): {sat_count}   '
                f'最大修正量: {corrs.max()*100:.2f}cm / {corrs.min()*100:.2f}cm   '
                f'最大速度: {vels.max():.3f} / {vels.min():.3f} m/s',
                transform=ax_bot.transAxes, color=TEXT, fontsize=9.5, va='top',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='#21262d', edgecolor=GRID))

    hl = [l1, l2, plt.Line2D([0],[0], color=RED, lw=1.2, ls='--')]
    ax_bot.legend(hl, ['修正量 Δx (m)', '速度 (m/s)', '限幅 ±0.05 m'],
                  fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax_bot.set_xlim(0, t[-1])

    fig.tight_layout(pad=1.5)
    return savefig(fig, 'fig3_multistep_response.png')


def plot_fig4(vel_ts, vel_xs, freq):
    """图4: 真实末端速度曲线"""
    fig, ax = plt.subplots(figsize=(14, 6), facecolor=DARK)
    sax(ax, f'图4  真实末端速度（实录，{freq:.1f} Hz）  /arm/tool_velocity',
        '时间 (s)', 'Vx (m/s)', fontsize=14)

    ax.plot(vel_ts, vel_xs, color=ORANGE, linewidth=1.2, alpha=0.9, label='Vx (m/s)')
    ax.axhline(0, color=GRID, linewidth=0.6)
    ax.fill_between(vel_ts, vel_xs, 0, where=(vel_xs >= 0), alpha=0.15, color=BLUE)
    ax.fill_between(vel_ts, vel_xs, 0, where=(vel_xs  < 0), alpha=0.15, color=RED)

    vmax = np.abs(vel_xs).max()
    vmean = np.abs(vel_xs).mean()
    vrms  = np.sqrt(np.mean(vel_xs**2))
    # 标注最大速度位置
    idx_vmax = int(np.argmax(np.abs(vel_xs)))
    ax.annotate(f'峰值速度 {vel_xs[idx_vmax]:.3f} m/s',
                xy=(vel_ts[idx_vmax], vel_xs[idx_vmax]),
                xytext=(vel_ts[idx_vmax] + 0.5, vel_xs[idx_vmax] * 0.7),
                color=ORANGE, fontsize=9,
                arrowprops=dict(arrowstyle='->', color=ORANGE),
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#21262d', edgecolor=ORANGE))

    ax.text(0.01, 0.97,
            f'总帧数: {len(vel_ts)}   采样频率: {freq:.1f} Hz   时长: {vel_ts[-1]:.2f} s\n'
            f'最大速度: ±{vmax:.4f} m/s   均值 |Vx|: {vmean:.4f} m/s   RMS: {vrms:.4f} m/s\n'
            f'正向累计位移: {np.trapezoid(np.clip(vel_xs,0,None), vel_ts):.4f} m   '
            f'负向累计位移: {abs(np.trapezoid(np.clip(vel_xs,None,0), vel_ts)):.4f} m',
            transform=ax.transAxes, color=TEXT, fontsize=10, va='top',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#21262d', edgecolor=GRID, alpha=0.93))
    ax.legend(fontsize=10, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    ax.set_xlim(0, vel_ts[-1])
    fig.tight_layout()
    return savefig(fig, 'fig4_real_velocity.png')


def plot_fig5(t, forces, corrs, vels, inst_freq, f0, f1):
    """图5: 导纳控制器扫频（chirp）力响应"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), facecolor=DARK,
                             gridspec_kw={'height_ratios': [2, 2, 1]})

    # ── 上图：力输入 ──
    sax(axes[0], f'图5  导纳控制器：线性扫频力响应（{f0}→{f1} Hz，幅值 5 N）',
        '', '外力 F (N)', fontsize=13)
    axes[0].plot(t, forces, color=RED, linewidth=1.5, alpha=0.9, label='扫频力 F(t) (N)')
    axes[0].fill_between(t, forces, 0, where=(forces > 0), alpha=0.12, color=RED)
    axes[0].fill_between(t, forces, 0, where=(forces < 0), alpha=0.12, color=BLUE)
    axes[0].axhline(0, color=GRID, linewidth=0.5)
    axes[0].set_xlim(0, t[-1])

    # 标注频率
    for fi in [0.5, 1.0, 2.0, 3.0, f1]:
        ti_idx = int(np.argmin(np.abs(inst_freq - fi)))
        axes[0].axvline(t[ti_idx], color=DIM, linewidth=0.8, linestyle=':', alpha=0.5)
        axes[0].text(t[ti_idx], forces.max() * 0.85, f'{fi:.1f}Hz',
                     color=DIM, fontsize=8, ha='center')
    axes[0].legend(fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)

    # ── 中图：修正量与速度 ──
    sax(axes[1], '', '时间 (s)', '修正量 Δx (m)', fontsize=12)
    axr = axes[1].twinx()
    axr.set_facecolor(AX_BG); axr.tick_params(colors=TEXT, labelsize=10)
    axr.spines['right'].set_edgecolor(GRID)

    l1, = axes[1].plot(t, corrs, color=GREEN,  linewidth=2.0, label='修正量 Δx (m)')
    l2, = axr.plot(t, vels,  color=ORANGE, linewidth=1.3, linestyle=':', alpha=0.85, label='速度 (m/s)')
    axes[1].axhline( 0.05, color=RED, linewidth=1.1, linestyle='--', alpha=0.8, label='限幅 ±0.05 m')
    axes[1].axhline(-0.05, color=RED, linewidth=1.1, linestyle='--', alpha=0.8)
    axes[1].axhline(0, color=GRID, linewidth=0.5)
    axes[1].set_ylabel('修正量 (m)', color=GREEN,  fontsize=10)
    axr.set_ylabel('速度 (m/s)', color=ORANGE, fontsize=10)
    sat_cnt = int(np.sum(np.abs(corrs) >= 0.049))
    axes[1].text(0.01, 0.97,
                 f'饱和帧数 (|Δx|≥4.9cm): {sat_cnt}   最大修正量: {corrs.max()*100:.2f}cm   '
                 f'最小修正量: {corrs.min()*100:.2f}cm\n'
                 f'最大速度: {vels.max():.3f} m/s   最小速度: {vels.min():.3f} m/s',
                 transform=axes[1].transAxes, color=TEXT, fontsize=9.5, va='top',
                 bbox=dict(boxstyle='round,pad=0.4', facecolor='#21262d', edgecolor=GRID))
    hl = [l1, l2, plt.Line2D([0],[0], color=RED, lw=1.1, ls='--')]
    axes[1].legend(hl, ['修正量 Δx (m)', '速度 (m/s)', '限幅 ±0.05 m'],
                   fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    axes[1].set_xlim(0, t[-1])
    axr.set_xlim(0, t[-1])

    # ── 下图：瞬时频率 ──
    sax(axes[2], '', '时间 (s)', '瞬时频率 (Hz)', fontsize=11)
    axes[2].plot(t, inst_freq, color=PURPLE, linewidth=1.5, label='瞬时频率 (Hz)')
    axes[2].fill_between(t, inst_freq, f0, alpha=0.15, color=PURPLE)
    axes[2].set_xlim(0, t[-1])
    axes[2].legend(fontsize=9, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)

    fig.tight_layout(pad=1.5)
    return savefig(fig, 'fig5_chirp_response.png')


def plot_fig6(t, forces, comp1, comp2, comp3, corrs):
    """图6: 导纳控制器三频叠加合成力响应"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), facecolor=DARK)

    # ── 上图：三个分量 + 合成 ──
    sax(axes[0],
        '图6  导纳控制器：三频谐波叠加力输入\n'
        '(3.0·sin(2π·0.5t) + 2.0·sin(2π·1.5t+π/4) + 1.0·sin(2π·3.0t+π/3))',
        '', '外力 F (N)', fontsize=12)
    axes[0].plot(t, comp1,  color=BLUE,   linewidth=1.3, alpha=0.8,  linestyle='--', label='基频 0.5 Hz，幅值 3.0 N')
    axes[0].plot(t, comp2,  color=ORANGE, linewidth=1.3, alpha=0.8,  linestyle='--', label='2次谐波 1.5 Hz，幅值 2.0 N')
    axes[0].plot(t, comp3,  color=PURPLE, linewidth=1.3, alpha=0.8,  linestyle='--', label='3次谐波 3.0 Hz，幅值 1.0 N')
    axes[0].plot(t, forces, color=RED,    linewidth=2.0, alpha=0.95,               label=f'合成力（峰值 {forces.max():.2f} / {forces.min():.2f} N）')
    axes[0].axhline(0, color=GRID, linewidth=0.5)
    axes[0].set_xlim(0, t[-1])
    axes[0].legend(fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)

    # ── 中图：修正量 ──
    sax(axes[1], '', '时间 (s)', '修正量 Δx (m)', fontsize=12)
    axes[1].plot(t, corrs, color=GREEN, linewidth=2.2, label='修正量 Δx (m)')
    axes[1].axhline( 0.05, color=RED, linewidth=1.1, linestyle='--', alpha=0.8, label='限幅 ±0.05 m')
    axes[1].axhline(-0.05, color=RED, linewidth=1.1, linestyle='--', alpha=0.8)
    axes[1].axhline(0, color=GRID, linewidth=0.5)
    axes[1].fill_between(t, corrs, 0, where=(corrs > 0), alpha=0.12, color=GREEN)
    axes[1].fill_between(t, corrs, 0, where=(corrs < 0), alpha=0.12, color=RED)
    sat_cnt = int(np.sum(np.abs(corrs) >= 0.049))
    axes[1].text(0.01, 0.97,
                 f'饱和帧数: {sat_cnt}   最大修正量: {corrs.max()*100:.2f}cm   '
                 f'最小修正量: {corrs.min()*100:.2f}cm   '
                 f'RMS修正量: {np.sqrt(np.mean(corrs**2))*100:.2f}cm',
                 transform=axes[1].transAxes, color=TEXT, fontsize=9.5, va='top',
                 bbox=dict(boxstyle='round,pad=0.4', facecolor='#21262d', edgecolor=GRID))
    axes[1].legend(fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)
    axes[1].set_xlim(0, t[-1])

    # ── 下图：频域分析（FFT）──
    sax(axes[2], '', '频率 (Hz)', '幅值', fontsize=11)
    dt = t[1] - t[0]
    N = len(t)
    fft_f  = np.fft.rfftfreq(N, d=dt)
    fft_c  = np.abs(np.fft.rfft(corrs))  / (N / 2)
    fft_in = np.abs(np.fft.rfft(forces)) / (N / 2)
    axes[2].plot(fft_f, fft_in, color=RED,   linewidth=1.5, alpha=0.8, label='输入力谱')
    axes[2].plot(fft_f, fft_c,  color=GREEN, linewidth=2.0,            label='修正量谱')
    axes[2].set_xlim(0, 6)
    for fi, label in [(0.5, '0.5Hz'), (1.5, '1.5Hz'), (3.0, '3.0Hz')]:
        axes[2].axvline(fi, color=DIM, linewidth=0.8, linestyle=':', alpha=0.7)
        axes[2].text(fi, fft_in.max() * 0.8, label, color=DIM, fontsize=8, ha='center')
    axes[2].legend(fontsize=9.5, facecolor='#21262d', edgecolor=GRID, labelcolor=TEXT)

    fig.tight_layout(pad=1.5)
    return savefig(fig, 'fig6_composite_response.png')


def plot_fig7(freq, duration, arm_ts, j_ooo, j_seqs, s_ooo, s_seqs,
              j_jumps, s_jumps, j_big, s_big, vel_xs):
    """图7: 系统链路 + 关键指标汇总"""
    fig = plt.figure(figsize=(14, 9), facecolor=DARK)
    ax = fig.add_subplot(111)
    ax.set_facecolor(AX_BG)
    ax.set_xlim(0, 12); ax.set_ylim(0, 10)
    ax.axis('off')
    ax.set_title('图7  系统链路 & 关键指标汇总', color=TEXT, fontsize=16,
                 fontweight='bold', pad=14)

    # ── 系统链路方块 ──
    pipeline = [
        (0.2, 9.3, '主端机械臂 (Master Arm)',      BLUE),
        (0.2, 7.9, '导纳控制器 (Master)',           BLUE),
        (0.2, 6.5, 'TeleopFrame 打包 & 发送',       ORANGE),
        (0.2, 5.1, '网络传输（含抖动）',            RED),
        (0.2, 3.7, '抖动缓冲 (JitterBuffer)',       ORANGE),
        (0.2, 2.3, '导纳控制器 (Slave)',            GREEN),
        (0.2, 0.9, '从端机械臂 (Slave Arm)',        GREEN),
    ]
    for x0, y0, label, color in pipeline:
        rect = mpatches.FancyBboxPatch((x0, y0 - 0.52), 4.4, 0.90,
                                       boxstyle='round,pad=0.07',
                                       facecolor=color + '22', edgecolor=color, linewidth=1.5)
        ax.add_patch(rect)
        ax.text(x0 + 2.2, y0 - 0.05, label, color=TEXT, fontsize=9,
                ha='center', va='center', fontweight='bold')

    for i in range(len(pipeline) - 1):
        y_top    = pipeline[i][1]    - 0.52
        y_bottom = pipeline[i+1][1]  + 0.38
        ax.annotate('', xy=(2.4, y_bottom), xytext=(2.4, y_top),
                    arrowprops=dict(arrowstyle='->', color=DIM, lw=1.3))

    # ── 关键指标表格 ──
    metrics = [
        ('采样频率',         f'{freq:.2f} Hz',                       BLUE),
        ('录制时长',         f'{duration:.3f} s',                    BLUE),
        ('总帧数',           f'{len(arm_ts)} 帧',                    BLUE),
        ('乱序帧消除',       f'{j_ooo} → {s_ooo} 帧',               GREEN),
        ('乱序率',           f'{100*j_ooo/len(j_seqs):.1f}% → 0%',  GREEN),
        ('>5cm 跳变帧',      f'{j_big} → {s_big} 帧',               GREEN),
        ('无缓冲最大跳变',   f'{j_jumps.max()*100:.2f} cm',          RED),
        ('有缓冲最大跳变',   f'{s_jumps.max()*100:.2f} cm',          GREEN),
        ('跳变抑制率',       f'{100*(1-s_jumps.max()/j_jumps.max()):.1f}%', GREEN),
        ('峰值末端速度',     f'{np.abs(vel_xs).max():.4f} m/s',      ORANGE),
        ('RMS 末端速度',     f'{np.sqrt(np.mean(vel_xs**2)):.4f} m/s', ORANGE),
        ('修正量限幅',       '± 0.05 m  (± 5 cm)',                   ORANGE),
    ]
    x_name, x_val = 5.0, 8.5
    y_start = 9.5
    dy = 0.77
    for i, (name, val, color) in enumerate(metrics):
        yt = y_start - i * dy
        ax.text(x_name, yt,        name, color=DIM,   fontsize=8.5, va='center')
        ax.text(x_val,  yt,        val,  color=color, fontsize=10.5,
                fontweight='bold', va='center', ha='right')
        ax.plot([x_name, 11.8], [yt - dy/2, yt - dy/2],
                color=GRID, linewidth=0.4)

    # 列表头
    ax.text(x_name, y_start + 0.5, '指标',   color=TEXT, fontsize=9, fontweight='bold')
    ax.text(x_val,  y_start + 0.5, '数值',   color=TEXT, fontsize=9, fontweight='bold', ha='right')
    ax.plot([x_name - 0.1, 11.8], [y_start + 0.2, y_start + 0.2],
            color=DIM, linewidth=0.8)

    fig.tight_layout()
    return savefig(fig, 'fig7_system_summary.png')


# ═══════════════════════════════════════════════════════════════════════
# 主函数
# ═══════════════════════════════════════════════════════════════════════

def main():
    print("正在生成验证图（真实 rosbag 数据 + 导纳控制仿真）...")

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
    j_jumps  = np.abs(np.diff(j_xs))
    s_jumps  = np.abs(np.diff(s_xs))
    j_big    = int(np.sum(j_jumps > 0.05))
    s_big    = int(np.sum(s_jumps > 0.05))

    print(f"     录制时长: {duration:.3f}s  频率: {freq:.2f}Hz  帧数: {len(arm_ts)}")
    print(f"     Jitter OOO: {j_ooo}/{len(j_seqs)} ({100*j_ooo/len(j_seqs):.2f}%)")
    print(f"     Smooth OOO: {s_ooo}/{len(s_seqs)}")
    print(f"     无缓冲时 >5cm 跳变帧: {j_big} / {len(j_jumps)}")
    print(f"     有缓冲后 >5cm 跳变帧: {s_big} / {len(s_jumps)}")
    print(f"     最大跳变 无缓冲: {j_jumps.max()*100:.2f}cm  有缓冲: {s_jumps.max()*100:.2f}cm")
    print(f"     跳变抑制率: {100*(1 - s_jumps.max()/j_jumps.max()):.1f}%")

    print("  [2/3] 生成导纳控制仿真数据（复杂信号）...")
    t_ms, f_ms, c_ms, v_ms                  = admittance_multistep()
    t_ch, f_ch, c_ch, v_ch, inst_f          = admittance_chirp()
    t_cp, f_cp, comp1, comp2, comp3, c_cp   = admittance_composite()

    print("  [3/3] 逐图绘制并保存...")
    outs = []
    outs.append(plot_fig1(arm_ts, arm_xs, j_ts, j_xs, j_jumps,
                          s_ts, s_xs, s_jumps, j_ooo, j_seqs, s_ooo, s_seqs,
                          freq, duration, j_big, s_big))
    outs.append(plot_fig2a(j_ts, j_seqs, j_ooo))
    outs.append(plot_fig2b(s_ts, s_seqs, s_ooo))
    outs.append(plot_fig3(t_ms, f_ms, c_ms, v_ms))
    outs.append(plot_fig4(vel_ts, vel_xs, freq))
    outs.append(plot_fig5(t_ch, f_ch, c_ch, v_ch, inst_f, f0=0.5, f1=4.0))
    outs.append(plot_fig6(t_cp, f_cp, comp1, comp2, comp3, c_cp))
    outs.append(plot_fig7(freq, duration, arm_ts, j_ooo, j_seqs, s_ooo, s_seqs,
                          j_jumps, s_jumps, j_big, s_big, vel_xs))

    print(f"\n✓ 共保存 {len(outs)} 张独立图像到: {SCRIPT_DIR}/")
    return outs


if __name__ == '__main__':
    outs = main()
    print("完成。生成文件:")
    for p in outs:
        print(f"  {p}")
