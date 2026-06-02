#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./debug_admittance_topics.sh [duration_sec] [output_txt]
#
# Example:
#   ./debug_admittance_topics.sh 30 /tmp/admittance_debug.txt
#
# This script:
# 1) Records ROS topics as CSV for a fixed duration.
# 2) Merges nearest timestamps offline.
# 3) Computes useful debug metrics.
# 4) Writes ALL debug output to one txt file.

DURATION_SEC="${1:-30}"
OUTPUT_TXT="${2:-/tmp/admittance_debug_$(date +%Y%m%d_%H%M%S).txt}"

DESIRED_TOPIC="${DESIRED_TOPIC:-/master/desired_pose}"
TOOL_POSE_TOPIC="${TOOL_POSE_TOPIC:-/master_arm/tool_pose}"
FORCE_TOPIC="${FORCE_TOPIC:-/master_force}"
TOOL_VEL_TOPIC="${TOOL_VEL_TOPIC:-/master_arm/tool_velocity}"

WORK_DIR="$(mktemp -d /tmp/admittance_dbg.XXXXXX)"
trap 'rm -rf "$WORK_DIR"' EXIT

DESIRED_CSV="$WORK_DIR/desired.csv"
TOOL_POSE_CSV="$WORK_DIR/tool_pose.csv"
FORCE_CSV="$WORK_DIR/force.csv"
TOOL_VEL_CSV="$WORK_DIR/tool_vel.csv"

echo "[INFO] duration_sec=$DURATION_SEC" | tee "$OUTPUT_TXT"
echo "[INFO] output_txt=$OUTPUT_TXT" | tee -a "$OUTPUT_TXT"
echo "[INFO] topics:" | tee -a "$OUTPUT_TXT"
echo "  desired:   $DESIRED_TOPIC" | tee -a "$OUTPUT_TXT"
echo "  tool_pose: $TOOL_POSE_TOPIC" | tee -a "$OUTPUT_TXT"
echo "  force:     $FORCE_TOPIC" | tee -a "$OUTPUT_TXT"
echo "  tool_vel:  $TOOL_VEL_TOPIC" | tee -a "$OUTPUT_TXT"
echo "[INFO] collecting topic CSV..." | tee -a "$OUTPUT_TXT"

timeout "$DURATION_SEC" rostopic echo -p "$DESIRED_TOPIC" > "$DESIRED_CSV" 2>/dev/null || true
timeout "$DURATION_SEC" rostopic echo -p "$TOOL_POSE_TOPIC" > "$TOOL_POSE_CSV" 2>/dev/null || true
timeout "$DURATION_SEC" rostopic echo -p "$FORCE_TOPIC" > "$FORCE_CSV" 2>/dev/null || true
timeout "$DURATION_SEC" rostopic echo -p "$TOOL_VEL_TOPIC" > "$TOOL_VEL_CSV" 2>/dev/null || true

if [[ ! -s "$DESIRED_CSV" || ! -s "$TOOL_POSE_CSV" ]]; then
  echo "[ERROR] desired/tool_pose CSV is empty. Check roscore and topic names." | tee -a "$OUTPUT_TXT"
  exit 1
fi

echo "[INFO] post-processing..." | tee -a "$OUTPUT_TXT"

python3 - "$DESIRED_CSV" "$TOOL_POSE_CSV" "$FORCE_CSV" "$TOOL_VEL_CSV" "$OUTPUT_TXT" <<'PY'
import csv
import math
import sys
from bisect import bisect_left
from pathlib import Path

desired_csv, tool_csv, force_csv, vel_csv, out_txt = sys.argv[1:]

def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        r = csv.reader(f)
        header = next(r, None)
        if not header:
            return [], []
        for row in r:
            if not row:
                continue
            try:
                rows.append(row)
            except Exception:
                continue
    return header, rows

def idx_map(header):
    return {name: i for i, name in enumerate(header)}

def get_float(row, m, key, default=0.0):
    i = m.get(key)
    if i is None or i >= len(row):
        return default
    try:
        return float(row[i])
    except Exception:
        return default

def norm3(a, b, c):
    return math.sqrt(a*a + b*b + c*c)

hdr_d, rows_d = load_csv(desired_csv)
hdr_t, rows_t = load_csv(tool_csv)
hdr_f, rows_f = load_csv(force_csv) if Path(force_csv).exists() else ([], [])
hdr_v, rows_v = load_csv(vel_csv) if Path(vel_csv).exists() else ([], [])

if not rows_d or not rows_t:
    with open(out_txt, "a") as out:
        out.write("[ERROR] No data after CSV parsing.\n")
    sys.exit(1)

md = idx_map(hdr_d)
mt = idx_map(hdr_t)
mf = idx_map(hdr_f) if hdr_f else {}
mv = idx_map(hdr_v) if hdr_v else {}

t_tool = [get_float(r, mt, "%time", 0.0) for r in rows_t]
t_force = [get_float(r, mf, "%time", 0.0) for r in rows_f] if rows_f else []
t_vel = [get_float(r, mv, "%time", 0.0) for r in rows_v] if rows_v else []

def nearest_row(time_list, rows, t):
    if not time_list or not rows:
        return None
    i = bisect_left(time_list, t)
    if i <= 0:
        return rows[0]
    if i >= len(time_list):
        return rows[-1]
    before = i - 1
    after = i
    if abs(time_list[after] - t) < abs(time_list[before] - t):
        return rows[after]
    return rows[before]

prev_des = None
prev_tool = None
stuck_count = 0
stuck_warns = 0
sample_count = 0

sum_err_pos = 0.0
sum_err_ori = 0.0
max_err_pos = 0.0
max_err_ori = 0.0
max_d_des = 0.0
max_d_tool = 0.0
max_f = 0.0
max_tau = 0.0
max_v = 0.0

with open(out_txt, "a") as out:
    out.write("\n===== DEBUG SAMPLES =====\n")
    out.write("sec\tdDes(m)\tdTool(m)\terrPos(m)\terrOri(rad)\tF(N)\tTau(Nm)\tv(m/s)\tdes_xyz\ttool_xyz\n")

    for rd in rows_d:
        t = get_float(rd, md, "%time", 0.0)
        rt = nearest_row(t_tool, rows_t, t)
        if rt is None:
            continue

        des = [
            get_float(rd, md, "field.position.x"),
            get_float(rd, md, "field.position.y"),
            get_float(rd, md, "field.position.z"),
            get_float(rd, md, "field.orientation.x"),
            get_float(rd, md, "field.orientation.y"),
            get_float(rd, md, "field.orientation.z"),
        ]
        tool = [
            get_float(rt, mt, "field.pose.position.x"),
            get_float(rt, mt, "field.pose.position.y"),
            get_float(rt, mt, "field.pose.position.z"),
            get_float(rt, mt, "field.pose.orientation.x"),
            get_float(rt, mt, "field.pose.orientation.y"),
            get_float(rt, mt, "field.pose.orientation.z"),
        ]

        err = [des[i] - tool[i] for i in range(6)]
        err_pos = norm3(err[0], err[1], err[2])
        err_ori = norm3(err[3], err[4], err[5])

        if prev_des is None:
            d_des = 0.0
            d_tool = 0.0
        else:
            d_des = norm3(des[0]-prev_des[0], des[1]-prev_des[1], des[2]-prev_des[2])
            d_tool = norm3(tool[0]-prev_tool[0], tool[1]-prev_tool[1], tool[2]-prev_tool[2])
        prev_des = des
        prev_tool = tool

        rf = nearest_row(t_force, rows_f, t) if rows_f else None
        if rf is not None:
            fx = get_float(rf, mf, "field.force.x")
            fy = get_float(rf, mf, "field.force.y")
            fz = get_float(rf, mf, "field.force.z")
            tx = get_float(rf, mf, "field.torque.x")
            ty = get_float(rf, mf, "field.torque.y")
            tz = get_float(rf, mf, "field.torque.z")
            fn = norm3(fx, fy, fz)
            taun = norm3(tx, ty, tz)
        else:
            fn = 0.0
            taun = 0.0

        rv = nearest_row(t_vel, rows_v, t) if rows_v else None
        if rv is not None:
            vx = get_float(rv, mv, "field.twist.linear.x")
            vy = get_float(rv, mv, "field.twist.linear.y")
            vz = get_float(rv, mv, "field.twist.linear.z")
            vn = norm3(vx, vy, vz)
        else:
            vn = 0.0

        if d_des > 0.001 and d_tool < 0.0002:
            stuck_count += 1
            if stuck_count >= 10:
                stuck_warns += 1
                stuck_count = 0
        else:
            stuck_count = 0

        sample_count += 1
        sum_err_pos += err_pos
        sum_err_ori += err_ori
        max_err_pos = max(max_err_pos, err_pos)
        max_err_ori = max(max_err_ori, err_ori)
        max_d_des = max(max_d_des, d_des)
        max_d_tool = max(max_d_tool, d_tool)
        max_f = max(max_f, fn)
        max_tau = max(max_tau, taun)
        max_v = max(max_v, vn)

        out.write(
            f"{t:.3f}\t{d_des:.4f}\t{d_tool:.4f}\t{err_pos:.4f}\t{err_ori:.4f}\t{fn:.3f}\t{taun:.3f}\t{vn:.4f}\t"
            f"[{des[0]:.3f},{des[1]:.3f},{des[2]:.3f}]\t[{tool[0]:.3f},{tool[1]:.3f},{tool[2]:.3f}]\n"
        )

    out.write("\n===== SUMMARY =====\n")
    out.write(f"samples={sample_count}\n")
    if sample_count > 0:
        out.write(f"avg_err_pos(m)={sum_err_pos/sample_count:.4f}\n")
        out.write(f"avg_err_ori(rad)={sum_err_ori/sample_count:.4f}\n")
    out.write(f"max_err_pos(m)={max_err_pos:.4f}\n")
    out.write(f"max_err_ori(rad)={max_err_ori:.4f}\n")
    out.write(f"max_dDes(m/step)={max_d_des:.4f}\n")
    out.write(f"max_dTool(m/step)={max_d_tool:.4f}\n")
    out.write(f"max_force_norm(N)={max_f:.3f}\n")
    out.write(f"max_torque_norm(Nm)={max_tau:.3f}\n")
    out.write(f"max_tool_speed(m/s)={max_v:.4f}\n")
    out.write(f"stuck_warnings={stuck_warns}\n")

print(f"[INFO] debug result written to: {out_txt}")
PY

echo "[DONE] saved debug info to $OUTPUT_TXT"
