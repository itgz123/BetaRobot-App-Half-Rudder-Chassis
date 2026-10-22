#!/usr/bin/env python3
"""半舵底盘换向代价函数日志分析。

CSV 布局（app_cfg.h VOFA_CHANNELS=28 + I0 时间戳）：
    I0  DWT 时间戳
    I1  左舵目标角 (deg，wrap ±180)      I2  左舵实际角 (deg)
    I3  右舵目标角 (deg，wrap ±180)      I4  右舵实际角 (deg)
    I5  左轮目标轮缘力矩 (N·m)           I6  左轮实测轮缘力矩 (N·m)
    I7  右轮目标轮缘力矩 (N·m)           I8  右轮实测轮缘力矩 (N·m)
    I9  combo (bit0=L 反向, bit1=R 反向)
    I10~13 cost[0..3] (s)               I14 小力矩锁定
    I15 状态机状态 (0=CAL 1=FAULT 2=SAFE 3=RUN)
    I16 左轮余弦系数                     I17 右轮余弦系数
    I18~20 指令 vx, vy, w
    I21~23 反馈 vx, vy, w
    I24~26 广义力 fx, fy, mw
    I27~28 左右轮速 (rad/s)

用法：
    python3 scripts/cost-analysis/cost_analysis.py ignore/data/*.csv
    python3 scripts/cost-analysis/cost_analysis.py --sweep ignore/data/*.csv
    python3 scripts/cost-analysis/cost_analysis.py --cr 0.25 --cw 0.10 --cd 0.15 \\
        --margin 0.15 --torque-lock 0.20 --sweep ignore/data/*.csv

输出分几块：日志复现校验、执行机构实测速率（舵角/力矩）、代价分量分解、
逐拍最优偏离、混合组合工况、锁定阈值、迟滞扫描、参数单变量扫描。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from data_preprocessing.vofa_csv import load_csv, prepare_time, runs_of  # noqa: E402

CH = {
    "steer_l_tgt": 1,
    "steer_l": 2,
    "steer_r_tgt": 3,
    "steer_r": 4,
    "torque_l_tgt": 5,
    "torque_l": 6,
    "torque_r_tgt": 7,
    "torque_r": 8,
    "combo": 9,
    "cost0": 10,
    "cost1": 11,
    "cost2": 12,
    "cost3": 13,
    "locked": 14,
    "state": 15,
    "align_l": 16,
    "align_r": 17,
    "vx_cmd": 18,
    "vy_cmd": 19,
    "w_cmd": 20,
    "vx_fb": 21,
    "vy_fb": 22,
    "w_fb": 23,
    "fx": 24,
    "fy": 25,
    "mw": 26,
    "speed_l": 27,
    "speed_r": 28,
}

STATE_RUN = 3

# 半舵几何（robot_def.h）
WHEEL_POS = ((0.0, 0.20225), (0.0, -0.20225))  # (x, y), [0]=L [1]=R
WHEEL_R = 0.05025
DIR_EPS = 1e-4       # LIB_CHASSIS_STEER_SPEED_EPS [m/s]
ANGLE_DEADBAND = 0.0175

# def.h 当前整定值
COST_RUDDER = 0.35
COST_WHEEL = 0.20
COST_DIFF = 0.15
SWITCH_MARGIN = 0.15
TORQUE_LOCK = 0.20

# 舵向电机上限 50 rad/s（CHASSIS_STEER_VMAX_MOTOR），舵轮侧 = /8
STEER_VMAX_WHEEL = 50.0 / 8.0  # 6.25 rad/s
SENTINEL_ABS = 1.0e30           # VOFA 首行残值等无效值
REV_SPEED_MIN = 1.0             # 判轮向反转的最小轮速 (rad/s)，低于此符号不可信
STEER_MOVE_RATE = 1.0           # 判“舵轮在动”的角速度阈值 (rad/s，舵轮侧)


def wrap_pi(x: float) -> float:
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def sanitize(x: np.ndarray) -> np.ndarray:
    """把 |x|>1e30 的哨兵/残值换成 NaN。"""
    y = np.asarray(x, dtype=np.float64).copy()
    y[np.abs(y) > SENTINEL_ABS] = np.nan
    return y


def unwrap_runs(x: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """只在 valid 的连续段内解卷绕（舵角反馈被 WRAP 到 ±π，跨段不连续）。"""
    y = np.full(np.shape(x), np.nan)
    idx = np.flatnonzero(np.asarray(valid) & np.isfinite(x))
    if idx.size == 0:
        return y
    splits = np.flatnonzero(np.diff(idx) != 1) + 1
    for seg in np.split(idx, splits):
        if seg.size:
            y[seg] = np.unwrap(np.asarray(x)[seg])
    return y


def wheel_ik(vx: float, vy: float, w: float) -> tuple[np.ndarray, np.ndarray]:
    """速度逆解：返回各轮 (phi_des, speed)。speed<eps 时 phi 无意义（置 0）。"""
    phi = np.zeros(2)
    spd = np.zeros(2)
    for i, (x, y) in enumerate(WHEEL_POS):
        vix = vx - w * y
        viy = vy + w * x
        s = float(np.hypot(vix, viy))
        spd[i] = s
        if s >= DIR_EPS:
            phi[i] = float(np.arctan2(viy, vix))
    return phi, spd


def alloc_torque(alpha: np.ndarray, wrench: np.ndarray) -> np.ndarray | None:
    """与 LibChassisAllocateTorque 等价：J 4x3（每轮驱动行+侧向行），返回各轮轮缘力矩。"""
    rows = []
    drive_row = []
    for i, (x, y) in enumerate(WHEEL_POS):
        c, s = np.cos(alpha[i]), np.sin(alpha[i])
        rows.append([c, s, s * x - c * y])
        drive_row.append(len(rows) - 1)
        rows.append([-s, c, c * x + s * y])
    J = np.asarray(rows, dtype=np.float64)
    A = J.T @ J
    if abs(float(np.linalg.det(A))) < 1e-9:
        return None
    q = np.linalg.solve(A, np.asarray(wrench, dtype=np.float64))
    f = J @ q
    return np.asarray([WHEEL_R * f[k] for k in drive_row])


def combo_costs(phi_des: np.ndarray, spd: np.ndarray, cur: np.ndarray,
                wrench: np.ndarray, tau_meas: np.ndarray,
                keep: int, cr: float = COST_RUDDER, cw: float = COST_WHEEL,
                cd: float = COST_DIFF) -> dict:
    """按当前固件逻辑计算 4 个组合的代价与分量（含无方向轮的 keep 处理）。"""
    dir_mask = int((spd[0] >= DIR_EPS)) | (int(spd[1] >= DIR_EPS) << 1)
    out = {
        "dir_mask": dir_mask,
        "eff": np.zeros(4, dtype=int),
        "cost": np.full(4, 1e30),
        "theta": np.zeros((4, 2)),
        "taue": np.zeros((4, 2)),
        "tau_t": np.zeros((4, 2)),
        "dphi": np.zeros((4, 2)),
        "rot": np.zeros((4, 2), dtype=int),
        "diff": np.zeros(4, dtype=bool),
        "alloc_ok": np.zeros(4, dtype=bool),
    }
    if dir_mask == 0:
        return out

    for raw in range(4):
        combo = (raw & dir_mask) | keep
        out["eff"][raw] = combo
        alpha = cur.copy()
        for i in range(2):
            if dir_mask >> i & 1:
                rev = (combo >> i) & 1
                phi_t = phi_des[i] + (np.pi if rev else 0.0)
                dphi = wrap_pi(phi_t - cur[i])
                out["dphi"][raw, i] = dphi
                alpha[i] = cur[i] + dphi
                # 固件用 |dphi| 扣死区，旋转方向按 dphi 判
                out["theta"][raw, i] = max(abs(dphi) - ANGLE_DEADBAND, 0.0)
                if dphi > ANGLE_DEADBAND:
                    out["rot"][raw, i] = 1
                elif dphi < -ANGLE_DEADBAND:
                    out["rot"][raw, i] = -1
            else:
                out["dphi"][raw, i] = 0.0
                out["theta"][raw, i] = 0.0
                out["rot"][raw, i] = 0

        tau_t = alloc_torque(alpha, wrench)
        if tau_t is None:
            continue
        out["alloc_ok"][raw] = True
        out["tau_t"][raw] = tau_t
        out["taue"][raw] = np.abs(tau_t - tau_meas)
        c = float(np.sum(out["theta"][raw] * cr + out["taue"][raw] * cw))
        if out["rot"][raw, 0] != 0 and out["rot"][raw, 1] != 0 and out["rot"][raw, 0] != out["rot"][raw, 1]:
            c += cd
            out["diff"][raw] = True
        out["cost"][raw] = c
    return out


def row_inputs(df: pd.DataFrame, idx: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    r = df.iloc[idx]
    vx, vy, w = float(r[f"I{CH['vx_cmd']}"]), float(r[f"I{CH['vy_cmd']}"]), float(r[f"I{CH['w_cmd']}"])
    phi, spd = wheel_ik(vx, vy, w)
    cur = np.radians([float(r[f"I{CH['steer_l']}"]), float(r[f"I{CH['steer_r']}"] )])
    cur = np.asarray([wrap_pi(x) for x in cur])
    wrench = np.asarray([float(r[f"I{CH['fx']}"]), float(r[f"I{CH['fy']}"]), float(r[f"I{CH['mw']}"])])
    tau_meas = np.asarray([float(r[f"I{CH['torque_l']}"]), float(r[f"I{CH['torque_r']}"])])
    return phi, spd, cur, wrench, tau_meas


def prepare(path: Path) -> tuple[pd.DataFrame, dict]:
    df = load_csv(path)
    prep = prepare_time(df["I0"].to_numpy())
    df = df.iloc[prep["dropped"]:].reset_index(drop=True)
    # 段内时间轴是段间拼接的，段外为 0；这里只保留第一段（这些日志都是一次会话）
    segs = prep["segs"]
    if segs:
        a, b = max(segs, key=lambda s: s[1] - s[0])
        df = df.iloc[a:b].reset_index(drop=True)
        prep["t_s"] = prep["t_s"][a:b]
    df = df.copy()
    df["t_s"] = prep["t_s"]
    return df, prep


def validate_reconstruction(df: pd.DataFrame) -> None:
    """用日志里的指令/状态/力复算 4 组合代价，与固件写入的 cost 通道对比。"""
    costs = np.column_stack([df[f"I{CH[f'cost{k}']}"].to_numpy() for k in range(4)])
    state = df[f"I{CH['state']}"].to_numpy()
    errs = []
    n = 0
    for idx in range(len(df)):
        if state[idx] != STATE_RUN:
            continue
        phi, spd, cur, wrench, tau_meas = row_inputs(df, idx)
        if np.count_nonzero(spd >= DIR_EPS) != 2:
            continue
        c = combo_costs(phi, spd, cur, wrench, tau_meas, keep=0)
        e = np.abs(c["cost"] - costs[idx])
        if np.all(np.isfinite(e)):
            errs.append(e)
            n += 1
    if not errs:
        print("  [validate] 没有 dir_mask=3 的 RUN 行")
        return
    e = np.asarray(errs)
    print(f"  [validate] n={n}  |recalc-logged| mean={e.mean():.4f} "
          f"p95={np.percentile(e, 95):.4f} max={e.max():.4f}  (四列合计)")


def simulate_cost(df: pd.DataFrame, cr: float = COST_RUDDER, cw: float = COST_WHEEL,
                  cd: float = COST_DIFF, margin: float = SWITCH_MARGIN,
                  torque_lock: float = TORQUE_LOCK) -> dict:
    """按固件逻辑逐帧模拟代价方案；返回每帧的选择与代价/分量。"""
    n = len(df)
    state = df[f"I{CH['state']}"].to_numpy()
    combo = np.full(n, -1, dtype=int)
    cost = np.full((n, 4), np.nan)
    theta = np.full((n, 2), np.nan)
    taue = np.full((n, 2), np.nan)
    tau_t = np.full((n, 2), np.nan)
    dphi = np.full((n, 2), np.nan)
    align = np.full((n, 2), np.nan)
    diff = np.zeros(n, dtype=bool)
    locked = np.zeros(n, dtype=bool)
    dir_mask_arr = np.zeros(n, dtype=int)
    best_eff_arr = np.full(n, -1, dtype=int)
    gap_arr = np.full(n, np.nan)      # 当前组合代价 − 最优组合代价
    blocked = np.zeros(n, dtype=bool)  # 最优 ≠ 当前（迟滞是否挡下）

    valid = False
    cur_combo = 0
    for idx in range(n):
        if state[idx] != STATE_RUN:
            valid = False
            cur_combo = 0
            continue
        phi, spd, cur, wrench, tau_meas = row_inputs(df, idx)
        dir_mask = int(spd[0] >= DIR_EPS) | (int(spd[1] >= DIR_EPS) << 1)
        dir_mask_arr[idx] = dir_mask
        keep = (cur_combo & ~dir_mask) & 0x3 if valid else 0

        if dir_mask == 0:
            tau_now = alloc_torque(cur, wrench)
            small = tau_now is not None and np.all(np.abs(tau_now) < torque_lock)
            if small:
                locked[idx] = True
                combo[idx] = cur_combo
                cost[idx] = 0.0
                theta[idx] = 0.0
                taue[idx] = 0.0
                tau_t[idx] = 0.0
                dphi[idx] = 0.0
                align[idx] = 1.0
                continue
            valid = True
            cur_combo = keep
            combo[idx] = cur_combo
            cost[idx] = 0.0
            theta[idx] = 0.0
            if tau_now is None:
                tau_t[idx] = 0.0
                taue[idx] = 0.0
            else:
                tau_t[idx] = tau_now
                taue[idx] = np.abs(tau_now - tau_meas)
            dphi[idx] = 0.0
            align[idx] = 1.0
            continue

        c = combo_costs(phi, spd, cur, wrench, tau_meas, keep, cr, cw, cd)
        cost[idx] = c["cost"]
        best = int(np.argmin(c["cost"]))
        best_eff = (best & dir_mask) | keep
        gap = float(c["cost"][cur_combo] - c["cost"][best])
        best_eff_arr[idx] = best_eff
        gap_arr[idx] = gap
        if not valid:
            cur_combo = best_eff
        elif (best_eff != cur_combo and c["alloc_ok"][best]
              and gap > margin):
            cur_combo = best_eff
        valid = True
        combo[idx] = cur_combo
        blocked[idx] = (combo[idx] != best_eff)
        sel = cur_combo
        theta[idx] = c["theta"][sel]
        taue[idx] = c["taue"][sel]
        tau_t[idx] = c["tau_t"][sel]
        dphi[idx] = c["dphi"][sel]
        for i in range(2):
            align[idx, i] = max(np.cos(c["dphi"][sel, i]), 0.0) if dir_mask >> i & 1 else 1.0
        diff[idx] = c["diff"][sel]
    return {
        "combo": combo, "cost": cost, "theta": theta, "taue": taue, "tau_t": tau_t,
        "dphi": dphi, "align": align, "diff": diff, "locked": locked,
        "dir_mask": dir_mask_arr, "best_eff": best_eff_arr, "gap": gap_arr,
        "blocked": blocked,
    }


def simulate_shortest(df: pd.DataFrame) -> dict:
    """模拟方案 0：逐轮选更近的 ±180° 分支（带角度死区/平手保持）。"""
    n = len(df)
    state = df[f"I{CH['state']}"].to_numpy()
    combo = np.full(n, -1, dtype=int)
    travel = np.full(n, np.nan)
    align = np.full((n, 2), np.nan)
    tau = np.full((n, 2), np.nan)
    dir_mask_arr = np.zeros(n, dtype=int)
    valid = False
    rev = np.zeros(2, dtype=int)
    for idx in range(n):
        if state[idx] != STATE_RUN:
            valid = False
            rev[:] = 0
            continue
        phi, spd, cur, wrench, tau_meas = row_inputs(df, idx)
        dir_mask_arr[idx] = int(spd[0] >= DIR_EPS) | (int(spd[1] >= DIR_EPS) << 1)
        alpha = cur.copy()
        d = np.zeros(2)
        for i in range(2):
            if spd[i] < DIR_EPS:
                rev[i] = rev[i] if valid else 0
                continue
            d_fwd = wrap_pi(phi[i] - cur[i])
            d_rev = wrap_pi(phi[i] + np.pi - cur[i])
            if abs(d_rev) < abs(d_fwd) - ANGLE_DEADBAND:
                rev[i] = 1
            elif abs(d_fwd) < abs(d_rev) - ANGLE_DEADBAND:
                rev[i] = 0
            elif not valid:
                rev[i] = 0
            d[i] = d_rev if rev[i] else d_fwd
            alpha[i] = cur[i] + d[i]
        tau_now = alloc_torque(alpha, wrench)
        combo[idx] = int(rev[0] | (rev[1] << 1))
        travel[idx] = float(np.sum(np.abs(d)))
        for i in range(2):
            align[idx, i] = max(np.cos(d[i]), 0.0) if spd[i] >= DIR_EPS else 1.0
            tau[idx, i] = tau_now[i] if tau_now is not None else 0.0
        valid = True
    return {"combo": combo, "travel": travel, "align": align, "tau": tau,
            "dir_mask": dir_mask_arr}


def selection_metrics(sim: dict, mask: np.ndarray) -> dict:
    """在 mask 行上汇总选择质量。"""
    travel = np.nansum(np.abs(sim["dphi"][mask]), axis=1)
    taue = np.nansum(sim["taue"][mask], axis=1)
    align = np.nanmin(sim["align"][mask], axis=1)
    combo = sim["combo"][mask]
    sw = int(np.count_nonzero(np.diff(combo) != 0)) if combo.size else 0
    return {
        "n": int(mask.sum()),
        "switches": sw,
        "travel_med": float(np.median(travel)) if travel.size else np.nan,
        "travel_mean": float(np.mean(travel)) if travel.size else np.nan,
        "taue_med": float(np.median(taue)) if taue.size else np.nan,
        "align_med": float(np.median(align)) if align.size else np.nan,
        "align_p5": float(np.percentile(align, 5)) if align.size else np.nan,
        "align_lt05": float(np.mean(align < 0.5) * 100) if align.size else np.nan,
        "mixed": float(np.mean((combo == 1) | (combo == 2)) * 100) if combo.size else np.nan,
    }


def reversal_stats(df: pd.DataFrame, sim: dict) -> dict:
    """统计“目标轮向自旋方向与实测轮速相反”的帧/起始次数。

    轮速 CH27/28 是 app 正方向（沿当前舵角），所以目标反向位 bit=1 时
    期望自旋符号为负；两者异号且轮速足够大 ⇒ 这一拍轮子正在反着转。
    """
    state = df[f"I{CH['state']}"].to_numpy()
    run = state == STATE_RUN
    speed = np.column_stack([sanitize(df[f"I{CH['speed_l']}"].to_numpy()),
                             sanitize(df[f"I{CH['speed_r']}"].to_numpy())])
    dm = sim["dir_mask"]
    combo = sim["combo"]
    n = len(df)
    rev_frames = np.zeros(n, dtype=bool)
    onset = np.zeros(n, dtype=bool)
    moving = np.zeros(n, dtype=bool)
    for i in range(2):
        has = (run & (((dm >> i) & 1) != 0) & np.isfinite(speed[:, i])
               & (np.abs(speed[:, i]) >= REV_SPEED_MIN))
        spin = np.where(((combo >> i) & 1) == 1, -1.0, 1.0)
        rev = has & (spin * np.sign(speed[:, i]) < 0)
        rev_frames |= rev
        onset |= rev & ~np.r_[False, rev[:-1]]
        moving |= has
    n_moving = int(moving.sum())
    return {
        "moving": n_moving,
        "rev_frames": int(rev_frames.sum()),
        "rev_pct": 100.0 * rev_frames.sum() / max(n_moving, 1),
        "rev_onsets": int(onset.sum()),
    }


def analyze_dynamics(df: pd.DataFrame) -> None:
    """从实测舵角/轮速反推执行机构的真实速率，用于标定代价系数的物理含义。"""
    t = df["t_s"].to_numpy()
    state = df[f"I{CH['state']}"].to_numpy()
    run = state == STATE_RUN
    n = len(df)
    dt = np.diff(t)
    pair = run[1:] & run[:-1] & (dt > 0)

    print(f"  [dynamics] 舵向电机上限 {STEER_VMAX_WHEEL:.2f} rad/s（舵轮侧）；"
          f"cost_rudder={COST_RUDDER:.2f} s/rad ⇒ 隐含 {1.0 / COST_RUDDER:.2f} rad/s")
    all_eff = []
    for name, label in [("steer_l", "左舵"), ("steer_r", "右舵")]:
        raw = np.radians(sanitize(df[f"I{CH[name]}"].to_numpy()))
        ang = unwrap_runs(raw, run & np.isfinite(raw))
        rate = np.full(n, np.nan)
        rate[1:] = np.diff(ang) / np.where(dt > 0, dt, np.nan)
        ok = np.r_[False, pair] & np.isfinite(rate)
        r = np.abs(rate[ok])
        moving = r[r >= STEER_MOVE_RATE]
        if moving.size == 0:
            print(f"    {label}: 没有 |rate|>={STEER_MOVE_RATE:.1f} rad/s 的舵轮运动")
            continue
        print(f"    {label}: |dφ/dt| 全部 p50={np.median(r):5.2f} p90={np.percentile(r, 90):5.2f} "
              f"p99={np.percentile(r, 99):5.2f} max={r.max():5.2f} rad/s；"
              f"运动中 p50={np.median(moving):5.2f} p90={np.percentile(moving, 90):5.2f}")

        # 整段转动（连续 |rate|>=阈值）的有效速率 = 总转角 / 总时长（含加减速）
        rm = np.zeros(n, dtype=bool)
        rm[1:] = ok[1:] & (np.abs(rate[1:]) >= STEER_MOVE_RATE)
        eps = []
        for a, b in runs_of(rm):
            dur = t[b - 1] - t[a - 1]
            travel = abs(ang[b - 1] - ang[a - 1])
            if dur >= 0.03 and travel >= 0.10:
                eps.append((travel, dur, travel / dur))
        if eps:
            e = np.asarray(eps)
            all_eff.append(e[:, 2])
            print(f"          转动片段 {len(eps)} 段：转角 med={np.median(e[:, 0]):5.2f} rad  "
                  f"时长 med={np.median(e[:, 1]) * 1e3:5.1f} ms  "
                  f"有效速率 med={np.median(e[:, 2]):5.2f} "
                  f"p10={np.percentile(e[:, 2], 10):5.2f} p90={np.percentile(e[:, 2], 90):5.2f} rad/s")
    if all_eff:
        x = np.concatenate(all_eff)
        print(f"    两舵合并有效速率 med={np.median(x):.2f} rad/s ⇒ 时间等价系数约 "
              f"{1.0 / np.median(x):.3f} s/rad（当前 {COST_RUDDER:.2f}）")

    print(f"  [dynamics] 轮向力矩：cost_wheel={COST_WHEEL:.2f} s/(N·m) ⇒ 隐含 1/{COST_WHEEL:.1f}="
          f"{1.0 / COST_WHEEL:.2f} N·m/s 的变化速率")
    for name, label in [("torque_l", "左轮"), ("torque_r", "右轮")]:
        tau = sanitize(df[f"I{CH[name]}"].to_numpy())
        dtau = np.abs(np.diff(tau))
        ok = pair & np.isfinite(dtau)
        if not ok.any():
            continue
        step = dtau[ok]
        slew = step / dt[ok]
        print(f"    {label}: 每拍 |Δτ| p50={np.median(step):5.3f} p90={np.percentile(step, 90):5.3f} "
              f"p99={np.percentile(step, 99):5.3f} N·m；等效 p90={np.percentile(slew, 90):6.1f} "
              f"p99={np.percentile(slew, 99):6.1f} N·m/s")


def analyze_deviations(df: pd.DataFrame, sim: dict, cr: float = COST_RUDDER,
                       cw: float = COST_WHEEL, cd: float = COST_DIFF, top: int = 10) -> None:
    """选中组合与逐拍代价最优不一致的片段：多转了多少、少要了多少力矩。"""
    t = df["t_s"].to_numpy()
    state = df[f"I{CH['state']}"].to_numpy()
    run = state == STATE_RUN
    mask = (run & (sim["dir_mask"] == 3) & (sim["combo"] >= 0)
            & (sim["best_eff"] >= 0) & (sim["combo"] != sim["best_eff"]))
    if not mask.any():
        print("  [deviations] 没有偏离逐拍最优的帧")
        return
    idxs = np.flatnonzero(mask)
    print(f"  [deviations] 偏离逐拍最优 {mask.sum()} 帧 / 双轮有向 {int((run & (sim['dir_mask'] == 3)).sum())} 帧 "
          f"= {mask.sum() / max(int((run & (sim['dir_mask'] == 3)).sum()), 1) * 100:.2f}%")
    dtravel, dtaue, dangle, dtorque, ddiff = [], [], [], [], []
    for idx in idxs:
        phi, spd, cur, wrench, tau_meas = row_inputs(df, idx)
        c = combo_costs(phi, spd, cur, wrench, tau_meas, keep=0, cr=cr, cw=cw, cd=cd)
        s, bs = int(sim["combo"][idx]), int(sim["best_eff"][idx])
        dtravel.append(np.abs(c["dphi"][s]).sum() - np.abs(c["dphi"][bs]).sum())
        dtaue.append(c["taue"][s].sum() - c["taue"][bs].sum())
        dangle.append((c["theta"][s].sum() - c["theta"][bs].sum()) * cr)
        dtorque.append((c["taue"][s].sum() - c["taue"][bs].sum()) * cw)
        ddiff.append((cd if c["diff"][s] else 0.0) - (cd if c["diff"][bs] else 0.0))
    for name, x in [("Δ转角(rad)", dtravel), ("Δ力矩误差(N·m)", dtaue),
                    ("Δ角度代价(s)", dangle), ("Δ力矩代价(s)", dtorque), ("Δdiff代价(s)", ddiff)]:
        x = np.asarray(x)
        print(f"    {name:16s} med={np.median(x):+7.3f} mean={np.mean(x):+7.3f} "
              f"p5={np.percentile(x, 5):+7.3f} p95={np.percentile(x, 95):+7.3f}")

    # 片段级：持续多久、在什么指令下
    runs = runs_of(mask)
    print(f"    共 {len(runs)} 个片段，按时长排序（最多 {top} 个）：")
    eps = []
    for a, b in runs:
        dur = t[b - 1] - t[a - 1] if b - 1 > a else 0.0
        vx = float(np.nanmedian(df[f"I{CH['vx_cmd']}"].to_numpy()[a:b]))
        vy = float(np.nanmedian(df[f"I{CH['vy_cmd']}"].to_numpy()[a:b]))
        w = float(np.nanmedian(df[f"I{CH['w_cmd']}"].to_numpy()[a:b]))
        eps.append((dur, t[a], a, b, vx, vy, w, int(sim["combo"][a]), int(sim["best_eff"][a])))
    for dur, t0, a, b, vx, vy, w, csel, cbest in sorted(eps, reverse=True)[:top]:
        gap = np.nanmedian(sim["gap"][a:b])
        print(f"      t={t0:6.2f}s dur={dur * 1e3:6.1f}ms cmd=({vx:+.2f},{vy:+.2f},{w:+.2f}) "
              f"chosen={csel} best={cbest} gap={gap:.3f}s")


def analyze_mixed(df: pd.DataFrame, sim: dict) -> None:
    """混合组合（一正一反）出现的工况：是否由 w / 平移速度决定。"""
    state = df[f"I{CH['state']}"].to_numpy()
    run = state == STATE_RUN
    dm3 = run & (sim["dir_mask"] == 3)
    if not dm3.any():
        return
    combo = sim["combo"]
    mixed = dm3 & ((combo == 1) | (combo == 2))
    print(f"  [mixed] 双轮有向帧 {dm3.sum()}，混合组合 {mixed.sum()} ({mixed.sum() / dm3.sum() * 100:.1f}%)")
    w = df[f"I{CH['w_cmd']}"].to_numpy()
    v = np.hypot(df[f"I{CH['vx_cmd']}"].to_numpy(), df[f"I{CH['vy_cmd']}"].to_numpy())
    for name, x, edges in [("|w|", np.abs(w), [0.0, 0.05, 0.2, 0.5, 1e9]),
                           ("|v|", v, [0.0, 0.05, 0.2, 0.5, 1e9])]:
        parts = []
        for lo, hi in zip(edges[:-1], edges[1:]):
            sel = dm3 & (x >= lo) & (x < hi)
            if sel.any():
                parts.append(f"[{lo:g},{hi:g}) {np.mean(mixed[sel]) * 100:4.0f}%({sel.sum()})")
        print(f"    by {name}: " + "  ".join(parts))
    print(f"    c_diff 实际计入的帧：{np.mean(sim['diff'][dm3]) * 100:.1f}%")


def analyze_hysteresis_sweep(df: pd.DataFrame, base: dict) -> None:
    """迟滞 margin 扫描：切换数、被挡下的时间、自旋反转起始数。"""
    state = df[f"I{CH['state']}"].to_numpy()
    run = state == STATE_RUN
    t = df["t_s"].to_numpy()
    dt = np.diff(t)
    print("  [hysteresis sweep] margin | switch  raw_nonopt%  blocked%  blocked_s  rev_onset  travel_med  taue_med")
    for margin in [0.0, 0.05, 0.10, 0.15, 0.25, 0.40]:
        sim = simulate_cost(df, cr=base["cr"], cw=base["cw"], cd=base["cd"],
                            margin=margin, torque_lock=TORQUE_LOCK)
        mask = run & (sim["combo"] >= 0)
        m = selection_metrics(sim, mask)
        rev = reversal_stats(df, sim)
        gap = sim["gap"]
        nonopt = mask & np.isfinite(gap) & (gap > 1e-9)
        blocked = mask & sim["blocked"] & np.isfinite(gap) & (gap > 1e-9)
        bl_pair = blocked[1:] & blocked[:-1] & (dt > 0)
        blocked_s = float(np.sum(dt[bl_pair])) if blocked.any() else 0.0
        print(f"    {margin:5.2f} | {m['switches']:6d}  {nonopt.sum() / max(mask.sum(), 1) * 100:9.1f}%  "
              f"{blocked.sum() / max(mask.sum(), 1) * 100:7.1f}%  {blocked_s:9.2f}  "
              f"{rev['rev_onsets']:9d}  {m['travel_med']:10.3f}  {m['taue_med']:8.3f}")


def analyze_lock(df: pd.DataFrame) -> None:
    """两轮都无速度方向时，锁定阈值 TORQUE_LOCK 相对实际分配力矩是否合理。"""
    state = df[f"I{CH['state']}"].to_numpy()
    run = state == STATE_RUN
    tau_max = []
    n_nodir = 0
    for idx in np.flatnonzero(run):
        phi, spd, cur, wrench, tau_meas = row_inputs(df, idx)
        if np.count_nonzero(spd >= DIR_EPS) != 0:
            continue
        n_nodir += 1
        tau = alloc_torque(cur, wrench)
        if tau is not None:
            tau_max.append(float(np.max(np.abs(tau))))
    if not tau_max:
        return
    x = np.asarray(tau_max)
    print(f"  [lock] 两轮无方向帧 {n_nodir}，分配 |τ|max：med={np.median(x):.3f} "
          f"p90={np.percentile(x, 90):.3f} p95={np.percentile(x, 95):.3f} "
          f"max={x.max():.3f} N·m（阈值 {TORQUE_LOCK:.2f}）")


def analyze_components(df: pd.DataFrame, cr: float = COST_RUDDER,
                       cw: float = COST_WHEEL, cd: float = COST_DIFF) -> None:
    """分解选中组合的代价，并对比“逐轮最短路径”基准。"""
    state = df[f"I{CH['state']}"].to_numpy()
    combo = df[f"I{CH['combo']}"].to_numpy().astype(int)
    ctx_rows = []
    for idx in range(len(df)):
        if state[idx] != STATE_RUN:
            continue
        phi, spd, cur, wrench, tau_meas = row_inputs(df, idx)
        if np.count_nonzero(spd >= DIR_EPS) != 2:
            continue
        c = combo_costs(phi, spd, cur, wrench, tau_meas, keep=0, cr=cr, cw=cw, cd=cd)
        ctx_rows.append((idx, c))
    if not ctx_rows:
        print("  [components] 没有双轮都有方向的 RUN 行")
        return
    idxs = np.asarray([r[0] for r in ctx_rows])
    cs = [r[1] for r in ctx_rows]
    sel = combo[idxs]

    angle = np.asarray([c["theta"][s].sum() * cr for c, s in zip(cs, sel)])
    torque = np.asarray([c["taue"][s].sum() * cw for c, s in zip(cs, sel)])
    diffterm = np.asarray([cd if c["diff"][s] else 0.0 for c, s in zip(cs, sel)])
    total = angle + torque + diffterm
    print(f"  [components] n={len(idxs)} 选中组合代价分解（中位/p95）：")
    for name, x in [("angle", angle), ("torque", torque), ("diff", diffterm), ("total", total)]:
        print(f"    {name:7s} med={np.median(x):7.3f} p95={np.percentile(x, 95):7.3f} "
              f"share={np.mean(x > 0) * 100:5.1f}%")
    share = np.where(total > 1e-9, angle / np.maximum(total, 1e-9), 0.0)
    print(f"    angle share of total: med={np.median(share):.2f} p95={np.percentile(share, 95):.2f}")

    # 逐轮看“角度偏好”与“力矩偏好”何时冲突：冲突帧的转角差/力矩差
    # 决定代价权重是否真的在改变选择。
    dis_ratio = []
    for c in cs:
        for i in range(2):
            a0, a1 = abs(c["dphi"][0, i]), abs(c["dphi"][1 << i, i])
            t0, t1 = c["taue"][0, i], c["taue"][1 << i, i]
            ang_pref = 0 if a0 <= a1 else 1
            tau_pref = 0 if t0 <= t1 else 1
            if ang_pref != tau_pref:
                d_ang = abs(a0 - a1)
                d_tau = abs(t0 - t1)
                if d_tau > 1e-6:
                    dis_ratio.append((d_ang, d_tau, d_ang / d_tau))
    if dis_ratio:
        r = np.asarray(dis_ratio)
        print(f"    角度偏好≠力矩偏好：{len(r)} / {2 * len(cs)} 个轮决策 = {len(r) / (2 * len(cs)) * 100:.1f}%")
        print(f"      冲突时 转角差 med={np.median(r[:, 0]):.3f} rad  "
              f"力矩差 med={np.median(r[:, 1]):.3f} N·m  "
              f"临界 cw/cr med={np.median(r[:, 2]):.3f}（当前 {cw / cr:.3f}）")
        print(f"      临界比 > 当前 ⇒ 力矩项能推翻最近分支：{np.mean(r[:, 2] > cw / cr) * 100:.1f}% 的冲突帧")

    # 逐轮严格最近分支作为基准（不计平手死区）
    rec_short = []
    rec_best = []
    for c in cs:
        d = c["dphi"]  # [raw, wheel], raw combos only valid when dir_mask=3; each wheel's neg branch
        # 对每轮：raw bit=0 与 bit=1 中 |dphi| 更小者
        bits = []
        for i in range(2):
            bits.append(0 if abs(d[0, i]) <= abs(d[1 << i, i]) else 1)
        rec_short.append(bits[0] | (bits[1] << 1))
        rec_best.append(int(np.argmin(c["cost"])))
    short = np.asarray(rec_short)
    best = np.asarray(rec_best)
    agree_short = np.mean(sel == short) * 100
    agree_best = np.mean(sel == best) * 100
    print(f"    chosen==strict-nearest {agree_short:.1f}%   chosen==argmin {agree_best:.1f}%")

    # 不一致时：代价方案多花/少花多少角度与力矩
    dis = sel != short
    if dis.any():
        def comp(c, s):
            return (c["theta"][s].sum(), c["taue"][s].sum(), c["diff"][s])
        d_ang, d_tau, d_diff = [], [], []
        d_travel, d_taue = [], []
        for c, s, sh in zip(cs, sel, short):
            a_s, t_s, _ = comp(c, s)
            a_h, t_h, _ = comp(c, sh)
            d_ang.append((a_s - a_h) * cr)
            d_tau.append((t_s - t_h) * cw)
            d_diff.append((cd if c["diff"][s] else 0.0) - (cd if c["diff"][sh] else 0.0))
            d_travel.append(c["theta"][s].sum() - c["theta"][sh].sum())
            d_taue.append(c["taue"][s].sum() - c["taue"][sh].sum())
        print(f"    偏离 shortest 的比例 {dis.mean() * 100:.1f}%；偏离时（选中 − shortest）：")
        for name, x in [("Δangle_cost", d_ang), ("Δtorque_cost", d_tau), ("Δdiff_cost", d_diff),
                        ("Δsteer_rad", d_travel), ("Δtorque_err", d_taue)]:
            x = np.asarray(x)[dis]
            print(f"      {name:12s} med={np.median(x):+7.3f} mean={np.mean(x):+7.3f} "
                  f"p5={np.percentile(x, 5):+7.3f} p95={np.percentile(x, 95):+7.3f}")


def analyze_hysteresis(df: pd.DataFrame, sim: dict) -> None:
    """统计候选切换的代价优势 / 迟滞挡下的次数。"""
    state = df[f"I{CH['state']}"].to_numpy()
    mask = (state == STATE_RUN) & (sim["dir_mask"] == 3) & np.isfinite(sim["cost"]).all(axis=1)
    if not mask.any():
        return
    cost = sim["cost"][mask]
    combo = sim["combo"][mask]
    best = np.argmin(cost, axis=1)
    cur = cost[np.arange(len(combo)), combo]
    adv = cur - cost[np.arange(len(combo)), best]  # 切换到 best 能省多少
    # 只关心“当前不是最优”的帧
    cand = best != combo
    cand_adv = adv[cand]
    if cand_adv.size:
        print(f"  [hysteresis] 当前非最优帧 {cand.mean() * 100:.1f}%  "
              f"可省代价 med={np.median(cand_adv):.3f} p95={np.percentile(cand_adv, 95):.3f} "
              f"max={cand_adv.max():.3f}")
        for m in [0.0, 0.05, 0.10, 0.15, 0.25, 0.50]:
            print(f"    margin={m:.2f}s 会切换的帧={np.mean(cand_adv > m) * 100:5.1f}%")
    raw_sw = int(np.count_nonzero(np.diff(best) != 0))
    print(f"  [hysteresis] 无迟滞 raw argmin 切换={raw_sw}  "
          f"实际（含迟滞）切换={int(np.count_nonzero(np.diff(combo) != 0))}")


def analyze_vs_shortest(df: pd.DataFrame, sim_cost: dict, sim_short: dict) -> None:
    state = df[f"I{CH['state']}"].to_numpy()
    mask = (state == STATE_RUN) & (sim_cost["dir_mask"] == 3)
    if not mask.any():
        return
    mc = selection_metrics(sim_cost, mask)
    ms = {
        "switches": int(np.count_nonzero(np.diff(sim_short["combo"][mask]) != 0)),
        "travel_med": float(np.nanmedian(sim_short["travel"][mask])),
        "travel_mean": float(np.nanmean(sim_short["travel"][mask])),
        "align_med": float(np.nanmedian(np.nanmin(sim_short["align"][mask], axis=1))),
        "align_p5": float(np.nanpercentile(np.nanmin(sim_short["align"][mask], axis=1), 5)),
        "align_lt05": float(np.nanmean(np.nanmin(sim_short["align"][mask], axis=1) < 0.5) * 100),
        "mixed": float(np.mean((sim_short["combo"][mask] == 1) | (sim_short["combo"][mask] == 2)) * 100),
    }
    tau_meas = np.column_stack([df[f"I{CH['torque_l']}"].to_numpy(),
                                df[f"I{CH['torque_r']}"].to_numpy()])
    short_taue = np.nanmedian(np.nansum(np.abs(sim_short["tau"] - tau_meas), axis=1)[mask])
    rv_c = reversal_stats(df, sim_cost)
    rv_s = reversal_stats(df, sim_short)
    print("  [cost vs shortest]                 cost      shortest")
    print(f"    switches                        {mc['switches']:6d}      {ms['switches']:6d}")
    print(f"    steer |dphi| med (rad)          {mc['travel_med']:6.3f}      {ms['travel_med']:6.3f}")
    print(f"    steer |dphi| mean (rad)         {mc['travel_mean']:6.3f}      {ms['travel_mean']:6.3f}")
    print(f"    torque err med (N·m)            {mc['taue_med']:6.3f}      {float(short_taue):6.3f}")
    print(f"    min align med                   {mc['align_med']:6.3f}      {ms['align_med']:6.3f}")
    print(f"    min align p5                    {mc['align_p5']:6.3f}      {ms['align_p5']:6.3f}")
    print(f"    align<0.5 time (%)              {mc['align_lt05']:6.1f}      {ms['align_lt05']:6.1f}")
    print(f"    mixed combos (%)                {mc['mixed']:6.1f}      {ms['mixed']:6.1f}")
    print(f"    自旋反转帧 (%)                  {rv_c['rev_pct']:6.2f}      {rv_s['rev_pct']:6.2f}")
    print(f"    自旋反转起始                    {rv_c['rev_onsets']:6d}      {rv_s['rev_onsets']:6d}")


def analyze_parameter_sweep(df: pd.DataFrame, base: dict) -> None:
    """单变量扫代价系数，观察决策改变率/切换数/转角/力矩/自旋反转。"""
    state = df[f"I{CH['state']}"].to_numpy()
    run = state == STATE_RUN
    base_sim = simulate_cost(df, **base)
    base_combo = base_sim["combo"]
    fail_onset = reversal_stats(df, base_sim)["rev_onsets"]
    print(f"  [sweep] base: cr={base['cr']:.2f} cw={base['cw']:.2f} cd={base['cd']:.2f} "
          f"margin={base['margin']:.2f}  反转起始={fail_onset}")
    print("    参数        | switch  改变%  travel_med  taue_med  align_p5  mixed%  rev_onset")
    cases = [(f"cr={x:.2f}", dict(cr=x)) for x in (0.15, 0.25, 0.35, 0.50, 0.70)]
    cases += [(f"cw={x:.2f}", dict(cw=x)) for x in (0.05, 0.10, 0.20, 0.40)]
    cases += [(f"cd={x:.2f}", dict(cd=x)) for x in (0.0, 0.15, 0.30)]
    cases += [
        ("仅角度", dict(cw=0.0, cd=0.0)),
        ("仅力矩", dict(cr=0.0, cd=0.0)),
        ("角度重(0.7/0.05)", dict(cr=0.70, cw=0.05)),
        ("力矩重(0.1/0.4)", dict(cr=0.10, cw=0.40)),
    ]
    for label, kw in cases:
        args = dict(base)
        args.update(kw)
        sim = simulate_cost(df, **args)
        mask = run & (sim["combo"] >= 0)
        m = selection_metrics(sim, mask)
        agree = float(np.mean(sim["combo"][mask] == base_combo[mask]) * 100)
        rev = reversal_stats(df, sim)
        print(f"    {label:10s} | {m['switches']:6d}  {100 - agree:5.1f}%  {m['travel_med']:10.3f}  "
              f"{m['taue_med']:8.3f}  {m['align_p5']:7.3f}  {m['mixed']:5.1f}  {rev['rev_onsets']:9d}")


def summarize(path: Path, df: pd.DataFrame, prep: dict, base: dict, do_sweep: bool = False) -> None:
    n = len(df)
    t = prep["t_s"]
    run = df[f"I{CH['state']}"].to_numpy() == STATE_RUN
    locked = df[f"I{CH['locked']}"].to_numpy() > 0.5
    combo = df[f"I{CH['combo']}"].to_numpy().astype(int)
    cost = np.column_stack([df[f"I{CH[f'cost{k}']}"].to_numpy() for k in range(4)])

    print(f"\n=== {path.name} ===")
    print(f"rows={n}  dur={t[-1] - t[0]:.2f}s  period={prep['period_s'] * 1e3:.2f}ms  "
          f"dropped={prep['dropped']}  segs={len(prep['segs'])}")
    print(f"RUN={run.sum()} ({run.mean() * 100:.1f}%)  locked={locked.sum()} "
          f"({locked.mean() * 100:.1f}%)  RUN&locked={(run & locked).sum()}")

    for name in ["vx_cmd", "vy_cmd", "w_cmd", "fx", "fy", "mw",
                 "torque_l", "torque_r", "align_l", "align_r"]:
        x = df[f"I{CH[name]}"].to_numpy()[run]
        if x.size:
            print(f"  {name:12s} med={np.median(x):8.3f} p5={np.percentile(x, 5):8.3f} "
                  f"p95={np.percentile(x, 95):8.3f} min={x.min():8.3f} max={x.max():8.3f}")

    if run.any():
        c = combo[run]
        print("  combo counts:", {int(k): int(v) for k, v in zip(*np.unique(c, return_counts=True))})
        sw = np.flatnonzero(np.diff(c) != 0)
        print(f"  combo switches={len(sw)}  ({len(sw) / max(t[-1] - t[0], 1e-9):.2f}/s)")
        for k in range(4):
            x = cost[run, k]
            x = x[np.isfinite(x) & (x < 1e20)]
            if x.size:
                print(f"  cost{k}: med={np.median(x):7.3f} p5={np.percentile(x, 5):7.3f} "
                      f"p95={np.percentile(x, 95):7.3f} max={x.max():7.3f}")
        # 当前 combo 与最优 combo 的代价差（只统计 RUN 且未锁定）
        valid = run & ~locked
        if valid.any():
            cv = combo[valid]
            cc = cost[valid]
            cur = cc[np.arange(len(cv)), cv]
            best = cc.min(axis=1)
            gap = cur - best
            print(f"  cost(current)-min: med={np.median(gap):.3f} p95={np.percentile(gap, 95):.3f} "
                  f"max={gap.max():.3f}  >{0.15:.2f}s:{(gap > 0.15).mean() * 100:.1f}%")
    validate_reconstruction(df)

    # 全算法复现（含迟滞/无方向 keep/锁定），先确认与日志一致
    sim_cost = simulate_cost(df, **base)
    sim_short = simulate_shortest(df)
    state = df[f"I{CH['state']}"].to_numpy()
    logged = df[f"I{CH['combo']}"].to_numpy().astype(int)
    m = (state == STATE_RUN) & (logged >= 0) & (sim_cost["combo"] >= 0)
    if m.any():
        agree = float(np.mean(sim_cost["combo"][m] == logged[m]) * 100)
        print(f"  [sim] 复现日志 combo 一致率={agree:.2f}%  ({m.sum()} 帧)")

    analyze_dynamics(df)
    analyze_components(df, base["cr"], base["cw"], base["cd"])
    analyze_deviations(df, sim_cost, base["cr"], base["cw"], base["cd"])
    analyze_mixed(df, sim_cost)
    analyze_lock(df)
    analyze_hysteresis(df, sim_cost)
    analyze_vs_shortest(df, sim_cost, sim_short)
    if do_sweep:
        analyze_hysteresis_sweep(df, base)
        analyze_parameter_sweep(df, base)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", nargs="+", type=Path)
    ap.add_argument("--sweep", action="store_true", help="额外做参数扫描（较慢）")
    ap.add_argument("--cr", type=float, default=COST_RUDDER, help="舵向代价系数 (s/rad)")
    ap.add_argument("--cw", type=float, default=COST_WHEEL, help="力矩代价系数 (s/(N·m))")
    ap.add_argument("--cd", type=float, default=COST_DIFF, help="两舵反向惩罚 (s)")
    ap.add_argument("--margin", type=float, default=SWITCH_MARGIN, help="切换迟滞 (s)")
    ap.add_argument("--torque-lock", type=float, default=TORQUE_LOCK, help="锁定阈值 (N·m)")
    args = ap.parse_args()
    base = dict(cr=args.cr, cw=args.cw, cd=args.cd,
                margin=args.margin, torque_lock=args.torque_lock)
    for path in args.csv:
        df, prep = prepare(path)
        summarize(path, df, prep, base, do_sweep=args.sweep)


if __name__ == "__main__":
    main()
