#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分层分配实车日志分析（appdrvlib_chassis）。

回答的问题：

  1) 日志里到底处在哪个分配区：静止 / 纯偏航 / 纯横移 / 斜向？
  2) 慢层 toe trim 的**需求量**和**权限**差多少？多少时间被 ±TRIM_MAX 卡死？
  3) 目标舵角每拍跳 180° 是分支翻转，还是 ψ 本身在跳？
  4) 把日志里的 (fx, fy, mz) 与**实际舵角**回放进 Python 镜像，
     分层分配器实际能交付多少 mz，残差多大？
  5) 指令 w 与实测 w 的跟踪误差（区分"分配器没权限"和"命令本身太猛"）。

列映射与 lib/lib_chassis/tools/analyze_force_alloc.py 一致（app_cfg.h 当前布局）。

用法：
    python3 analyze_layered.py ../../../../ignore/new/效果非常差.csv
    python3 analyze_layered.py a.csv b.csv --bins
    python3 analyze_layered.py a.csv --trim-max-deg 30 --cos-min 0.3
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verify_alloc as va  # noqa: E402  (同目录的 Python 镜像)

# ---------------------------------------------------------------- 通道
CH_TGT_L, CH_ACT_L = 1, 2
CH_TGT_R, CH_ACT_R = 3, 4
CH_COMBO = 9
CH_LOCKED = 14
CH_STATE = 15
CH_ALIGN_L, CH_ALIGN_R = 16, 17
CH_VX_CMD, CH_VY_CMD, CH_W_CMD = 18, 19, 20
CH_VX_FB, CH_VY_FB, CH_W_FB = 21, 22, 23
CH_FX, CH_FY, CH_MZ = 24, 25, 26
CH_WS_L, CH_WS_R = 27, 28

POS_Y_DEFAULT = 0.20225
RADIUS_DEFAULT = 0.05025


def load_csv(path):
    """VOFA CSV -> dict。丢掉录数时偶发多插的首行（CH16 align > 1）。"""
    raw = np.loadtxt(path, delimiter=",", skiprows=1)
    if raw.ndim != 2 or raw.shape[1] < 29:
        raise SystemExit(f"✗ {path}: 列数不足（{raw.shape}）")
    if abs(raw[0, CH_ALIGN_L]) > 1.01 or abs(raw[0, 0]) > 1e15:
        raw = raw[1:]
    t = (raw[:, 0] - raw[0, 0]) / 1e6
    if np.any(np.diff(t) <= 0):
        raise SystemExit(f"✗ {path}: 时间戳非严格递增")
    return t, raw


def wrap(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def unwrap_rad(deg):
    return np.unwrap(np.radians(deg))


def pctl(x, q):
    return float(np.percentile(x, q))


def describe(name, x, unit="", q=(50, 90, 95, 99)):
    parts = [f"med {np.median(x):8.3f}"]
    parts += [f"p{q_:.0f} {np.percentile(x, q_):8.3f}" for q_ in q[1:]]
    print(f"  {name:22s} " + "  ".join(parts) + f"  max {np.max(x):8.3f} {unit}")


def analyze(path, args):
    t, a = load_csv(path)
    run = a[:, CH_STATE] == args.run_state
    if run.sum() < 20:
        print(f"\n{path}: RUN 帧只有 {run.sum()}，跳过")
        return

    dt = np.diff(t)
    fs = 1.0 / np.median(dt)
    aL = unwrap_rad(a[:, CH_ACT_L])
    aR = unwrap_rad(a[:, CH_ACT_R])
    alpha = np.column_stack([aL, aR])
    tgt = np.column_stack([unwrap_rad(a[:, CH_TGT_L]), unwrap_rad(a[:, CH_TGT_R])])
    fx, fy, mz = a[:, CH_FX], a[:, CH_FY], a[:, CH_MZ]
    F = np.hypot(fx, fy)
    cos_psi = np.where(F > 1e-6, fx / np.maximum(F, 1e-9), 1.0)
    sin_psi = np.where(F > 1e-6, fy / np.maximum(F, 1e-9), 0.0)
    w_cmd, w_fb = a[:, CH_W_CMD], a[:, CH_W_FB]
    vx_cmd, vy_cmd = a[:, CH_VX_CMD], a[:, CH_VY_CMD]

    print("\n" + "=" * 92)
    print(f"{path}  ({len(a)} 帧，RUN {run.sum()}，{t[-1]:.2f}s，{fs:.0f}Hz)")

    # ---------------------------------------------------------------- 1) 工况
    print("\n[1] 分配区分布（RUN）")
    m = run
    force_min = args.force_min
    mz_min = args.mz_min
    idle = (F < force_min) & (np.abs(mz) <= mz_min)
    yaw_only = (F < force_min) & (np.abs(mz) > mz_min)
    lateral = (~idle) & (~yaw_only) & (np.abs(cos_psi) < args.cos_min)
    forward = (~idle) & (~yaw_only) & (np.abs(cos_psi) >= args.cos_min)
    n = m.sum()
    for nm, mask in (("静止（保持）", idle), ("纯偏航（差模）", yaw_only),
                     ("横移/斜向（toe 慢层）", lateral), ("前进/后退（快层）", forward)):
        print(f"  {nm:18s} {np.mean(mask[m]) * 100:5.1f} %  "
              f"(F med {np.median(F[m & mask]) if (m & mask).any() else 0:6.2f} N)")

    print("\n  指令幅值：")
    describe("|w_cmd|", np.abs(w_cmd[m]), "rad/s")
    describe("|w_cmd| 中 >0.5 占比", np.array([np.mean(np.abs(w_cmd[m]) > 0.5)]))
    print(f"  |w_cmd|>0.5 时间占比 {np.mean(np.abs(w_cmd[m]) > 0.5) * 100:5.1f} %，"
          f">1.5 占 {np.mean(np.abs(w_cmd[m]) > 1.5) * 100:5.1f} %")
    describe("|vx_cmd|", np.abs(vx_cmd[m]), "m/s")
    describe("|vy_cmd|", np.abs(vy_cmd[m]), "m/s")

    # ---------------------------------------------------------------- 2) trim 权限
    print("\n[2] 慢层 toe trim：需求 vs 权限")
    fast_den = 1.0 - args.cos_min
    w_fast = np.clip((np.abs(cos_psi) - args.cos_min) / fast_den, 0.0, 1.0)
    w_fast = np.where(yaw_only | idle, 1.0, w_fast)
    mz_slow_ss = np.where(yaw_only | idle, 0.0, (1.0 - w_fast) * mz)
    # 稳态需求（不含 LPF/限速）：trim = atan(mz_slow/(a·F·sinψ))
    denom = args.pos_y * F * sin_psi
    usable = (~yaw_only) & (~idle) & (np.abs(sin_psi) > args.trim_sin_min) & (np.abs(denom) > 1e-4)
    trim_req = np.zeros_like(mz)
    trim_req[usable] = np.arctan(mz_slow_ss[usable] / denom[usable])
    trim_max = math.radians(args.trim_max_deg)
    sat = np.abs(trim_req) >= trim_max
    print(f"  |trim_req|: med {np.degrees(np.median(np.abs(trim_req[m]))):6.2f}°  "
          f"p95 {np.degrees(pctl(np.abs(trim_req[m]), 95)):6.2f}°  "
          f"max {np.degrees(np.max(np.abs(trim_req[m]))):6.2f}°")
    sat_frac = np.mean(sat[m & usable]) * 100 if (m & usable).any() else 0.0
    print(f"  被 ±{args.trim_max_deg:.0f}° 限幅的时间占比（可 trim 区）: {sat_frac:5.1f} %")
    # 该区里有需求但 sinψ 太小、toe 无权
    no_auth = np.mean(m & lateral & ~usable) * 100 if (m & lateral & ~usable).any() else 0.0
    print(f"  在慢层区但 |sinψ|≤{args.trim_sin_min}: "
          f"{no_auth:5.1f} % (toe 无权限、快层又为 0 ⇒ 偏航无执行器)")
    # 慢层请求的 mz 与 ±trim_max 能交付的 mz
    mz_cap = np.abs(args.pos_y * F * sin_psi * math.tan(trim_max))
    need = np.abs(mz_slow_ss)
    print(f"  |mz_slow| : med {np.median(need[m]):6.3f}  p95 {pctl(need[m], 95):6.3f} N·m")
    print(f"  |mz_cap|  : med {np.median(mz_cap[m]):6.3f}  p95 {pctl(mz_cap[m], 95):6.3f} N·m")
    print(f"  需求/权限 >1 的时间占比: {np.mean(need[m] > mz_cap[m]) * 100:5.1f} %"
          f"（这些时刻慢层已饱和，实际交付不了）")

    # ---------------------------------------------------------------- 3) 分支
    print("\n[3] 分支 / 目标跳变")
    combo = np.round(a[:, CH_COMBO]).astype(int)
    print(f"  CH9 combo 分布: " + ", ".join(
        f"{v}:{np.sum(combo[m] == v)}" for v in np.unique(combo[m])))
    branch = (combo & 1).astype(int)
    flips = np.where(np.diff(branch) != 0)[0]
    flips = flips[run[flips] & run[flips + 1]]
    print(f"  分支翻转 {len(flips)} 次 / {t[m][-1] - t[m][0]:.1f}s")
    if len(flips):
        # 抖动式翻转：相邻两次翻转间隔 < 30ms（100Hz 日志里就是连续帧）
        if len(flips) > 1:
            gap_ms = np.diff(t[flips]) * 1000.0
            print(f"    翻转间隔: med {np.median(gap_ms):6.0f}ms  "
                  f"<30ms 的 {np.sum(gap_ms < 30)} 次；"
                  f"<100ms 的 {np.sum(gap_ms < 100)} 次")
        # 翻转附近力很小（F 在 FORCE_MIN 附近）的比例
        Fnear = np.array([np.min(F[max(0, k - 2):k + 3]) for k in flips])
        print(f"    翻转附近 min(F) < 2·FORCE_MIN 的: {np.sum(Fnear < 2 * force_min)} / {len(flips)}")
        print("    前 12 次分支翻转（d = wrap(ψ − 连续角 mid)）：")
        psi_f = np.arctan2(fy, fx)
        for k in flips[:12]:
            d0 = np.degrees(wrap(psi_f[k] - 0.5 * (aL[k] + aR[k])))
            d1 = np.degrees(wrap(psi_f[k + 1] - 0.5 * (aL[k + 1] + aR[k + 1])))
            print(f"      t={t[k]:6.2f}  {branch[k]}→{branch[k + 1]}  "
                  f"d {d0:7.1f}→{d1:7.1f}°  F {F[k]:5.1f}N  "
                  f"αL/R {np.degrees(aL[k]):6.1f}/{np.degrees(aR[k]):6.1f}°")
    jump = np.abs(np.diff(tgt, axis=0))
    big = (jump > np.radians(90)) & run[1:, None] & run[:-1, None]
    print(f"  目标单拍跳变 >90°: L {np.sum(big[:, 0])} 次, R {np.sum(big[:, 1])} 次")
    if len(flips):
        near = sum(np.any(np.abs(flips - k) <= 1) for k in np.where(big[:, 0])[0])
        print(f"    其中目标 L 跳变与分支翻转同一拍/相邻拍: {near} / {np.sum(big[:, 0])}")
    # 连续角表示：两轮"同一物理方向"却相差 360° 时，mid=(aL+aR)/2 会错 180°
    pairs = np.abs(wrap(aL - aR))
    print(f"  |wrap(αL−αR)|: med {np.degrees(np.median(pairs[m])):5.1f}°  "
          f"p95 {np.degrees(pctl(pairs[m], 95)):6.1f}°  max {np.degrees(np.max(pairs[m])):6.1f}°")
    vec_mid = np.arctan2(np.sin(aL) + np.sin(aR), np.cos(aL) + np.cos(aR))
    mid_log = 0.5 * (aL + aR)
    confuse = np.abs(wrap(mid_log - vec_mid)) > np.radians(45)
    print(f"  连续角 mid 与矢量 mid 相差>45° 的帧: {np.sum(confuse[m])} "
          f"({np.mean(confuse[m]) * 100:.1f} %) ⇒ 分支决策可能被 360° 表示干扰")
    ev = np.where(big[:, 0] & run[:-1])[0][:8]
    if len(ev):
        print("  前几次目标 L 跳变：")
        for k in ev:
            print(f"    t={t[k]:6.2f}  Δ={np.degrees(jump[k, 0]):7.1f}°  "
                  f"分支 {branch[k]}→{branch[k + 1]}  "
                  f"αL {np.degrees(aL[k]):7.1f}→{np.degrees(aL[k + 1]):7.1f}  "
                  f"αR {np.degrees(aR[k]):7.1f}→{np.degrees(aR[k + 1]):7.1f}  "
                  f"ψ {np.degrees(math.atan2(fy[k], fx[k])):7.1f}  "
                  f"mid/vec Δ {np.degrees(wrap(mid_log[k] - vec_mid[k])):6.1f}°")
    # ψ（力方向）本身的单拍跳变：>90° 时 base 直接翻半圈
    psi_all = np.arctan2(fy, fx)
    psi_jump = np.degrees(np.abs(wrap(np.diff(psi_all))))
    psi_jump[~((F[1:] > force_min) & (F[:-1] > force_min))] = 0.0
    psi_big = (psi_jump > 90.0) & run[1:] & run[:-1]
    print(f"  ψ(=atan2(fy,fx)) 单拍跳变 >90°: {np.sum(psi_big)} 次"
          f"（对应力方向在 F>{force_min:.1f}N 时翻转半圈）")
    # 对 ψ 的跳变：目标减去分支后应该等于 ψ±trim
    base_log = tgt[:, 0] - branch * np.pi
    base_jump = np.degrees(np.abs(wrap(np.diff(base_log))))
    base_jump = base_jump * (run[1:] & run[:-1])
    print(f"  扣掉分支后的基础角单拍跳变: med {np.median(base_jump[base_jump > 0.01]):6.2f}°  "
          f"p95 {pctl(base_jump[base_jump > 0.01], 95):7.2f}°  max {np.max(base_jump):8.2f}°")

    # ---------------------------------------------------------------- 4) 回放
    print("\n[4] 按日志实际舵角回放分层分配器（Python 镜像）")
    va.FORCE_MIN = args.force_min
    va.MZ_MIN = args.mz_min
    va.TRIM_LPF_RC = args.trim_rc
    va.TRIM_MAX = trim_max
    va.TRIM_SLEW = math.radians(args.trim_slew_dps)
    va.TRIM_SIN_MIN = args.trim_sin_min
    va.FAST_COS_MIN = args.cos_min
    va.FORCE_MAX = args.force_max
    va.BRANCH_MARGIN = math.radians(args.branch_margin_deg)
    cfg = {"a": args.pos_y, "radius": args.radius}
    st = va.State()
    rec = np.zeros((len(a), 5))  # trim, mz_fast, mz_slow, delivered_mz, delivered_xy
    trims = np.zeros(len(a))
    tgt_mirror = np.zeros((len(a), 2))
    for k in range(len(a)):
        out = va.allocate(st, cfg, [alpha[k, 0], alpha[k, 1]], fx[k], fy[k], mz[k],
                          dt[k - 1] if k else 0.0)
        trims[k] = out["trim"]
        tgt_mirror[k] = out["target"]
        dx, dy, dz = va.wrench_from_out(out, alpha[k], cfg)
        rec[k] = (out["trim"], out["mz_fast"], out["mz_slow"], dz, math.hypot(dx, dy))
    trim_deg = np.degrees(trims)
    mismatch = np.abs(np.degrees(wrap(tgt_mirror[:, 0] - tgt[:, 0])))
    print(f"  镜像目标 vs 日志目标: med {np.median(mismatch[m]):5.2f}°  "
          f"p95 {pctl(mismatch[m], 95):6.2f}°  >30° 占 {np.mean(mismatch[m] > 30) * 100:5.1f} %"
          f"（≈0 ⇒ 板上跑的就是这套逻辑）")
    print(f"  镜像 trim: med|{np.median(np.abs(trim_deg[m])):5.2f}|°  "
          f"p95|{pctl(np.abs(trim_deg[m]), 95):5.2f}|°  "
          f"饱和(|trim|≥{args.trim_max_deg:.0f}°) 占 {np.mean(np.abs(trim_deg[m]) >= args.trim_max_deg - 0.1) * 100:5.1f} %")
    res_mz = mz - rec[:, 3]
    res_xy = F - rec[:, 4]
    print(f"  mz 交付残差: med {np.median(res_mz[m]):7.3f}  p95|{pctl(np.abs(res_mz[m]), 95):7.3f}| "
          f"N·m （期望交付 mz={np.median(mz[m]):.2f}）")
    print(f"  平移力交付残差 |F−F_out|: med {np.median(np.abs(res_xy[m])):6.3f}  "
          f"p95 {pctl(np.abs(res_xy[m]), 95):6.3f} N")
    align = np.minimum(a[:, CH_ALIGN_L], a[:, CH_ALIGN_R])
    print(f"  最小对齐 cos: med {np.median(align[m]):5.3f}  p5 {pctl(align[m], 5):6.3f}  "
          f"<0.5 占 {np.mean(align[m] < 0.5) * 100:5.1f} %  <0 占 {np.mean(align[m] < 0) * 100:5.1f} %")

    # ---------------------------------------------------------------- 5) yaw 跟踪
    print("\n[5] yaw 跟踪")
    err = w_cmd - w_fb
    print(f"  w 误差: std {np.std(err[m]):.3f} rad/s，"
          f"|e|>0.5 占 {np.mean(np.abs(err[m]) > 0.5) * 100:5.1f} %，"
          f"|e|>1 占 {np.mean(np.abs(err[m]) > 1) * 100:5.1f} %")
    # 纯横移 + 有偏航命令的窗口
    vyonly = m & (np.abs(vx_cmd) < 0.15) & (np.abs(vy_cmd) > 0.3)
    if vyonly.any():
        print(f"  其中纯横移帧 {vyonly.sum()}：|e| med {np.median(np.abs(err[vyonly])):.3f}，"
              f"std {np.std(err[vyonly]):.3f} rad/s")
    # 再按有没有偏航命令拆开：只有 w_cmd≈0 才是"横移漂移"本身
    quiet_lat = vyonly & (np.abs(w_cmd) < 0.10)
    active_lat = vyonly & (np.abs(w_cmd) >= 0.10)
    for nm, mask in (("纯横移+w≈0", quiet_lat), ("纯横移+有w命令", active_lat)):
        if mask.sum() > 10:
            print(f"    {nm:12s} {mask.sum():5d} 帧: w_fb std {np.std(w_fb[mask]):6.3f}  "
                  f"|e| med {np.median(np.abs(err[mask])):6.3f}  "
                  f"mz med {np.median(mz[mask]):6.2f} p95|{pctl(np.abs(mz[mask]), 95):6.2f}| N·m")
    # 纯前进 + 有偏航命令（快层能不能扛）
    fwd = m & (np.abs(vx_cmd) > 0.3) & (np.abs(vy_cmd) < 0.15)
    if fwd.any():
        print(f"    纯前进帧 {fwd.sum():5d}: |e| med {np.median(np.abs(err[fwd])):6.3f}  "
              f"std {np.std(err[fwd]):6.3f}  mz p95|{pctl(np.abs(mz[fwd]), 95):6.2f}| N·m")

    # 连续的"稳态纯横移、w=0"段：这是新旧方案唯一能公平对比的工况
    steady = quiet_lat & (np.abs(vx_cmd) < 0.15) & (np.abs(vy_cmd) > 0.3)
    runs, start = [], None
    for k in range(len(a) + 1):
        on = k < len(a) and steady[k]
        if on and start is None:
            start = k
        elif not on and start is not None:
            if (k - start) >= int(fs):
                runs.append((start, k))
            start = None
    if runs:
        print("\n  稳态纯横移段（|w_cmd|<0.1，≥1s）：")
        print("     区间[s]       vy   w_fb std  |e|med  toe std  mz p95 | mz_cap med/p95")
        for s, e in runs:
            cap = np.abs(args.pos_y * F[s:e] * sin_psi[s:e] * math.tan(trim_max))
            psi_t = np.arctan2(vy_cmd[s:e], vx_cmd[s:e])
            # 两轮相对运动方向的 toe：先折 180°（同一物理方向的正/反向等价），再相减
            fold = lambda x: (x + np.pi / 2) % np.pi - np.pi / 2
            toe = np.degrees(fold(aL[s:e] - psi_t) - fold(aR[s:e] - psi_t))
            print(f"    {t[s]:5.2f}~{t[e - 1]:5.2f} {np.median(vy_cmd[s:e]):6.2f}  "
                  f"{np.std(w_fb[s:e]):7.3f}  {np.median(np.abs(err[s:e])):6.3f}  "
                  f"{np.std(toe):7.2f}°  {pctl(np.abs(mz[s:e]), 95):6.2f} | "
                  f"{np.median(cap):5.2f}/{pctl(cap, 95):5.2f} N·m")

    # ---------------------------------------------------------------- 6) 原始帧
    if args.dump:
        t0, t1 = args.dump
        print(f"\n[6] 原始帧 {t0:.2f}~{t1:.2f}s")
        print("     t | state combo | tgtL tgtR actL actR | alignL/R | fx fy mz | vxc vyc wc")
        for k in np.where((t >= t0) & (t <= t1))[0]:
            print(f"  {t[k]:6.2f} | {a[k, CH_STATE]:3.0f} {combo[k]:3d} | "
                  f"{a[k, CH_TGT_L]:6.1f} {a[k, CH_TGT_R]:6.1f} "
                  f"{np.degrees(aL[k]):6.1f} {np.degrees(aR[k]):6.1f} | "
                  f"{a[k, CH_ALIGN_L]:5.2f}/{a[k, CH_ALIGN_R]:5.2f} | "
                  f"{fx[k]:6.1f} {fy[k]:6.1f} {mz[k]:6.2f} | "
                  f"{vx_cmd[k]:5.2f} {vy_cmd[k]:5.2f} {w_cmd[k]:5.2f}")

    # ---------------------------------------------------------------- 7) 分箱
    if args.bins:
        print("\n[6] 0.5s 分箱（RUN）")
        print("   t | vx vy wcmd wfb | F mz | trimReq trimAct | 区 | align")
        step = max(1, int(fs / 2))
        for s in range(0, len(a), step):
            e = min(s + step, len(a))
            if run[s:e].sum() < step // 4:
                continue
            reg = ("idle" if idle[s:e].mean() > 0.5 else
                   "yaw" if yaw_only[s:e].mean() > 0.5 else
                   "lat" if lateral[s:e].mean() > 0.5 else "fwd")
            print(f"  {t[s]:5.2f} | {vx_cmd[s:e].mean():5.2f} {vy_cmd[s:e].mean():5.2f} "
                  f"{w_cmd[s:e].mean():5.2f} {w_fb[s:e].mean():5.2f} | "
                  f"{F[s:e].mean():5.1f} {mz[s:e].mean():6.2f} | "
                  f"{np.degrees(trim_req[s:e]).mean():7.1f} {trim_deg[s:e].mean():7.1f} | "
                  f"{reg:4s} | {align[s:e].mean():5.2f}")


def main():
    ap = argparse.ArgumentParser(description="分层分配实车日志分析")
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--run-state", type=int, default=3, help="CH15 状态值")
    ap.add_argument("--pos-y", type=float, default=POS_Y_DEFAULT)
    ap.add_argument("--radius", type=float, default=RADIUS_DEFAULT)
    # 与 appdrvlib_chassis.h 默认值一致
    ap.add_argument("--force-min", type=float, default=1.0)
    ap.add_argument("--mz-min", type=float, default=0.05)
    ap.add_argument("--trim-rc", type=float, default=0.20)
    ap.add_argument("--trim-max-deg", type=float, default=30.0)
    ap.add_argument("--trim-slew-dps", type=float, default=60.0)
    ap.add_argument("--trim-sin-min", type=float, default=0.30)
    ap.add_argument("--cos-min", type=float, default=0.30)
    ap.add_argument("--force-max", type=float, default=100.0)
    ap.add_argument("--branch-margin-deg", type=float, default=10.0)
    ap.add_argument("--bins", action="store_true", help="打印 0.5s 分箱")
    ap.add_argument("--dump", type=float, nargs=2, default=None, metavar=("T0", "T1"),
                    help="打印该时间窗的原始帧")
    args = ap.parse_args()
    for p in args.csv:
        analyze(p, args)


if __name__ == "__main__":
    main()
