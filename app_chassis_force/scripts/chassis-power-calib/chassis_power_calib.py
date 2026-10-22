#!/usr/bin/env python3
"""半舵底盘驱动轮功率标定/分析工具（VOFA CSV）。

两个用途：
  1. `--static` 静态功耗标定：从底盘静止日志里统计 P_meas，给出 k3；
  2. 默认模式：对账 P_fb / P_cmd 与 P_meas，检查门控接受率与 k1/k2 收敛。

VOFA 列约定（app_cfg.h）：列 k = 通道 k（列 0 = I0 时间戳，单位为 us）：
  CH1 limit_eff / CH2 P_before / CH3 P_after / CH4 scale_min（RLS 布局）
  CH29 P_meas / CH30 P_fb_lpf / CH31 P_cmd / CH32 P_fb_raw
  CH33 k1     / CH34 k2       / CH35 innovation / CH36 gate_mask

示例：
  python chassis_power_calib.py --csv ~/log/静止功耗.csv --static
  python chassis_power_calib.py --csv ~/log/跑车.csv --residual
  python chassis_power_calib.py --csv ~/log/跑车.csv --limit-check
"""

import argparse
import sys

import numpy as np
import pandas as pd

# 通道号（= 列头 I<k>）→ 含义
CH = {
    # 两种 VOFA 布局共用
    "tau_cmd_l": 5,
    "tau_l": 6,
    "tau_cmd_r": 7,
    "tau_r": 8,
    "vx_cmd": 18,
    "vy_cmd": 19,
    "w_cmd": 20,
    "vx": 21,
    "vy": 22,
    "w": 23,
    "w_l": 27,
    "w_r": 28,
    "p_meas": 29,
    "p_fb_lpf": 30,
    "p_cmd": 31,
    "p_fb_raw": 32,
    "k1": 33,
    "k2": 34,
    "innov": 35,
    "gate": 36,
}

# 仅 RLS 调参布局（CHASSIS_VOFA_DEBUG_CHASSIS=0）存在的通道；旧日志没有
OPT_CH = {
    "limit_eff": 1,
    "p_before": 2,
    "p_after": 3,
    "scale_min": 4,
    "reg_w_lpf": 9,
    "reg_t2_lpf": 10,
    "reg_w2_lpf": 11,
    "mech_lpf": 12,
    "mech_gain": 13,
    "upd_count": 14,
    "reg_w_raw": 24,
    "reg_t2_raw": 25,
    "mech_raw": 26,
}

GATE_RUN = 1 << 0
GATE_WHEEL_FRESH = 1 << 1
GATE_SUPERCAP_OK = 1 << 2
GATE_SUPERCAP_NEW = 1 << 3
GATE_POWER_RANGE = 1 << 4
GATE_MOTION = 1 << 5
GATE_TORQUE = 1 << 6
GATE_NO_REVERSAL = 1 << 7
GATE_FINITE = 1 << 8
GATE_WARMUP = 1 << 9
GATE_PERIOD = 1 << 10
GATE_ALL = 0x07FF

GATE_NAMES = [
    (GATE_RUN, "RUN"),
    (GATE_WHEEL_FRESH, "WHEEL_FRESH"),
    (GATE_SUPERCAP_OK, "SUPERCAP_OK"),
    (GATE_SUPERCAP_NEW, "SUPERCAP_NEW"),
    (GATE_POWER_RANGE, "POWER_RANGE"),
    (GATE_MOTION, "MOTION"),
    (GATE_TORQUE, "TORQUE"),
    (GATE_NO_REVERSAL, "NO_REVERSAL"),
    (GATE_FINITE, "FINITE"),
    (GATE_WARMUP, "WARMUP"),
    (GATE_PERIOD, "PERIOD"),
]


def load(path):
    df = pd.read_csv(path)
    need = {f"I{ch}" for ch in CH.values()} | {"I0"}
    missing = need - set(df.columns)
    if missing:
        sys.exit(f"CSV 缺少列：{sorted(missing)}\n实际列：{list(df.columns)[:8]} ...")
    out = pd.DataFrame({"t_us": df["I0"].astype(np.float64)})
    for name, ch in CH.items():
        out[name] = df[f"I{ch}"].astype(np.float64)
    for name, ch in OPT_CH.items():
        col = f"I{ch}"
        out[name] = df[col].astype(np.float64) if col in df.columns else np.nan
    return out


def detect_layout(df):
    """auto 判断 CH9~14 是 RLS 回归量还是底盘调试量。

    可靠判据：RLS 布局的 CH14 是 update_count（单调不减、值域大）；
    调试布局是 locked（0/1 常量）。
    """
    v = df["upd_count"].to_numpy()
    if np.isnan(v).all():
        return "debug"
    finite = v[np.isfinite(v)]
    if finite.size and np.all(np.diff(finite) >= -1e-6) and len(np.unique(finite)) > 2:
        return "rls"
    return "debug"


def describe(x, name, unit=""):
    x = np.asarray(x, dtype=np.float64)
    if x.size == 0:
        print(f"  {name}: 无样本")
        return
    q = np.percentile(x, [1, 5, 50, 95, 99])
    print(
        f"  {name:<10} n={x.size:<6} mean={x.mean():9.3f} std={x.std(ddof=1) if x.size > 1 else 0:8.3f} "
        f"min={x.min():9.3f} p1={q[0]:9.3f} p5={q[1]:9.3f} med={q[2]:9.3f} "
        f"p95={q[3]:9.3f} p99={q[4]:9.3f} max={x.max():9.3f} {unit}"
    )


def gate_report(df):
    mask = df["gate"].round().astype(np.int64)
    n = len(mask)
    print(f"\n[门控] 共 {n} 帧")
    for bit, name in GATE_NAMES:
        cnt = int(((mask & bit) != 0).sum())
        print(f"  bit{bit.bit_length() - 1:<2} {name:<14} 通过 {cnt / n * 100:6.2f}%")
    full = int((mask == GATE_ALL).sum())
    print(f"  全部通过（会触发更新）      {full / n * 100:6.2f}%  ({full} 帧)")


def static_k3(df, args):
    # 静止判据：RUN 且 MOTION 未通过；默认还要求轮新鲜 + 超电正常
    mask = df["gate"].round().astype(np.int64)
    sel = (mask & GATE_RUN) != 0
    sel &= (mask & GATE_MOTION) == 0
    if args.require_fresh:
        sel &= (mask & GATE_WHEEL_FRESH) != 0
        sel &= (mask & GATE_SUPERCAP_OK) != 0
    sub = df.loc[sel]

    print("[静态功耗标定]")
    print(f"  输入文件：{args.csv}")
    print(f"  静止样本：{len(sub)} / {len(df)} 帧")
    if len(sub) < 10:
        sys.exit("  静止样本太少，检查日志或放宽 --no-require-fresh")
    dur = (sub["t_us"].iloc[-1] - sub["t_us"].iloc[0]) * 1e-6
    print(f"  时长：{dur:.2f} s")

    p = sub["p_meas"].to_numpy()
    describe(p, "P_meas", "W")
    describe(sub["p_fb_lpf"].to_numpy(), "P_fb_lpf", "W")
    describe(sub["innov"].to_numpy(), "innovation", "W")

    # 时间漂移：线性拟合斜率（W/s），判断是否还在爬升（电容/风扇/温漂）
    t = (sub["t_us"].to_numpy() - sub["t_us"].iloc[0]) * 1e-6
    slope, intercept = np.polyfit(t, p, 1)
    print(f"  线性趋势：{slope:+.4f} W/s，截距 {intercept:.3f} W")

    # 分窗均值：看前后半段 / 四段是否稳定
    k = len(p) // 4
    if k >= 1:
        wins = [p[i * k : (i + 1) * k].mean() for i in range(4)]
        print(f"  四分窗均值：{', '.join(f'{w:.3f}' for w in wins)} W")

    # 离群剔除后均值（±3σ 迭代）
    x = p.copy()
    for _ in range(5):
        mu, sd = x.mean(), x.std(ddof=1)
        keep = np.abs(x - mu) <= 3.0 * sd
        if keep.all() or keep.sum() < 3:
            break
        x = x[keep]
    print(f"  去离群后（±3σ）：mean={x.mean():.3f} W  std={x.std(ddof=1):.3f} W  n={x.size}")
    print(f"\n  ⇒ 建议 k3 = {x.mean():.3f} W（若静息期内已使能 DCDC 且驱动失能）")


def residual(df, args):
    mask = df["gate"].round().astype(np.int64)
    sel = (mask & GATE_RUN) != 0
    if args.only_full_gate:
        sel &= mask == GATE_ALL
    if args.min_abs_p > 0:
        sel &= np.abs(df["p_meas"]) > args.min_abs_p
    sub = df.loc[sel]

    print("[预测对账]")
    print(f"  样本：{len(sub)} / {len(df)} 帧")
    if len(sub) < 10:
        sys.exit("  样本太少")
    e = (sub["p_fb_lpf"] - sub["p_meas"]).to_numpy()
    describe(e, "resid_fb", "W")
    print(f"  RMS={np.sqrt(np.mean(e**2)):.3f} W  |mean|={abs(e.mean()):.3f} W")
    e_cmd = (sub["p_cmd"] - sub["p_meas"]).to_numpy()
    describe(e_cmd, "resid_cmd", "W")
    describe(sub["p_meas"].to_numpy(), "P_meas", "W")
    print(f"  相关系数 corr(resid_fb, P_meas) = {np.corrcoef(e, sub['p_meas'])[0, 1]:+.3f}")

    print("\n[k1/k2 收敛]")
    n = len(df)
    for name in ("k1", "k2"):
        v = df[name].to_numpy()
        tail = v[int(n * 0.8) :]
        print(
            f"  {name}: 末 20% mean={tail.mean():.4f} std={tail.std(ddof=1):.4f} "
            f"min={v.min():.4f} max={v.max():.4f}"
        )
    print(f"  innovation: mean={df['innov'].mean():.3f} W std={df['innov'].std(ddof=1):.3f} W")


def ols_fit(y, X, names, label):
    """最小二乘拟合 y = X·c，打印系数/R²/条件数。"""
    X1 = np.column_stack([np.ones(len(y)), X])
    names1 = ["1"] + names
    c, *_ = np.linalg.lstsq(X1, y, rcond=None)
    pred = X1 @ c
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rms = float(np.sqrt(np.mean((y - pred) ** 2)))
    cond = float(np.linalg.cond(X1))
    print(f"  [{label}] n={len(y)}  R²={r2:.4f}  RMS={rms:.2f} W  cond={cond:.1f}")
    for nm, v in zip(names1, c):
        print(f"      {nm:<10} = {v:+10.4f}")
    return c, r2


def fit_mode(df, args):
    """离线拟合功率模型，诊断 k1/k2 可辨识性与口径。"""
    mask = df["gate"].round().astype(np.int64)
    base = (mask & GATE_RUN) != 0
    wl, wr = df["w_l"].to_numpy(), df["w_r"].to_numpy()
    tl, tr = df["tau_l"].to_numpy(), df["tau_r"].to_numpy()
    p = df["p_meas"].to_numpy()
    wcmd, vxcmd = np.abs(df["w_cmd"].to_numpy()), np.abs(df["vx_cmd"].to_numpy())

    sum_w = np.abs(wl) + np.abs(wr)
    sum_t2 = tl * tl + tr * tr
    mech = tl * wl + tr * wr

    seg = np.full(len(df), "mixed", dtype=object)
    seg[(wcmd < 0.2) & (vxcmd > 0.05)] = "straight"
    seg[(wcmd >= 0.2) & (vxcmd <= 0.05)] = "spin"

    ok = base & np.isfinite(p) & np.isfinite(sum_w) & np.isfinite(sum_t2) & np.isfinite(mech)
    ok &= np.abs(p) > args.min_abs_p
    ok &= np.isfinite(wl) & np.isfinite(wr) & np.isfinite(tl) & np.isfinite(tr)

    print("\n[分段]")
    for s in ("straight", "spin", "mixed"):
        sel = ok & (seg == s)
        if sel.sum() < 10:
            print(f"  {s:<9} n={sel.sum()}  （样本不足）")
            continue
        same = float((np.sign(tl[sel] * wl[sel]) + np.sign(tr[sel] * wr[sel]) > 0).mean())
        print(
            f"  {s:<9} n={int(sel.sum()):<5} P_meas={p[sel].mean():7.2f}±{p[sel].std():6.2f} W  "
            f"Σ|ω|={sum_w[sel].mean():7.2f}  Στ²={sum_t2[sel].mean():6.3f}  "
            f"Στω={mech[sel].mean():7.2f} W  同号率={same * 100:5.1f}%"
        )

    print("\n[驱动轮量程（判断 Kt / 绕组）]")
    for nm, v in (("ω_L", wl), ("ω_R", wr), ("τ_L", tl), ("τ_R", tr)):
        vv = v[ok]
        print(f"  {nm}: mean={vv.mean():+8.3f} std={vv.std():7.3f} min={vv.min():+9.3f} max={vv.max():+9.3f}")
    print("  （MF7015-23T 空载 805 rpm≈84.3 rad/s、峰值 3.8 N·m；10T 空载 1890 rpm≈197.9 rad/s、峰值 2.0 N·m）")

    print("\n[回归量相关（全部有效样本）]")
    print(f"  corr(Σ|ω|, Στ²) = {np.corrcoef(sum_w[ok], sum_t2[ok])[0, 1]:+.3f}")
    print(f"  corr(P_meas, Στω) = {np.corrcoef(p[ok], mech[ok])[0, 1]:+.3f}")
    print(f"  corr(P_meas, Σ|ω|) = {np.corrcoef(p[ok], sum_w[ok])[0, 1]:+.3f}")
    print(f"  corr(P_meas, Στ²) = {np.corrcoef(p[ok], sum_t2[ok])[0, 1]:+.3f}")

    print("\n[全模型 P = c0 + c1·Σ|ω| + c2·Στ² + c3·Στω]")
    for s in ("straight", "spin", "mixed"):
        sel = ok & (seg == s)
        if sel.sum() < 30:
            continue
        ols_fit(p[sel], np.column_stack([sum_w[sel], sum_t2[sel], mech[sel]]),
                ["Σ|ω|", "Στ²", "Στω"], s)

    sel = ok
    print("\n[只用在线模型的两项（k3 固定为 4.03）]")
    ols_fit(p[sel] - 4.03, np.column_stack([sum_w[sel], sum_t2[sel]]), ["Σ|ω|", "Στ²"], "all")

    print("\n[单/双项候选模型（含自由截距）]")
    sum_abs_t = np.abs(tl) + np.abs(tr)
    sum_t = tl + tr
    ols_fit(p[sel], np.column_stack([mech[sel]]), ["Στω"], "Στω")
    ols_fit(p[sel], np.column_stack([sum_t2[sel]]), ["Στ²"], "Στ²")
    ols_fit(p[sel], np.column_stack([sum_w[sel]]), ["Σ|ω|"], "Σ|ω|")
    ols_fit(p[sel], np.column_stack([sum_abs_t[sel]]), ["Σ|τ|"], "Σ|τ|")
    ols_fit(p[sel], np.column_stack([sum_t[sel]]), ["Στ"], "Στ")
    ols_fit(p[sel], np.column_stack([mech[sel], sum_abs_t[sel]]), ["Στω", "Σ|τ|"], "Στω+Σ|τ|")
    ols_fit(p[sel], np.column_stack([mech[sel], sum_t2[sel]]), ["Στω", "Στ²"], "Στω+Στ²")

    # 匀速段：相邻两拍 |Δω| 小、且转动为主，去掉加减速瞬态
    dw = np.abs(np.gradient(wl)) + np.abs(np.gradient(wr))
    cruise = ok & (dw < 1.0) & (np.abs(mech) > 5.0)
    if cruise.sum() > 30:
        print("\n[匀速段（|Δω/拍|<1 rad/s、|Στω|>5 W）]")
        ols_fit(p[cruise], np.column_stack([sum_w[cruise], sum_t2[cruise], mech[cruise]]),
                ["Σ|ω|", "Στ²", "Στω"], "cruise")

    print("\n[固定 k3=4.03、约束 k1∈[0.001,0.5] 时的最优 k2]")
    for s in ("straight", "spin", "mixed"):
        sl = ok & (seg == s)
        if sl.sum() < 30:
            continue
        best = (np.inf, 0.0, 0.0)
        for k1 in np.linspace(0.001, 0.5, 500):
            z = p[sl] - mech[sl] - 4.03 - k1 * sum_w[sl]
            den = float(np.dot(sum_t2[sl], sum_t2[sl]))
            k2 = float(np.dot(sum_t2[sl], z) / den) if den > 0 else 0.0
            k2 = float(np.clip(k2, 0.1, 50.0))
            mse = float(np.mean((z - k2 * sum_t2[sl]) ** 2))
            if mse < best[0]:
                best = (mse, k1, k2)
        print(f"  {s:<9} k1={best[1]:.4f} N·m  k2={best[2]:.3f} W/(N·m)²  RMS={np.sqrt(best[0]):.2f} W")

    print("\n[k1/k2 时序（在线 RLS 实际值）]")
    n = len(df)
    for frac in (0.0, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99):
        i = min(int(n * frac), n - 1)
        print(
            f"  t={frac * 100:5.1f}%  k1={df['k1'].iloc[i]:8.5f}  k2={df['k2'].iloc[i]:8.3f}  "
            f"P_meas={df['p_meas'].iloc[i]:8.2f}  mask={int(df['gate'].iloc[i])}"
        )

    # 稳态窗：消除 P_meas 内部低通与轮量之间的相位差，检验“尺度”是否真的不是 1
    win = args.steady_win
    if win >= 5 and len(df) > win:
        print(f"\n[稳态窗拟合（窗长 {win} 帧={win * 0.01:.1f}s）]")
        rows = []
        for a in range(0, len(df) - win + 1, win):
            sl = np.arange(a, a + win)
            if not ok[sl].all():
                continue
            mw, s2, mm, pp = sum_w[sl], sum_t2[sl], mech[sl], p[sl]
            if mw.std() > args.steady_std_w or mm.std() > args.steady_std_mech:
                continue
            rows.append((mw.mean(), s2.mean(), mm.mean(), pp.mean()))
        if len(rows) >= 10:
            arr = np.array(rows)
            ols_fit(arr[:, 3], np.column_stack([arr[:, 0], arr[:, 1], arr[:, 2]]),
                    ["Σ|ω|", "Στ²", "Στω"], f"steady(n={len(rows)})")
            ols_fit(arr[:, 3] - 4.03, np.column_stack([arr[:, 0], arr[:, 1]]),
                    ["Σ|ω|", "Στ²"], "steady/2项")
        else:
            print(f"  稳态窗不足（{len(rows)} 个）")

    print("\n[候选参数在有效样本上的预测误差]")
    g_fw = args.mech_gain
    cand = [
        ("g=1     k1=0.001 k2=16.1 k3=4.03", 1.0, 0.001, 16.1, 4.03),
        ("g=1     k1=0.001 k2=14.3 k3=4.03", 1.0, 0.001, 14.3, 4.03),
        (f"g={g_fw:.3f} k1=0.001 k2=13.7 k3=4.03", g_fw, 0.001, 13.7, 4.03),
        (f"g={g_fw:.3f} k1=0.014 k2=13.7 k3=4.03", g_fw, 0.014, 13.7, 4.03),
    ]
    for name, g, a, b, c in cand:
        pred = g * mech[sel] + a * sum_w[sel] + b * sum_t2[sel] + c
        e = pred - p[sel]
        print(f"  {name:<38} mean={e.mean():+7.2f}  RMS={np.sqrt(np.mean(e**2)):6.2f}  "
              f"P95|e|={np.percentile(np.abs(e), 95):6.2f} W")

    if args.time_split > 0:
        print(f"\n[前 {args.time_split:.0f}% / 后 {100 - args.time_split:.0f}% 各拟合一次]")
        n = len(df)
        cuts = [(0, int(n * args.time_split / 100), "前段"),
                (int(n * args.time_split / 100), n, "后段")]
        for a, b, name in cuts:
            sl = ok.copy()
            sl[:a] = False
            sl[b:] = False
            if sl.sum() < 30:
                continue
            ols_fit(p[sl] - 4.03, np.column_stack([sum_w[sl], sum_t2[sl]]),
                    ["Σ|ω|", "Στ²"], name)


def abs_vs_sq(df, args):
    """|ω| 与 ω² 两种摩擦项、以及是否带机械增益 g 的对比（自由截距）。"""
    mask = df["gate"].round().astype(np.int64)
    wl, wr = df["w_l"].to_numpy(), df["w_r"].to_numpy()
    tl, tr = df["tau_l"].to_numpy(), df["tau_r"].to_numpy()
    p = df["p_meas"].to_numpy()

    sw = np.abs(wl) + np.abs(wr)
    s2 = tl * tl + tr * tr
    w2 = wl * wl + wr * wr
    mech = tl * wl + tr * wr

    ok = ((mask & GATE_RUN) != 0) & (mask & GATE_MOTION != 0)
    ok &= np.isfinite(p) & np.isfinite(sw) & np.isfinite(s2) & np.isfinite(w2) & np.isfinite(mech)
    ok &= np.abs(p) > args.min_abs_p

    # 稳态窗（与 --fit 相同判据）
    win = args.steady_win
    steady = np.zeros_like(ok)
    for a in range(0, len(df) - win + 1, win):
        sl = np.arange(a, a + win)
        if not ok[sl].all():
            continue
        if sw[sl].std() > args.steady_std_w or mech[sl].std() > args.steady_std_mech:
            continue
        steady[sl] = True

    # (显示名, 回归项, 机械项系数是否固定为 1)
    models = [
        ("τω(系数1) + |ω| + τ²", ["mech", "abs", "t2"], True),
        ("τω(系数1) + ω² + τ²", ["mech", "sq", "t2"], True),
        ("τω(系数1) + τ²（无摩擦项）", ["mech", "t2"], True),
        ("g + |ω| + τ²", ["mech", "abs", "t2"], False),
        ("g + ω² + τ²", ["mech", "sq", "t2"], False),
        ("g + τ²（无摩擦项）", ["mech", "t2"], False),
    ]
    cols = {"mech": mech, "abs": sw, "sq": w2, "t2": s2}
    names = {"mech": "Στω", "abs": "Σ|ω|", "sq": "Σω²", "t2": "Στ²"}

    for label, sub in (("全部有效样本", ok), ("稳态窗样本", steady)):
        if sub.sum() < 30:
            continue
        print(f"\n[abs vs sq — {label}，n={int(sub.sum())}]")
        for title, terms, fixed in models:
            if fixed:
                y = p[sub] - mech[sub]
                use = terms[1:]
            else:
                y = p[sub]
                use = terms
            X = np.column_stack([cols[t][sub] for t in use])
            ols_fit(y, X, [names[t] for t in use], title)


def _ols(y, X):
    """带截距的最小二乘，返回 (beta, se, rms, r2)。beta[0] 是截距。"""
    A = np.column_stack([np.ones_like(y), X])
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    e = y - A @ beta
    n, k = A.shape
    dof = max(n - k, 1)
    s2 = float(e @ e) / dof
    try:
        cov = s2 * np.linalg.inv(A.T @ A)
        se = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        se = np.full(k, np.nan)
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - float(e @ e) / ss_tot if ss_tot > 0 else np.nan
    return beta, se, float(np.sqrt(np.mean(e ** 2))), r2


def gain_scan(df, args):
    """检查机械项增益 g 是否随转速/力矩/方向变化。

    非恒定 g ⇒ 不是纯 Kt 刻度问题，而是模型缺项（铁损、齿槽、功率口径等）。
    做法：全局拟合 P = c0 + g·Στω + k2·Στ²，固定 k2 后在分箱内单独拟合 g。
    """
    mask = df["gate"].round().astype(np.int64)
    wl, wr = df["w_l"].to_numpy(), df["w_r"].to_numpy()
    tl, tr = df["tau_l"].to_numpy(), df["tau_r"].to_numpy()
    p = df["p_meas"].to_numpy()

    sw = np.abs(wl) + np.abs(wr)
    s2 = tl * tl + tr * tr
    mech = tl * wl + tr * wr

    ok = ((mask & GATE_RUN) != 0) & (mask & GATE_MOTION != 0)
    ok &= np.isfinite(p) & np.isfinite(sw) & np.isfinite(s2) & np.isfinite(mech)
    ok &= np.abs(p) > args.min_abs_p
    if ok.sum() < 40:
        print(f"  有效样本不足（{int(ok.sum())}），跳过")
        return

    beta, se, rms, r2 = _ols(p[ok], np.column_stack([mech[ok], s2[ok]]))
    g, k2 = beta[1], beta[2]
    print(f"\n[gain scan — 全局拟合] n={int(ok.sum())}  "
          f"k3={beta[0]:.3f}  g={g:.4f}±{se[1]:.4f}  k2={k2:.3f}±{se[2]:.3f}  RMS={rms:.2f} W  R²={r2:.4f}")

    # 分箱：固定全局 k2/k3，拟合残差 r = P − k3 − k2·Στ² 的斜率 g
    r = p - beta[0] - k2 * s2

    def bins(values, edges, label, unit):
        print(f"\n  按 {label} 分箱（固定 k3、k2，仅拟合 g）")
        print(f"    {'区间':<22}{'n':>6}{'g':>10}{'±se':>9}{'RMS(W)':>9}")
        idx = np.digitize(values, edges)
        for b in range(1, len(edges) + 1):
            sel = ok & (idx == b) & (np.abs(mech) > 1.0)
            if sel.sum() < 30:
                continue
            bb, ss, rr, _ = _ols(r[sel], mech[sel][:, None])
            lo = edges[b - 2] if b > 1 else -np.inf
            hi = edges[b - 1] if b <= len(edges) else np.inf
            print(f"    [{lo:7.1f},{hi:7.1f}) {unit:<5}{int(sel.sum()):>6}{bb[1]:>10.4f}{ss[1]:>9.4f}{rr:>9.2f}")

    bins(sw, np.percentile(sw[ok], [20, 40, 60, 80]), "Σ|ω|", "rad/s")
    bins(np.sqrt(s2), np.percentile(np.sqrt(s2[ok]), [20, 40, 60, 80]), "√Στ²", "N·m")
    # 电机发热会让 Kt 下降（约 −0.1%/°C），若 g 随时间下滑说明是热致 Kt 漂移
    t_s = (df["t_us"].to_numpy() - df["t_us"].iloc[0]) * 1e-6
    bins(t_s, np.percentile(t_s[ok], [25, 50, 75]), "时间", "s")

    print("\n  按工况拆分（固定 k3、k2，仅拟合 g）")
    print(f"    {'工况':<22}{'n':>6}{'g':>10}{'±se':>9}{'RMS(W)':>9}")
    splits = (
        ("驱动 Στω>0", ok & (mech > 1.0)),
        ("制动 Στω<0", ok & (mech < -1.0)),
        ("前进 ω>0", ok & ((wl + wr) > 5.0)),
        ("倒车 ω<0", ok & ((wl + wr) < -5.0)),
        ("原地自旋 |Σω|小", ok & (np.abs(wl + wr) < 5.0) & (sw > 5.0)),
    )
    for name, sel in splits:
        if sel.sum() < 30:
            continue
        bb, ss, rr, _ = _ols(r[sel], mech[sel][:, None])
        print(f"    {name:<22}{int(sel.sum()):>6}{bb[1]:>10.4f}{ss[1]:>9.4f}{rr:>9.2f}")

    # g 是否随转速漂移：P = c0 + (g0 + g1·Σ|ω|)·Στω + k2·Στ²
    X = np.column_stack([mech[ok], mech[ok] * sw[ok], s2[ok]])
    beta2, se2, rms2, r22 = _ols(p[ok], X)
    print("\n  交叉项检验：P = c0 + (g0 + g1·Σ|ω|)·Στω + k2·Στ²")
    print(f"    g0={beta2[1]:.4f}±{se2[1]:.4f}  g1={beta2[2]:+.5f}±{se2[2]:.5f} /(rad/s)  "
          f"RMS={rms2:.2f} W  (全局 RMS {rms:.2f} W)")
    print(f"    g1 的 |t|={abs(beta2[2] / se2[2]) if se2[2] > 0 else float('nan'):.2f}"
          "（>3 才说明 g 随转速显著变化）")


def check_reg(df, args, layout="rls"):
    """校验 RLS 调参布局的回归量通道（CH9~12、CH24~26）与重建值一致。"""
    if layout != "rls":
        print("\n[寄存器通道校验] 该 CSV 是底盘调试布局，CH9~14/CH24~26 不是回归量，跳过")
        return
    if np.isnan(df["reg_w_raw"].to_numpy()).all():
        print("  CSV 没有 RLS 调参通道（CH9~14 / CH24~26），跳过")
        return

    mask = df["gate"].round().astype(np.int64)
    wl, wr = df["w_l"].to_numpy(), df["w_r"].to_numpy()
    tl, tr = df["tau_l"].to_numpy(), df["tau_r"].to_numpy()
    g = df["mech_gain"].to_numpy()
    g = np.where(np.isfinite(g) & (g > 0), g, 1.0)

    sw, s2, w2 = np.abs(wl) + np.abs(wr), tl * tl + tr * tr, wl * wl + wr * wr
    mech = tl * wl + tr * wr

    print("\n[寄存器通道校验（raw）]")
    for name, val, unit in (
        ("reg_w_raw", sw, "rad/s"),
        ("reg_t2_raw", s2, "(N·m)²"),
        ("mech_raw", mech, "W"),
    ):
        e = df[name].to_numpy() - val
        sel = np.isfinite(e)
        print(f"  CH{OPT_CH[name]} {name:<11} 误差 RMS={np.sqrt(np.mean(e[sel] ** 2)):.4g}  "
              f"max|e|={np.max(np.abs(e[sel])):.4g} {unit}")

    print("\n[寄存器通道校验（低通，RC 扫描）]")
    dt = 0.01
    for rc in (0.03, 0.05, 0.07, 0.1):
        alpha = dt / (rc + dt)
        lw = l2 = lw2 = lm = 0.0
        seeded = was_run = False
        errs = {k: [] for k in ("reg_w_lpf", "reg_t2_lpf", "reg_w2_lpf", "mech_lpf")}
        for i in range(len(df)):
            run = bool(mask[i] & GATE_RUN)
            if run and not was_run:
                lw, l2, lw2 = sw[i], s2[i], w2[i]
                lm = mech[i] * g[i]
                seeded = True
            elif not run and was_run:
                seeded = False
            was_run = run
            if seeded and run:
                lw += (sw[i] - lw) * alpha
                l2 += (s2[i] - l2) * alpha
                lw2 += (w2[i] - lw2) * alpha
                lm += (mech[i] * g[i] - lm) * alpha
            for k, v in (("reg_w_lpf", lw), ("reg_t2_lpf", l2),
                         ("reg_w2_lpf", lw2), ("mech_lpf", lm)):
                errs[k].append(v)
        line = f"  RC={rc:4.2f}s "
        for k, v in errs.items():
            e = df[k].to_numpy() - np.array(v)
            sel = np.isfinite(e) & (mask & GATE_RUN != 0)
            line += f" {k.replace('reg_', '').replace('_lpf', '')}={np.sqrt(np.mean(e[sel] ** 2)):7.3f}"
        print(line + "  (RMS)")


def check_lpf(df, args):
    """用日志 k1/k2 反算 P_fb_lpf，检验我们对模块内部低通状态的重建。"""
    mask = df["gate"].round().astype(np.int64)
    wl, wr = df["w_l"].to_numpy(), df["w_r"].to_numpy()
    tl, tr = df["tau_l"].to_numpy(), df["tau_r"].to_numpy()
    k1 = df["k1"].to_numpy()
    k2 = df["k2"].to_numpy()
    logged = df["p_fb_lpf"].to_numpy()
    dt = 0.01

    for rc in (0.0, 0.05, 0.1, 0.2, 0.5):
        alpha = 1.0 if rc <= 0 else dt / (rc + dt)
        lw = ls2 = lm = 0.0
        seeded = False
        was_run = False
        err = []
        for i in range(len(df)):
            run = bool(mask[i] & GATE_RUN)
            if run and not was_run:
                lw = abs(wl[i]) + abs(wr[i]); ls2 = tl[i]**2 + tr[i]**2
                lm = tl[i]*wl[i] + tr[i]*wr[i]; seeded = True
            elif not run and was_run:
                seeded = False
            was_run = run
            if seeded and run:
                lw += (abs(wl[i]) + abs(wr[i]) - lw) * alpha
                ls2 += (tl[i]**2 + tr[i]**2 - ls2) * alpha
                lm += (tl[i]*wl[i] + tr[i]*wr[i] - lm) * alpha
            pred = lm + k1[i]*lw + k2[i]*ls2 + args.k3
            err.append(pred - logged[i])
        err = np.array(err)
        okk = np.isfinite(err) & (mask & GATE_RUN != 0)
        print(f"  RC={rc:4.2f}s  重建误差 RMS={np.sqrt(np.mean(err[okk]**2)):7.3f} W  "
              f"mean={err[okk].mean():+7.3f} W  P95|e|={np.percentile(np.abs(err[okk]), 95):7.3f} W")


def replay_mode(df, args):
    """按 app 模块的滤波/门控/RLS 逻辑离线复算，与日志里的 k1/k2 对比。

    日志只有 100 Hz，而固件 500 Hz；这里用 dt=10ms 近似，
    以“记录到 gate_mask==ALL”的帧作为一次更新（PERIOD 位已编码 20ms 节奏）。
    """
    mask = df["gate"].round().astype(np.int64)
    wl, wr = df["w_l"].to_numpy(), df["w_r"].to_numpy()
    tl, tr = df["tau_l"].to_numpy(), df["tau_r"].to_numpy()
    p = df["p_meas"].to_numpy()

    lam = args.lambda_
    k1i, k2i = args.k1_init, args.k2_init
    theta = np.array([k1i * 10.0, k2i * 0.1])
    P = np.eye(2) * args.p_init
    pmin, pmax = 1e-6, 100.0
    tmin, tmax = 0.01, 5.0
    dt = 0.01
    rc = args.rc
    alpha = dt / (rc + dt)

    lw = ls2 = lm = 0.0
    seeded = False
    acc = 0.0
    was_run = False
    log_th = []
    updates = 0

    for i in range(len(df)):
        run = bool(mask[i] & GATE_RUN)
        if run and not was_run:
            lw = abs(wl[i]) + abs(wr[i])
            ls2 = tl[i] ** 2 + tr[i] ** 2
            lm = (tl[i] * wl[i] + tr[i] * wr[i]) * args.mech_gain
            seeded = True
            acc = 0.0
        elif not run and was_run:
            seeded = False
            acc = 0.0
        was_run = run

        if not np.isfinite(p[i]) or not np.isfinite(wl[i]) or not np.isfinite(wr[i]):
            log_th.append(theta.copy())
            continue

        raw_w = abs(wl[i]) + abs(wr[i])
        raw_s2 = tl[i] ** 2 + tr[i] ** 2
        raw_m = (tl[i] * wl[i] + tr[i] * wr[i]) * args.mech_gain
        if seeded and run:
            lw += (raw_w - lw) * alpha
            ls2 += (raw_s2 - ls2) * alpha
            lm += (raw_m - lm) * alpha  # raw_m 已在上面乘 mech_gain
            acc = min(acc + dt, args.period)

        full = (mask[i] | (GATE_PERIOD if args.ignore_period else 0)) == GATE_ALL
        if run and full and acc >= (0.0 if args.ignore_period else args.period - 1e-9):
            phi = np.array([lw / 100.0, ls2 / 1.0])
            j = max(i - args.delay, 0)
            y = (p[j] - lm - args.k3) / 10.0
            innov = y - float(phi @ theta)
            innov = float(np.clip(innov, -args.innov_lim, args.innov_lim))
            Pphi = P @ phi
            K = Pphi / (lam + float(phi @ Pphi))
            theta = theta + K * innov
            theta = np.clip(theta, tmin, tmax)
            P = (P - np.outer(K, phi @ P)) / lam
            P = 0.5 * (P + P.T)
            P = np.clip(P, -pmax, pmax)  # 与 lib_rls 一致：非对角允许负值
            np.fill_diagonal(P, np.clip(np.diag(P), pmin, pmax))
            acc = 0.0
            updates += 1

        log_th.append(theta.copy())

    # 用日志里的 k1/k2 + 我们的 φ/y 反算新息，与 CH35 对比，检验 φ 重建是否可靠
    th_log = np.column_stack([df["k1"].to_numpy() / 0.1, df["k2"].to_numpy() / 10.0])
    y_all = (p - np.array(log_th)[:, 0] * 0.0 - args.k3) / 10.0  # 占位，下面逐帧重算
    phi_all = None
    print("\n[φ 重建校验] ")
    print("  （在 --fit 模式下另见残差；这里只在末端用日志 k1/k2 比对一次）")
    n_last = min(500, len(df))
    errs = []
    lw = ls2 = lm = 0.0
    seeded = False
    was_run = False
    for i in range(len(df)):
        run = bool(mask[i] & GATE_RUN)
        if run and not was_run:
            lw = abs(wl[i]) + abs(wr[i]); ls2 = tl[i] ** 2 + tr[i] ** 2
            lm = (tl[i] * wl[i] + tr[i] * wr[i]) * args.mech_gain; seeded = True
        elif not run and was_run:
            seeded = False
        was_run = run
        if seeded and run:
            lw += (abs(wl[i]) + abs(wr[i]) - lw) * alpha
            ls2 += (tl[i] ** 2 + tr[i] ** 2 - ls2) * alpha
            lm += ((tl[i] * wl[i] + tr[i] * wr[i]) * args.mech_gain - lm) * alpha
        if i >= len(df) - n_last:
            phi = np.array([lw / 100.0, ls2])
            yv = (p[i] - lm - args.k3) / 10.0
            innov_pred = yv - float(phi @ th_log[i])
            errs.append(innov_pred - df["innov"].iloc[i] / 10.0)
    errs = np.array(errs)
    print(f"  末 {n_last} 帧：innov 重建误差 RMS={np.sqrt(np.mean(errs**2)) * 10:.2f} W  "
          f"mean={errs.mean() * 10:+.2f} W")

    th = np.array(log_th)
    k1 = th[:, 0] * 0.1
    k2 = th[:, 1] * 10.0
    print("\n[离线复算 RLS]")
    print(f"  更新次数 {updates}（日志全门控帧 {int((mask == GATE_ALL).sum())}）")
    n = len(df)
    for frac in (0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99):
        i = min(int(n * frac), n - 1)
        print(
            f"  t={frac * 100:5.1f}%  复算 k1={k1[i]:8.5f} k2={k2[i]:8.3f}   "
            f"日志 k1={df['k1'].iloc[i]:8.5f} k2={df['k2'].iloc[i]:8.3f}"
        )
    print(f"  末值：复算 k1={k1[-1]:.5f} k2={k2[-1]:.3f} | 日志 k1={df['k1'].iloc[-1]:.5f} k2={df['k2'].iloc[-1]:.3f}")


def limit_check(df, args, layout):
    """校验功率限幅通道（CH1~4）与实际削力矩效果。"""
    print("\n[功率限幅校验（CH1~4，固件 CHASSIS_POWER_LIMIT_ENABLE）]")
    if layout != "rls":
        print("  当前为底盘调试布局（CH1~4 是舵角），跳过；请用 --layout rls 重看。")
        return

    lim = df["limit_eff"].to_numpy()
    p_before = df["p_before"].to_numpy()
    p_after = df["p_after"].to_numpy()
    scale = df["scale_min"].to_numpy()
    p_meas = df["p_meas"].to_numpy()
    p_cmd = df["p_cmd"].to_numpy()

    if not np.any(np.isfinite(lim)) or np.all(np.nan_to_num(lim) == 0.0):
        print("  CH1 全 0：日志不是带限幅的固件（或 CHASSIS_POWER_LIMIT_ENABLE=0），跳过。")
        return

    ok = np.isfinite(lim) & np.isfinite(p_before) & np.isfinite(p_after) & (lim > 0.0)
    n = int(ok.sum())
    print(f"  有效样本 {n}")
    if n == 0:
        return
    describe(lim[ok], "limit_eff", "W")
    describe(p_before[ok], "P_before", "W")
    describe(p_after[ok], "P_after", "W")
    describe(scale[ok], "scale_min")

    over = ok & (p_before > lim)
    n_over = int(over.sum())
    print(f"  超限样本：{n_over}（{100.0 * n_over / n:.2f}%）")
    if n_over:
        describe(scale[over], "限幅时 scale_min")
        describe(p_before[over] - lim[over], "超限幅度 P_before-limit", "W")
        exc = p_after[over] - lim[over]
        print(f"  限幅后仍超上限：{int(np.sum(exc > 1.0))} 帧（容差 1 W）")
        describe(exc, "P_after - limit_eff", "W")

        both = over & np.isfinite(p_cmd)
        if both.any():
            describe(p_after[both] - p_cmd[both], "P_after - P_cmd（限幅段）", "W")

        m = over & np.isfinite(p_meas) & (np.abs(p_meas) > args.min_abs_p)
        if m.any():
            d = p_meas[m] - p_after[m]
            print(f"\n  限幅期间 P_meas 对账（|P_meas|>{args.min_abs_p:g} W，n={int(m.sum())}）")
            describe(d, "P_meas - P_after", "W")
            print(f"    RMS = {np.sqrt(np.mean(d * d)):.2f} W，均值 = {np.mean(d):.2f} W")
            print(f"    P_meas 超限比例：{100.0 * np.mean(p_meas[m] > lim[m]):.1f}%")

    # 未限幅段：P_before 与 P_cmd 应基本一致（同模型、同指令）
    no = ok & ~over & np.isfinite(p_cmd)
    if no.any():
        describe(p_before[no] - p_cmd[no], "P_before - P_cmd（未限幅）", "W")


def main():
    ap = argparse.ArgumentParser(description="半舵底盘功率标定/分析")
    ap.add_argument("--csv", required=True, help="VOFA CSV 路径")
    ap.add_argument("--static", action="store_true", help="静态功耗标定（给 k3）")
    ap.add_argument("--residual", action="store_true", help="预测残差对账")
    ap.add_argument("--fit", action="store_true", help="离线回归拟合（诊断 k1/k2 可辨识性）")
    ap.add_argument("--time-split", type=float, default=50.0, help="--fit 前后段拆分百分比")
    ap.add_argument("--check-lpf", action="store_true", help="检验内部低通状态重建（对比 CH30）")
    ap.add_argument("--replay", action="store_true", help="按模块逻辑离线复算 RLS，与日志对比")
    ap.add_argument("--check-reg", action="store_true", help="校验 RLS 回归量通道（CH9~12 / CH24~26）")
    ap.add_argument("--abs-vs-sq", action="store_true", help="对比 |ω| 与 ω² 两种摩擦项")
    ap.add_argument("--gain-scan", action="store_true", help="检查机械项增益 g 是否随转速/力矩/工况恒定")
    ap.add_argument("--limit-check", action="store_true", help="校验功率限幅（CH1~4）与削力矩效果")
    ap.add_argument("--layout", choices=["auto", "rls", "debug"], default="auto",
                    help="CH9~14/CH24~26 的布局（旧日志自动判为 debug）")
    ap.add_argument("--lambda", dest="lambda_", type=float, default=0.999)
    ap.add_argument("--rc", type=float, default=0.05, help="复算用低通 RC（s）")
    ap.add_argument("--period", type=float, default=0.02, help="复算用更新周期（s）")
    ap.add_argument("--k3", type=float, default=3.0, help="复算用 k3（日志对应版本的取值）")
    ap.add_argument("--mech-gain", type=float, default=0.723,
                    help="复算用机械项增益 g（默认对齐固件 CHASSIS_POWER_MECH_GAIN）")
    ap.add_argument("--innov-lim", type=float, default=2.0, help="归一化新息限幅")
    ap.add_argument("--p-init", type=float, default=1.0)
    ap.add_argument("--k1-init", type=float, default=0.2)
    ap.add_argument("--k2-init", type=float, default=1.2)
    ap.add_argument("--ignore-period", action="store_true", help="复算时忽略 PERIOD 位（模拟更高更新率）")
    ap.add_argument("--delay", type=int, default=0, help="复算时 y 相对 φ 的滞后帧数")
    ap.add_argument("--steady-win", type=int, default=50, help="稳态窗长（帧，100Hz）")
    ap.add_argument("--steady-std-w", type=float, default=0.5, help="稳态窗 Σ|ω| 标准差上限")
    ap.add_argument("--steady-std-mech", type=float, default=3.0, help="稳态窗 Στω 标准差上限 (W)")
    ap.add_argument(
        "--require-fresh",
        action="store_true",
        default=True,
        help="静止样本额外要求轮反馈新鲜 + 超电正常（默认开）",
    )
    ap.add_argument("--no-require-fresh", dest="require_fresh", action="store_false")
    ap.add_argument("--only-full-gate", action="store_true", help="只取全门控通过的样本")
    ap.add_argument("--min-abs-p", type=float, default=10.0, help="对账时 |P_meas| 下限（W）")
    args = ap.parse_args()

    df = load(args.csv)
    layout = detect_layout(df) if args.layout == "auto" else args.layout
    print(f"帧数 {len(df)}，时长 {(df['t_us'].iloc[-1] - df['t_us'].iloc[0]) * 1e-6:.2f} s")
    print(f"VOFA 布局：{layout}" + ("（CH9~14/CH24~26 为 RLS 回归量）" if layout == "rls" else "（CH9~14/CH24~26 为底盘调试量）"))
    gate_report(df)

    if args.static:
        static_k3(df, args)
    if args.residual:
        residual(df, args)
    if args.fit:
        fit_mode(df, args)
    if args.abs_vs_sq:
        abs_vs_sq(df, args)
    if args.gain_scan:
        gain_scan(df, args)
    if args.check_reg:
        check_reg(df, args, layout)
    if args.check_lpf:
        print("\n[内部 P_fb_lpf 重建检验（CH30）]")
        check_lpf(df, args)
    if args.replay:
        replay_mode(df, args)
    if args.limit_check:
        limit_check(df, args, layout)
    if not (args.static or args.residual or args.fit or args.replay or args.limit_check):
        print("\n（未指定分析模式，只显示门控统计）")


if __name__ == "__main__":
    main()
