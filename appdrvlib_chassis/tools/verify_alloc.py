#!/usr/bin/env python3
"""离线验证 appdrvlib_chassis 分层分配（C 实现的 Python 镜像）。

用途：不烧板子先检查符号/几何是否正确。对若干稳态工况仿真足够长时间，
比较"分配器输出经实际舵角/力矩投影后的合力"与目标 wrench。

用法：
    python3 verify_alloc.py
    python3 verify_alloc.py --a 0.20225 --radius 0.05025 --settle 2.0 --rate 200
"""

import argparse
import math

# 与 appdrvlib_chassis.h 的默认参数保持一致
FORCE_MIN = 1.0
MZ_MIN = 0.05
TRIM_LPF_RC = 0.20
TRIM_MAX = math.radians(30.0)
TRIM_SLEW = math.radians(60.0)
TRIM_SIN_MIN = 0.30
FAST_COS_MIN = 0.30
FORCE_MAX = 100.0
BRANCH_MARGIN = math.radians(10.0)
PI = math.pi


def clamp(x, lo, hi):
    return min(max(x, lo), hi)


def wrap_pi(x):
    """(-pi, pi]"""
    x = math.fmod(x, 2.0 * PI)
    if x >= PI:
        x -= 2.0 * PI
    elif x < -PI:
        x += 2.0 * PI
    return x


class State:
    def __init__(self):
        self.mz_slow = 0.0
        self.trim = 0.0
        self.branch = 0
        self.valid = False


def allocate(st, cfg, alpha, fx, fy, mz, dt):
    """C 函数 AppDrvlibChassisAllocate 的镜像。alpha 为实际连续舵角。"""
    if not st.valid:
        st.mz_slow = 0.0
        st.trim = 0.0
        st.branch = 0
        st.valid = True

    F = math.hypot(fx, fy)
    yaw_only = F < FORCE_MIN and abs(mz) > MZ_MIN
    idle = F < FORCE_MIN and abs(mz) <= MZ_MIN

    cos_psi, sin_psi = 1.0, 0.0
    if F > 1e-6:
        cos_psi, sin_psi = fx / F, fy / F

    fast_den = 1.0 - FAST_COS_MIN
    if yaw_only or idle:
        w_fast = 1.0
    else:
        w_fast = clamp((abs(cos_psi) - FAST_COS_MIN) / fast_den, 0.0, 1.0)

    if yaw_only or idle:
        mz_slow_target = 0.0
    else:
        mz_slow_target = (1.0 - w_fast) * mz
    if dt > 0.0:
        k = dt / (TRIM_LPF_RC + dt)
        st.mz_slow += k * (mz_slow_target - st.mz_slow)

    out = dict(valid=True, reverse=st.branch, mz_slow=st.mz_slow,
               mz_fast=0.0, trim=st.trim)

    if idle:
        out["target"] = [alpha[0], alpha[1]]
        out["torque"] = [0.0, 0.0]
        out["align"] = [1.0, 1.0]
        step = TRIM_SLEW * max(dt, 0.0)
        if abs(st.trim) <= step:
            st.trim = 0.0
        else:
            st.trim -= math.copysign(step, st.trim)
        out["trim"] = st.trim
        return out

    base = 0.0 if yaw_only else math.atan2(fy, fx)

    mid = 0.5 * (alpha[0] + alpha[1])
    d = wrap_pi(base - mid)
    if st.branch == 0 and abs(d) > PI / 2 + BRANCH_MARGIN:
        st.branch = 1
    elif st.branch == 1 and abs(d) < PI / 2 - BRANCH_MARGIN:
        st.branch = 0
    branch = st.branch
    branch_angle = PI if branch else 0.0
    force_sign = -1.0 if branch else 1.0

    trim_target = 0.0
    if not yaw_only and abs(sin_psi) > TRIM_SIN_MIN:
        denom = cfg["a"] * F * sin_psi
        if abs(denom) > 1e-4:
            trim_target = math.atan(st.mz_slow / denom)
    trim_target = clamp(trim_target, -TRIM_MAX, TRIM_MAX)
    trim_step = TRIM_SLEW * max(dt, 0.0)
    dt_trim = clamp(trim_target - st.trim, -trim_step, trim_step)
    st.trim += dt_trim

    f_l = f_r = 0.0
    if yaw_only:
        df = mz / (2.0 * cfg["a"])
        f_l, f_r = -df, df
        out["mz_fast"] = mz
    else:
        mz_fast = mz - st.mz_slow
        cos_eff = cos_psi * math.cos(st.trim)
        df = 0.0
        if w_fast > 0.0 and abs(cos_eff) > 1e-3:
            df = mz_fast / (2.0 * cfg["a"] * cos_eff)
        f_common = F / (2.0 * math.cos(st.trim))
        f_l, f_r = f_common - df, f_common + df
        out["mz_fast"] = mz_fast

    f_l = clamp(f_l, -FORCE_MAX, FORCE_MAX)
    f_r = clamp(f_r, -FORCE_MAX, FORCE_MAX)

    tgt = [base + branch_angle + st.trim, base + branch_angle - st.trim]
    out["target"] = tgt
    out["torque"] = [force_sign * f_l * cfg["radius"], force_sign * f_r * cfg["radius"]]
    out["align"] = [math.cos(wrap_pi(tgt[i] - alpha[i])) for i in range(2)]
    out["reverse"] = branch
    out["trim"] = st.trim
    return out


def wrench_from_out(out, alpha, cfg):
    """分配器输出 → 实际作用在车体上的 (fx, fy, mz)。

    电机只能沿**当前**舵向出力：torque_cmd = torque * align。
    """
    a = cfg["a"]
    ws = [out["torque"][i] * out["align"][i] / cfg["radius"] for i in range(2)]
    fx = fy = mz = 0.0
    for i, y in ((0, +a), (1, -a)):
        fx += ws[i] * math.cos(alpha[i])
        fy += ws[i] * math.sin(alpha[i])
        mz += -y * ws[i] * math.cos(alpha[i])  # x = 0
    return fx, fy, mz


def simulate(cfg, alpha0, fx, fy, mz, settle, rate_dps, dt=0.002):
    st = State()
    alpha = list(alpha0)
    out = None
    steps = int(settle / dt)
    tail = []
    for k in range(steps):
        out = allocate(st, cfg, alpha, fx, fy, mz, dt)
        # 理想舵向：以 rate_dps 跟随目标
        step_max = math.radians(rate_dps) * dt
        for i in range(2):
            d = wrap_pi(out["target"][i] - alpha[i])
            alpha[i] += clamp(d, -step_max, step_max)
        if k >= steps // 2:
            tail.append(wrench_from_out(out, alpha, cfg))
    avg = tuple(sum(v[i] for v in tail) / len(tail) for i in range(3))
    return out, alpha, avg


def main():
    ap = argparse.ArgumentParser(description="分层分配离线验证")
    ap.add_argument("--a", type=float, default=0.20225, help="舵轮回转中心 |y| (m)")
    ap.add_argument("--radius", type=float, default=0.05025, help="驱动轮半径 (m)")
    ap.add_argument("--settle", type=float, default=2.0, help="每工况仿真时长 (s)")
    ap.add_argument("--rate", type=float, default=200.0, help="舵向跟随速率 (deg/s)")
    args = ap.parse_args()
    cfg = {"a": args.a, "radius": args.radius}

    # (名称, 初始舵角 L/R, fx, fy, mz)
    cases = [
        ("纯横移 +y", [math.pi / 2, math.pi / 2], 0.0, 20.0, 0.0),
        ("纯横移 -y", [math.pi / 2, math.pi / 2], 0.0, -20.0, 0.0),
        ("纯前进 +x", [0.0, 0.0], 20.0, 0.0, 0.0),
        ("前进 + 正 yaw", [0.0, 0.0], 20.0, 0.0, 2.0),
        ("横移 + 正 yaw", [math.pi / 2, math.pi / 2], 0.0, 20.0, 1.0),
        ("斜向 + yaw", [math.pi / 4, math.pi / 4], 14.142, 14.142, 1.0),
        ("纯自旋 +yaw", [math.pi / 2, math.pi / 2], 0.0, 0.0, 2.0),
    ]

    print(f"几何: a={cfg['a']} m, R={cfg['radius']} m, settle={args.settle}s, "
          f"舵向跟随={args.rate}°/s")
    print(f"{'工况':14s} {'目标 (fx,fy,mz)':>24s} {'实际 (fx,fy,mz)':>24s} "
          f"{'误差':>18s} {'分支':>4s} {'trim°':>7s}")
    ok = True
    for name, a0, fx, fy, mz in cases:
        out, alpha, avg = simulate(cfg, a0, fx, fy, mz, args.settle, args.rate)
        ex, ey, ez = avg[0] - fx, avg[1] - fy, avg[2] - mz
        good = abs(ex) < 1.0 and abs(ey) < 1.0 and abs(ez) < 0.20
        ok &= good
        print(f"{name:14s} ({fx:7.2f},{fy:7.2f},{mz:6.2f}) "
              f"({avg[0]:7.2f},{avg[1]:7.2f},{avg[2]:6.2f}) "
              f"({ex:6.2f},{ey:6.2f},{ez:6.2f}) {'OK' if good else 'FAIL':>4s} "
              f"{out['reverse']:4d} {math.degrees(out['trim']):7.2f}")

    # 换向测试：+y 收敛后切 -y，检查舵角是否原地反转而不是转 180°
    out, alpha, _ = simulate(cfg, [math.pi / 2, math.pi / 2], 0.0, 20.0, 0.0, 1.0, args.rate)
    alpha_rev = list(alpha)
    out2, alpha2, avg2 = simulate(cfg, alpha_rev, 0.0, -20.0, 0.0, 1.0, args.rate)
    moved = max(abs(wrap_pi(alpha2[i] - alpha_rev[i])) for i in range(2))
    print(f"\n换向 +y→-y: 舵角最大移动 {math.degrees(moved):7.2f}°, 分支={out2['reverse']}, "
          f"平均力=({avg2[0]:6.2f},{avg2[1]:6.2f},{avg2[2]:6.2f})")
    print("结论:", "全部通过" if ok else "存在失败工况")


if __name__ == "__main__":
    main()
