# -*- coding: utf-8 -*-
"""
问题 1 的公式与代码自检（三套互相独立的方法交叉验证 + 计算器）

为什么需要：同一个"定位区域直径"会因为【口径】不同而给出完全不同的数：
    · 是否把"有效接收半径圆"和"目标圆域"也作为约束？
    · 误差是 ±1°（半宽）还是 2°（全宽）？
    · 直径取的是区域内部任意两点最大距离，还是别的量（如横向宽度、对角线）？

三套独立方法：
   方法 A  半平面裁剪（Sutherland–Hodgman）—— problem1_clip.py 的做法
   方法 B  顶点枚举：把 4 条边界线两两求交，保留落在所有楔形内的点，再取最大点距
            （完全不依赖裁剪、不依赖起始多边形）
   方法 C  解析近似：两条窄带交成平行四边形，d ≈ (2ε/sinα)·√(r₁²+r₂²+2r₁r₂cosα)

用法：
    python problem1_selftest.py                       # 跑自检
    python problem1_selftest.py --case "0,0;600,0" --bearings "55,125" --eps 1
"""

import argparse
import math
import random

from problem1_clip import (localization_region, polygon_diameter, min_enclosing_circle,
                          clip_polygon, halfplane, TARGET_R, RECV_R)

BOX = 1.0e5


# ----------------------------------------------------------------------
# 方法 B：顶点枚举（独立于裁剪）
# ----------------------------------------------------------------------
def _ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def in_wedge(P, S, th, eps, tol=1e-9):
    ang = math.degrees(math.atan2(P[1] - S[1], P[0] - S[0]))
    return abs(_ang_diff(ang, th)) <= eps + tol


def vertices_by_enumeration(points, bearings, eps):
    """把每个楔形的两条边界线汇总，两两求交，保留在所有楔形内的交点。"""
    lines = []
    for S, th in zip(points, bearings):
        for a in (th - eps, th + eps):
            r = math.radians(a)
            lines.append((S, (math.cos(r), math.sin(r))))
    cand = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            (p, d1), (q, d2) = lines[i], lines[j]
            den = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(den) < 1e-14:
                continue
            t = ((q[0] - p[0]) * d2[1] - (q[1] - p[1]) * d2[0]) / den
            cand.append((p[0] + t * d1[0], p[1] + t * d1[1]))
    keep = [P for P in cand
            if all(in_wedge(P, S, th, eps) for S, th in zip(points, bearings))]
    # 去重
    uniq = []
    for P in keep:
        if not any(math.dist(P, Q) < 1e-7 for Q in uniq):
            uniq.append(P)
    return uniq


def clip_no_disk(points, bearings, eps):
    """方法 A 的"纯楔形交会"版本：不裁剪圆盘，从大盒子出发；无界则返回 None。"""
    poly = [(-BOX, -BOX), (BOX, -BOX), (BOX, BOX), (-BOX, BOX)]
    for S, th in zip(points, bearings):
        for plane in (halfplane(S, th - eps, True), halfplane(S, th + eps, False)):
            poly = clip_polygon(poly, plane)
            if len(poly) < 3:
                return None
    for p in poly:
        if abs(p[0]) > BOX * 0.99 or abs(p[1]) > BOX * 0.99:
            return None
    return poly


def diam_of_points(pts):
    best = (0.0, None, None)
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            d = math.dist(pts[i], pts[j])
            if d > best[0]:
                best = (d, pts[i], pts[j])
    return best


# ----------------------------------------------------------------------
# 方法 C：解析近似
# ----------------------------------------------------------------------
def analytic_diameter(S1, S2, th1, th2, eps_deg, G=None):
    """
    解析近似：两条窄带（宽 wᵢ = 2ε·rᵢ）交成平行四边形，
        d ≈ (2ε/sinα)·√(r₁² + r₂² + 2 r₁ r₂ |cosα|)

    ⚠ 两个易错点（都踩过）：
      ① 必须用 |cosα|：平行四边形两条对角线是 √(a²+b²±2ab cosα)，
         锐角时长的取 "+"，**钝角时 cosα<0，长的反而是 "−"**，合起来就是 |cosα|。
         写成带符号的 cosα 会在钝角交会时严重低估（实测偏小一半以上）。
      ② 必须用【区域内的代表点】估计 r₁、r₂、α：
         若用两条中心线的交点（近乎平行时交点会跑到很远甚至反向），公式会失真。
    """
    if G is None:
        G = _estimate_source(S1, S2, th1, th2)
    if G is None:
        return None
    r1 = math.dist(S1, G)
    r2 = math.dist(S2, G)
    a1 = math.degrees(math.atan2(S1[1] - G[1], S1[0] - G[0]))
    a2 = math.degrees(math.atan2(S2[1] - G[1], S2[0] - G[0]))
    alpha = abs(_ang_diff(a1, a2))
    sa = math.sin(math.radians(alpha))
    if sa < 1e-6:
        return None
    ca = abs(math.cos(math.radians(alpha)))     # ← 关键：取绝对值
    e = math.radians(eps_deg)
    return (2 * e / sa) * math.sqrt(r1 * r1 + r2 * r2 + 2 * r1 * r2 * ca)


def _estimate_source(S1, S2, th1, th2):
    """两站示向度中心线求交 —— 只用于解析近似里的 r₁、r₂、α。"""
    d1 = (math.cos(math.radians(th1)), math.sin(math.radians(th1)))
    d2 = (math.cos(math.radians(th2)), math.sin(math.radians(th2)))
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-12:
        return None
    t = ((S2[0] - S1[0]) * d2[1] - (S2[1] - S1[1]) * d2[0]) / den
    if t <= 0:
        return None
    return (S1[0] + t * d1[0], S1[1] + t * d1[1])


# ----------------------------------------------------------------------
# 自检
# ----------------------------------------------------------------------
def run_selftest(n_rand=800):
    print("=" * 76)
    print("自检 1：方法 A（裁剪） vs 方法 B（顶点枚举） —— 纯楔形交会，无圆盘裁剪")
    print("=" * 76)
    rng = random.Random(20260911)
    worst = 0.0
    tested = 0
    bad = 0
    for _ in range(n_rand):
        S1 = (0.0, 0.0)
        psi = rng.uniform(0, 2 * math.pi)
        L = rng.uniform(300, 2500)
        S2 = (L * math.cos(psi), L * math.sin(psi))
        th1 = rng.uniform(0, 360)
        th2 = rng.uniform(0, 360)
        if abs(_ang_diff(th1, th2)) < 3:            # 近乎平行 → 无界，跳过
            continue
        pa = clip_no_disk([S1, S2], [th1, th2], 1.0)
        if pa is None:
            continue
        pb = vertices_by_enumeration([S1, S2], [th1, th2], 1.0)
        if len(pb) < 3:
            continue
        da = polygon_diameter(pa)[0]
        db = diam_of_points(pb)[0]
        tested += 1
        rel = abs(da - db) / max(da, 1e-9)
        worst = max(worst, rel)
        if rel > 1e-6:
            bad += 1
    print("   有效算例 %d 组；两种方法的最大相对偏差 = %.3e；超过 1e-6 的 = %d 组"
          % (tested, worst, bad))
    print("   结论：%s" % ("一致 ✔" if worst < 1e-6 else "不一致 ✘"))

    print()
    print("=" * 76)
    print("自检 2：方法 A vs 方法 C（解析近似） —— 检验 4ε²r₁r₂/sinα 这套公式")
    print("=" * 76)
    worst = 0.0
    tested = 0
    worst_ok = 0.0
    tested_ok = 0
    worst_case = None
    for _ in range(n_rand):
        # 物理上有意义的采样：先取真实源 G，两站都在 G 的接收半径内，再叠加 ±1° 误差
        ang = rng.uniform(0, 2 * math.pi)
        rg = rng.uniform(50.0, 1700.0)
        G = (rg * math.cos(ang), rg * math.sin(ang))
        S1 = (0.0, 0.0)
        a2 = rng.uniform(0, 2 * math.pi)
        r2 = rng.uniform(200.0, 1500.0)
        S2 = (G[0] + r2 * math.cos(a2), G[1] + r2 * math.sin(a2))
        if math.dist(S1, G) > 1500.0 or math.hypot(*S2) > 2600.0:
            continue
        th1 = math.degrees(math.atan2(G[1] - S1[1], G[0] - S1[0])) + rng.uniform(-1, 1)
        th2 = math.degrees(math.atan2(G[1] - S2[1], G[0] - S2[0])) + rng.uniform(-1, 1)
        if abs(_ang_diff(th1, th2)) < 3:
            continue
        pa = clip_no_disk([S1, S2], [th1, th2], 1.0)
        if pa is None:
            continue
        Gc = (sum(p[0] for p in pa) / len(pa), sum(p[1] for p in pa) / len(pa))
        r1, r2 = math.dist(S1, Gc), math.dist(S2, Gc)
        a1 = math.degrees(math.atan2(S1[1] - Gc[1], S1[0] - Gc[0]))
        a2b = math.degrees(math.atan2(S2[1] - Gc[1], S2[0] - Gc[0]))
        alpha = abs(_ang_diff(a1, a2b))
        da = polygon_diameter(pa)[0]
        dc = analytic_diameter(S1, S2, th1, th2, 1.0, G=Gc)
        if dc is None or dc < 1.0:
            continue
        tested += 1
        rel = abs(da - dc) / da
        if rel > worst:
            worst = rel
            worst_case = (math.dist(S1, S2), alpha, da, dc)
        # 近似公式的适用域：交会角既不接近 0° 也不接近 180°（两者都会让 1/sinα 爆炸），
        # 且区域尺寸远小于两站到源的距离（否则"窄带"近似失效）
        if 20.0 <= alpha <= 160.0 and da <= 0.25 * min(r1, r2):
            tested_ok += 1
            worst_ok = max(worst_ok, rel)
    print("   全部有效算例 %d 组；最大相对偏差 = %.1f%%" % (tested, worst * 100))
    if worst_case:
        print("      最差算例：站间距 %.0f m、交会角 %.1f°、精确 d=%.1f m、近似 %.1f m"
              % (worst_case[0], worst_case[1], worst_case[2], worst_case[3]))
    print("   限定在适用域内（20° ≤ 交会角 ≤ 160° 且 d ≤ 0.25·min(r₁,r₂)）—— %d 组："
          % tested_ok)
    print("      最大相对偏差 = %.2f%%" % (worst_ok * 100))
    print("   结论：适用域内近似公式很准；交会角趋近 0° 或 180° 时 1/sinα 会爆炸，此时")
    print("         必须用精确裁剪（方法 A/B），不能用近似公式。")

    print()
    print("=" * 76)
    print("自检 3：与手工解析式对齐（两站关于 y 轴对称的对称算例）")
    print("=" * 76)
    print("   %-34s %10s %10s %10s" % ("算例", "方法A", "方法B", "方法C"))
    for r, alpha_deg in ((1000.0, 90.0), (1000.0, 60.0), (600.0, 90.0), (500.0, 90.0)):
        G = (0.0, 0.0)
        hal = alpha_deg / 2.0
        S1 = (r * math.cos(math.radians(180 - hal)), r * math.sin(math.radians(180 - hal)))
        S2 = (r * math.cos(math.radians(180 + hal)), r * math.sin(math.radians(180 + hal)))
        th1 = math.degrees(math.atan2(G[1] - S1[1], G[0] - S1[0]))
        th2 = math.degrees(math.atan2(G[1] - S2[1], G[0] - S2[0]))
        pa = clip_no_disk([S1, S2], [th1, th2], 1.0)
        da = polygon_diameter(pa)[0] if pa else float("nan")
        pb = vertices_by_enumeration([S1, S2], [th1, th2], 1.0)
        db = diam_of_points(pb)[0] if len(pb) >= 3 else float("nan")
        dc = analytic_diameter(S1, S2, th1, th2, 1.0) or float("nan")
        print("   r=%.0fm, 交会角=%.0f°%-16s %10.3f %10.3f %10.3f"
              % (r, alpha_deg, "", da, db, dc))


def run_case(case, bearings, eps):
    points = []
    for seg in case.split(";"):
        x, y = seg.split(",")
        points.append((float(x), float(y)))
    bs = [float(x) for x in bearings.split(",")]
    print("=" * 76)
    print("计算器：检测点 %s  示向度 %s  误差半宽 ±%.3f°" % (points, bs, eps))
    print("=" * 76)

    pb = vertices_by_enumeration(points, bs, eps)
    db = diam_of_points(pb)[0] if len(pb) >= 3 else float("nan")
    print("【方法 B 顶点枚举】区域顶点 %d 个：" % len(pb))
    for p in pb:
        print("     (%10.3f, %10.3f)" % p)
    print("   直径 = %.3f m" % db)

    pa = clip_no_disk(points, bs, eps)
    if pa:
        da, PA, PB = polygon_diameter(pa)
        print("【方法 A 纯楔形裁剪】顶点 %d 个，直径 = %.3f m" % (len(pa), da))
    else:
        print("【方法 A 纯楔形裁剪】区域无界（两站示向度近乎平行），必须再加圆盘约束")

    full = localization_region(points, bs, eps_deg=eps, verbose=False)
    if full:
        df, PA, PB = polygon_diameter(full)
        r_star, _c = min_enclosing_circle(full)
        print("【方法 A + 目标圆域/接收半径圆裁剪（本程序默认口径）】顶点 %d 个" % len(full))
        print("   直径 d = %.3f m ；d/2 = %.3f m ；最小覆盖圆 r* = %.3f m ；2r*/d = %.6f"
              % (df, df / 2, r_star, 2 * r_star / df))
        print("   以 d 为直径的圆能否覆盖区域：%s" % ("能" if 2 * r_star <= df + 1e-6 else "不能"))
    if len(points) == 2:
        Gc = None
        if pa and len(pa) >= 3:
            Gc = (sum(p[0] for p in pa) / len(pa), sum(p[1] for p in pa) / len(pa))
        dc = analytic_diameter(points[0], points[1], bs[0], bs[1], eps, G=Gc)
        print("【方法 C 解析近似】d ≈ %.3f m" % (dc if dc else float("nan")))
    print()
    print("提示：把 GPT 用的检测点坐标与示向度填进来，对比上面三个数。")
    print("      若三个数一致而仍与 23 不同，差异就出在【输入或口径】上（见文末排查清单）。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default=None, help='检测点，如 "0,0;600,0"')
    ap.add_argument("--bearings", default=None, help='示向度，如 "55,125"')
    ap.add_argument("--eps", type=float, default=1.0, help="误差半宽（度），默认 1")
    a = ap.parse_args()
    if a.case and a.bearings:
        run_case(a.case, a.bearings, a.eps)
    else:
        run_selftest()


if __name__ == "__main__":
    main()
