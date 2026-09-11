# -*- coding: utf-8 -*-
"""
2026 国赛 B 题 · 问题 1
交会定位区域：半平面裁剪（Sutherland–Hodgman）+ 区域直径 + 覆盖圆判定

核心思想
--------
在检测点 S 测得示向度 θ 后，真实方位角 β 满足 |β − θ| ≤ 1°，
于是干扰源必落在【以 S 为顶点、方向 θ、半角 ε = 1° 的角扇形（楔形）】内。

每个角扇形 = 两个半平面的交集：
    W = { P : (P − S) · n(θ−ε) ≥ 0 }  ∩  { P : (P − S) · n(θ+ε) ≤ 0 }
其中 n(α) = (−sin α, cos α) 是方向 α 的"左法向"。

多个测点 → 多个扇形求交 → 交集仍是凸多边形（凸集的交仍是凸集）。
求交用【逐条半平面裁剪多边形】，即 Sutherland–Hodgman 算法。

作者备注：本文件可直接 import 使用，也可 python 运行看演示。
"""

import math

EPS_DEG = 1.0            # 示向度误差半宽（度）
TARGET_R = 1800.0        # 目标圆域半径（米）
RECV_R = 1500.0          # 有效接收半径上界（米），用于额外收紧区域
N_CIRCLE = 720           # 用正 N 边形近似圆的边数


# ----------------------------------------------------------------------
# 1. 基础几何
# ----------------------------------------------------------------------
def normal_left(angle_deg):
    """方向 angle_deg 的"左法向"单位向量 n = (−sin α, cos α)。"""
    a = math.radians(angle_deg)
    return (-math.sin(a), math.cos(a))


def halfplane(apex, angle_deg, keep_left=True):
    """
    由【顶点 apex + 方向 angle_deg】定义的射线，生成一个半平面 (a, b, c)。

    半平面约定：保留满足  f(x, y) = a*x + b*y + c >= 0  的部分。

    keep_left=True  → 保留射线的左侧（逆时针一侧）
    keep_left=False → 保留射线的右侧

    说明： (P − apex)·n >= 0 即保留左侧，其中 n = normal_left(angle_deg)。
    """
    nx, ny = normal_left(angle_deg)
    c = -(nx * apex[0] + ny * apex[1])
    if keep_left:
        return (nx, ny, c)
    return (-nx, -ny, -c)


def clip_polygon(poly, plane, tol=1e-9):
    """
    Sutherland–Hodgman：用一条直线把凸多边形切成"保留侧"。

    plane = (a, b, c)，保留 f = a*x + b*y + c >= 0。

    逐边处理，四种情况（就是演示图里那四种）：
        f(p)>=0, f(q)>=0 → 输出 q
        f(p)>=0, f(q)< 0 → 输出交点（边穿出）
        f(p)< 0, f(q)>=0 → 输出交点、再输出 q（边穿入）
        f(p)< 0, f(q)< 0 → 什么都不输出
    """
    if not poly:
        return []
    a, b, c = plane
    out = []
    n = len(poly)
    for i in range(n):
        p = poly[i]
        q = poly[(i + 1) % n]
        fp = a * p[0] + b * p[1] + c
        fq = a * q[0] + b * q[1] + c
        if fp >= -tol:
            out.append(p)                       # 端点 p 在内 → 输出 p
        # 两端分居直线两侧 → 求交点并输出
        if (fp > tol and fq < -tol) or (fp < -tol and fq > tol):
            t = fp / (fp - fq)                  # p + t*(q-p) 落在直线上
            out.append((p[0] + t * (q[0] - p[0]),
                        p[1] + t * (q[1] - p[1])))
    return out


def polygon_edges_halfplanes(center, radius, n=N_CIRCLE):
    """
    把圆（近似为正 n 边形）写成 n 条边的【内侧半平面】列表。
    用途：把定位区域再与"有效接收半径圆"求交。
    注意：正 n 边形是圆的内接多边形，结果略偏小（乐观）；
          n=720 时误差约 1e-5 量级，可忽略。
    """
    planes = []
    for k in range(n):
        a1 = 360.0 * k / n
        a2 = 360.0 * (k + 1) / n
        p1 = (center[0] + radius * math.cos(math.radians(a1)),
              center[1] + radius * math.sin(math.radians(a1)))
        p2 = (center[0] + radius * math.cos(math.radians(a2)),
              center[1] + radius * math.sin(math.radians(a2)))
        # 边 p1->p2 的左法向指向圆内，故保留左侧
        ex, ey = p2[0] - p1[0], p2[1] - p1[1]
        nx, ny = -ey, ex                        # 左法向
        norm = math.hypot(nx, ny)
        nx, ny = nx / norm, ny / norm
        planes.append((nx, ny, -(nx * p1[0] + ny * p1[1])))
    return planes


def wedge_planes(apex, theta_deg, eps_deg=EPS_DEG):
    """角扇形（楔形）对应的两条半平面：下边界保留左侧，上边界保留右侧。"""
    return [halfplane(apex, theta_deg - eps_deg, keep_left=True),
            halfplane(apex, theta_deg + eps_deg, keep_left=False)]


# ----------------------------------------------------------------------
# 2. 问题 1 主算法
# ----------------------------------------------------------------------
def localization_region(points, bearings, eps_deg=EPS_DEG,
                        target_r=TARGET_R, recv_r=RECV_R,
                        n_circle=N_CIRCLE, verbose=False):
    """
    输入：
        points   : [(x, y), ...]        检测点坐标
        bearings : [theta, ...]         对应示向度（度）
        eps_deg  : 误差半宽（度），默认 1
        target_r : 目标圆域半径，默认 1800
        recv_r   : 有效接收半径上界，默认 1500；设为 None 则不裁剪
    输出：
        poly : 定位区域凸多边形顶点列表（按顺序，逆时针或顺时针一致）
               若为空列表，说明约束不相容（各示向度不可能来自同一源）
    """
    # 起点：包围整个目标圆域的正多边形（近似半径 target_r 的圆）
    poly = [(target_r * math.cos(2 * math.pi * k / n_circle),
             target_r * math.sin(2 * math.pi * k / n_circle))
            for k in range(n_circle)]

    for idx, (S, th) in enumerate(zip(points, bearings)):
        for plane in wedge_planes(S, th, eps_deg):
            poly = clip_polygon(poly, plane)
            if not poly:
                if verbose:
                    print("  第 %d 个测点的楔形已使交集为空 → 约束不相容" % (idx + 1))
                return []
        if recv_r is not None:
            for plane in polygon_edges_halfplanes(S, recv_r, n_circle):
                poly = clip_polygon(poly, plane)
                if not poly:
                    return []
        if verbose:
            print("  加入测点 %d（θ=%.2f°）后：顶点数 = %d" % (idx + 1, th, len(poly)))
    return poly


def polygon_diameter(poly):
    """
    凸多边形直径 = 顶点两两距离的最大值。
    返回 (d, p, q)；多边形顶点数 ≤ 2n 很小，O(m^2) 足够。
    （要写得更漂亮可用旋转卡壳 O(m)。）
    """
    best = (0.0, None, None)
    for i in range(len(poly)):
        for j in range(i + 1, len(poly)):
            d = math.dist(poly[i], poly[j])
            if d > best[0]:
                best = (d, poly[i], poly[j])
    return best


def min_enclosing_circle(poly):
    """
    最小覆盖圆（MEC）：半径由 1/2/3 个顶点确定，枚举即可。
    返回 (r_star, center)。
    """
    pts = poly
    n = len(pts)
    best = [float("inf"), (0.0, 0.0)]

    def consider(cx, cy):
        r2 = max((p[0] - cx) ** 2 + (p[1] - cy) ** 2 for p in pts)
        if r2 < best[0]:
            best[0], best[1] = r2, (cx, cy)

    for i in range(n):
        consider(pts[i][0], pts[i][1])
        for j in range(i + 1, n):
            consider((pts[i][0] + pts[j][0]) / 2.0,
                     (pts[i][1] + pts[j][1]) / 2.0)
            for k in range(j + 1, n):
                (ax, ay), (bx, by), (cx0, cy0) = pts[i], pts[j], pts[k]
                dd = 2 * (ax * (by - cy0) + bx * (cy0 - ay) + cx0 * (ay - by))
                if abs(dd) < 1e-12:
                    continue
                ux = ((ax * ax + ay * ay) * (by - cy0) +
                      (bx * bx + by * by) * (cy0 - ay) +
                      (cx0 * cx0 + cy0 * cy0) * (ay - by)) / dd
                uy = ((ax * ax + ay * ay) * (cx0 - bx) +
                      (bx * bx + by * by) * (ax - cx0) +
                      (cx0 * cx0 + cy0 * cy0) * (bx - ax)) / dd
                consider(ux, uy)
    return math.sqrt(best[0]), best[1]


def coverage_answer(poly, tol=1e-6):
    """
    回答问题 1 的后半句：以【区域直径 d 为直径】的圆能否覆盖该区域？

    判据： 令 r* 为最小覆盖圆半径。以 d 为直径的圆半径是 d/2，
           能覆盖  <=>  r* == d/2 （直径两端点恰为 MEC 的对径点）。
    一般凸多边形并不成立（Jung 定理给出 d/2 <= r* <= d/sqrt(3)）。

    返回 dict：d, r_star, d_over_2, covered, ratio(2r*/d)
    """
    d, pa, pb = polygon_diameter(poly)
    r_star, c_star = min_enclosing_circle(poly)
    covered = (2.0 * r_star <= d + tol)
    return {"d": d, "r_star": r_star, "d_over_2": d / 2.0,
            "covered": covered, "ratio": (2.0 * r_star / d) if d else float("nan"),
            "center": c_star, "p": pa, "q": pb}


# ----------------------------------------------------------------------
# 3. 演示
# ----------------------------------------------------------------------
def _demo_teaching():
    print("=" * 66)
    print("演示 A：教学算例（ε 放大到 12°，只为看清裁剪过程）")
    print("=" * 66)
    S1, S2, G = (-450.0, -200.0), (450.0, -200.0), (60.0, 320.0)
    th1 = math.degrees(math.atan2(G[1] - S1[1], G[0] - S1[0]))
    th2 = math.degrees(math.atan2(G[1] - S2[1], G[0] - S2[0]))
    print("  真方位角：θ1 = %.2f°,  θ2 = %.2f°" % (th1, th2))

    # 为了演示看得清，这里把初始多边形换成一个已知的大矩形
    # （真实使用时起点是包围目标圆域的正 N 边形，见 localization_region）
    poly = [(-440.0, -180.0), (560.0, -180.0), (560.0, 820.0), (-440.0, 820.0)]
    print("  初始顶点数 = %d" % len(poly))

    steps = [("S1 下边界 θ1−ε", halfplane(S1, th1 - 12.0, True)),
             ("S1 上边界 θ1+ε", halfplane(S1, th1 + 12.0, False)),
             ("S2 下边界 θ2−ε", halfplane(S2, th2 - 12.0, True)),
             ("S2 上边界 θ2+ε", halfplane(S2, th2 + 12.0, False))]
    for i, (name, pl) in enumerate(steps, 1):
        poly = clip_polygon(poly, pl)
        print("  第 %d 步 [%s]：顶点数 = %d" % (i, name, len(poly)))
    d, pa, pb = polygon_diameter(poly)
    print("  最终区域：%d 个顶点，直径 d = %.1f" % (len(poly), d))
    print("  区域是否包含真实源 G：%s" % _point_in_convex(poly, G))


def _demo_real():
    print()
    print("=" * 66)
    print("演示 B：本题真实参数 ε = 1°，检验问题的两个结论")
    print("=" * 66)
    S1, S2, G = (-450.0, -200.0), (450.0, -200.0), (60.0, 320.0)
    th1 = math.degrees(math.atan2(G[1] - S1[1], G[0] - S1[0]))
    th2 = math.degrees(math.atan2(G[1] - S2[1], G[0] - S2[0]))

    poly = localization_region([S1, S2], [th1, th2], eps_deg=1.0, verbose=True)
    if not poly:
        print("  区域为空")
        return
    rep = coverage_answer(poly)
    print("  区域顶点数        = %d" % len(poly))
    print("  区域直径 d        = %.3f m" % rep["d"])
    print("  d/2               = %.3f m" % rep["d_over_2"])
    print("  最小覆盖圆半径 r* = %.3f m" % rep["r_star"])
    print("  2r*/d             = %.6f" % rep["ratio"])
    print("  以 d 为直径的圆能否覆盖区域：%s" % ("能" if rep["covered"] else "不能"))
    print("  区域是否包含真实源 G：%s" % _point_in_convex(poly, G))


def _point_in_convex(poly, pt, tol=1e-7):
    """点是否在凸多边形内（顶点需按同一方向排列）。"""
    n = len(poly)
    sign = 0
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        cr = (q[0] - p[0]) * (pt[1] - p[1]) - (q[1] - p[1]) * (pt[0] - p[0])
        if abs(cr) <= tol:
            continue
        s = 1 if cr > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return True


if __name__ == "__main__":
    _demo_teaching()
    _demo_real()
