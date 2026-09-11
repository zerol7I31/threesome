# -*- coding: utf-8 -*-
"""
"交会角越接近 90° 越好" —— 严格推导 + 数值验证

一、用什么算法得出这个结论？
    测角定位（AOA）的【线性化最小二乘 + 信息矩阵 / GDOP 分析】。
    步骤：把观测方程一阶泰勒展开 -> 构造雅可比 H -> 协方差 Cov = σ²(HᵀH)⁻¹
          -> 算 det(Cov) 与 trace(Cov) -> 看它们如何依赖交会角 α。

二、推导（本题符号）
    观测： β_i = atan2(y − y_i, x − x_i) + δ_i            （δ_i 为测角误差，零均值）
    一阶线性化：
        ∂β_i/∂x = −(y − y_i)/r_i² ,  ∂β_i/∂y = (x − x_i)/r_i²
        即 ∇β_i = n_i / r_i ,  n_i 是 r_i 单位法向（垂直于视线）
    雅可比 H 的第 i 行 = (1/r_i)·n_iᵀ

        HᵀH = Σ_i (1/r_i²) n_i n_iᵀ

    两站时令 n₁=(1,0)、n₂=(cosα, sinα)（α 即交会角，两条视线在目标处的夹角），
    记 a = 1/r₁², b = 1/r₂²，则

        HᵀH = [[ a + b·cos²α ,  b·cosα·sinα ],
               [ b·cosα·sinα ,  b·sin²α     ]]

        det(HᵀH) = a·b·sin²α = sin²α / (r₁² r₂²)

    所以

        det(Cov)   = σ⁴ / det(HᵀH) = σ⁴ · r₁² r₂² / sin²α
        sqrt(det Cov)              = σ² · r₁ r₂ / |sin α|        ← 不确定性"面积"∝ r₁r₂/sinα
        trace(Cov) = σ² (r₁² + r₂²) / sin²α
        GDOP       = sqrt(trace)/σ = sqrt(r₁² + r₂²) / |sin α|

    两个标量都在 sin α = 1（即 α = 90°）取到最小值 —— 这就是"交会角越接近 90° 越好"。

三、几何等价解释（为什么是 90°）
    测角误差 ±σ 让每条视线绕测点旋转，在目标附近形成一条"宽 2σ·r_i 的窄带"。
    两条窄带夹角为 α，它们相交得到平行四边形，
        两条边长分别为 2σ·r₁/sinα 与 2σ·r₂/sinα ，面积 = 4σ² r₁ r₂ / sinα。
    与上面的 det 结论完全一致(只差常数)。
    另有一个漂亮的等价说法：
        ∠(目标处两视线) = 90°  <=>  目标落在以 S₁S₂ 为直径的圆上（Thales 定理）
                              <=>  第二点位于"过源、且垂直于第一视线"的直线上。

作者备注：直接 python 运行即可。
"""

import math
import random
import sys

sys.path.insert(0, r"C:\Users\yangj\Desktop\B题")
from problem1_clip import clip_polygon, halfplane        # 复用问题 1 的半平面裁剪

EPS_DEG = 1.0
BOX = 1.0e5


# ----------------------------------------------------------------------
# 工具
# ----------------------------------------------------------------------
def _poly_area(poly):
    n = len(poly)
    return abs(sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1]
                   for i in range(n))) / 2.0


def wedge_region(S1, th1, S2, th2, eps):
    """两站扇形交会区域（用问题 1 的裁剪算法），返回顶点列表或 None（无界）。"""
    poly = [(-BOX, -BOX), (BOX, -BOX), (BOX, BOX), (-BOX, BOX)]
    for S, th in ((S1, th1), (S2, th2)):
        poly = clip_polygon(poly, halfplane(S, th - eps, True))
        poly = clip_polygon(poly, halfplane(S, th + eps, False))
        if not poly:
            return None
    for p in poly:
        if abs(p[0]) > BOX * 0.99 or abs(p[1]) > BOX * 0.99:
            return None
    return poly


def setup(alpha_deg, r1, r2):
    """目标置于原点；S1、S2 距目标分别为 r1、r2，两条视线在目标处夹角 = α。"""
    T = (0.0, 0.0)
    S1 = (-r1, 0.0)                                   # T→S1 方向 180°
    a = math.radians(180.0 - alpha_deg)
    S2 = (r2 * math.cos(a), r2 * math.sin(a))
    th1 = math.degrees(math.atan2(T[1] - S1[1], T[0] - S1[0]))   # S1 处看向 T 的方位角
    th2 = math.degrees(math.atan2(T[1] - S2[1], T[0] - S2[0]))
    return T, S1, S2, th1, th2


def intersect(P1, a1, P2, a2):
    """两条直线的交点：过 P1 方向 a1 度，过 P2 方向 a2 度。"""
    d1 = (math.cos(math.radians(a1)), math.sin(math.radians(a1)))
    d2 = (math.cos(math.radians(a2)), math.sin(math.radians(a2)))
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-15:
        return None
    t = ((P2[0] - P1[0]) * d2[1] - (P2[1] - P1[1]) * d2[0]) / den
    return (P1[0] + t * d1[0], P1[1] + t * d1[1])


def mc_covariance(alpha_deg, r1, r2, eps, K=40000, seed=1):
    """Monte-Carlo：直接模拟"两次测角 -> 交汇求交点"的估计量，统计其协方差。"""
    rng = random.Random(seed + int(alpha_deg * 1000))
    T, S1, S2, th1, th2 = setup(alpha_deg, r1, r2)
    xs, ys = [], []
    for _ in range(K):
        b1 = th1 + rng.uniform(-eps, eps)
        b2 = th2 + rng.uniform(-eps, eps)
        P = intersect(S1, b1, S2, b2)
        if P is None or abs(P[0]) > 1e7 or abs(P[1]) > 1e7:
            continue
        xs.append(P[0]); ys.append(P[1])
    m = len(xs)
    mx = sum(xs) / m; my = sum(ys) / m
    cxx = sum((x - mx) ** 2 for x in xs) / (m - 1)
    cyy = sum((y - my) ** 2 for y in ys) / (m - 1)
    cxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(m)) / (m - 1)
    det = cxx * cyy - cxy * cxy
    return math.sqrt(max(det, 0.0)), cxx + cyy, m


# ----------------------------------------------------------------------
# 主程序
# ----------------------------------------------------------------------
def main():
    r1 = r2 = 1000.0
    R = math.radians(EPS_DEG)
    sigma = R / math.sqrt(3.0)                     # U(-eps,eps) 的标准差
    print("=" * 100)
    print("设定：r1 = r2 = %.0f m, eps = %.1f deg, sigma = %.6f rad, 目标在原点"
          % (r1, EPS_DEG, sigma))
    print("=" * 100)

    # ---------- 验证 1：精确区域面积 vs 交会角（复现 1/sinα 律，并找最小值） ----------
    print()
    print("验证 1  两站扇形交会区域的【精确面积】随交会角 α 的变化")
    print("  α(°) | 精确面积(m²) | 归一化 | 理论 4ε²r₁r₂/sinα 归一化 | 比值")
    print("  " + "-" * 82)
    rows = []
    for alpha in range(10, 171, 10):
        T, S1, S2, th1, th2 = setup(alpha, r1, r2)
        poly = wedge_region(S1, th1, S2, th2, EPS_DEG)
        if poly is None:
            continue
        A = _poly_area(poly)
        theory = 4 * R * R * r1 * r2 / math.sin(math.radians(alpha))
        rows.append((alpha, A, theory))
        A90 = A
    base = min(r[1] for r in rows)
    for alpha, A, theory in rows:
        th90 = 4 * R * R * r1 * r2 / 1.0
        print("  %5d | %13.1f | %6.3f | %25.3f | %6.4f"
              % (alpha, A, A / base, theory / th90, A / theory))
    amin = min(rows, key=lambda z: z[1])
    print("  -> 精确面积最小值出现在 α = %d°（理论最小值在 90°）" % amin[0])

    # ---------- 验证 2：Monte-Carlo 估计量协方差 vs 交会角 ----------
    print()
    print("验证 2  Monte-Carlo：模拟'两次测角→交汇求交点'，统计估计量协方差（每格 4 万次）")
    print("  α(°) | sqrt(det Cov) 经验 | 理论 σ²r₁r₂/sinα | 比值 | 归一化 | GDOP 经验 | GDOP 理论")
    print("  " + "-" * 92)
    mc = []
    for alpha in range(10, 171, 10):
        s_det, tr, m = mc_covariance(alpha, r1, r2, EPS_DEG)
        th_det = sigma ** 2 * r1 * r2 / math.sin(math.radians(alpha))
        th_gdop = math.sqrt(r1 * r1 + r2 * r2) / math.sin(math.radians(alpha))
        mc.append((alpha, s_det, th_det, math.sqrt(tr) / sigma, th_gdop))
    bd = min(z[1] for z in mc)
    bg = min(z[3] for z in mc)
    for alpha, s_det, th_det, gd, th_gdop in mc:
        print("  %5d | %21.2f | %17.2f | %4.2f | %6.3f | %9.2f | %9.2f"
              % (alpha, s_det, th_det, s_det / th_det, s_det / bd, gd, th_gdop))
    a_det = min(mc, key=lambda z: z[1])[0]
    a_gdop = min(mc, key=lambda z: z[3])[0]
    print("  -> 经验 sqrt(det Cov) 最小值在 α = %d°" % a_det)
    print("  -> 经验 GDOP      最小值在 α = %d°" % a_gdop)

    # ---------- 验证 3：细网格找最优交会角 ----------
    print()
    print("验证 3  细网格（1° 步长）搜索使不确定性最小的交会角")
    best_det, best_a = None, None
    best_gdop, best_ag = None, None
    for alpha in range(1, 180):
        T, S1, S2, th1, th2 = setup(alpha, r1, r2)
        poly = wedge_region(S1, th1, S2, th2, EPS_DEG)
        if poly is None:
            continue
        A = _poly_area(poly)
        g = math.sqrt(r1 * r1 + r2 * r2) / math.sin(math.radians(alpha))
        if best_det is None or A < best_det:
            best_det, best_a = A, alpha
        if best_gdop is None or g < best_gdop:
            best_gdop, best_ag = g, alpha
    print("  按【精确区域面积】最小：α* = %d°  (面积 %.1f m²)" % (best_a, best_det))
    print("  按【GDOP】最小      ：α* = %d°" % best_ag)

    # ---------- 验证 4：几何等价 —— α = 90° 意味着什么 ----------
    print()
    print("验证 4  几何等价：α = 90°  <=>  目标在以 S1S2 为直径的圆上  <=>  S2 在'过源且垂直于视线1'的直线上")
    r = 800.0
    print("  （把 S1 置于原点、源置于 (r,0)，r = %.0f m，扫描第二点位置）" % r)
    print("  L(m)  φ(°)   α(°)   L·cosφ(m)  是否≈r")
    print("  " + "-" * 52)
    for L, phi in ((600, 60), (900, 60), (1000, 53), (1200, 48), (1600, 60)):
        S1 = (0.0, 0.0)
        G = (r, 0.0)
        S2 = (L * math.cos(math.radians(phi)), L * math.sin(math.radians(phi)))
        a1 = math.degrees(math.atan2(G[1] - S1[1], G[0] - S1[0]))
        a2 = math.degrees(math.atan2(G[1] - S2[1], G[0] - S2[0]))
        d1 = math.degrees(math.atan2(S1[1] - G[1], S1[0] - G[0]))
        d2 = math.degrees(math.atan2(S2[1] - G[1], S2[0] - G[0]))
        da = abs(d1 - d2) % 360
        if da > 180:
            da = 360 - da
        print("  %5.0f  %5.0f  %6.1f  %10.1f  %s"
              % (L, phi, da, L * math.cos(math.radians(phi)),
                 "是" if abs(da - 90) < 2 else "否"))
    print("  说明：要让 α 精确等于 90°，必须满足 L·cosφ = r，")
    print("        而 r（源的距离）是未知量 —— 这正是问题 2 里'最优 φ* 不是 90°'的原因：")
    print("        我们只能在 r 的不确定性下折中，而不是直接把交会角摆成 90°。")


if __name__ == "__main__":
    main()
