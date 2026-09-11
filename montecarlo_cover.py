# -*- coding: utf-8 -*-
"""
B 题 问题 1 后半句的 Monte-Carlo 检验（严谨版）
"以定位区域直径为直径的圆，能否覆盖该定位区域？"

一、统计量设计（先想清楚"要检验什么"）
--------------------------------------
记区域 K 的直径为 d，最小覆盖圆（MEC）半径为 r*。
因为 K 中任意两点距离 <= 2 r*，所以恒有 d <= 2 r*。定义【超额量】

        E = 2 r* / d - 1   ( >= 0 )

则     E = 0  <=>  r* = d/2  <=>  以 d 为直径的圆能覆盖 K
       E > 0  <=>  不能覆盖，且 E 就是"半径相对放大比例"

所以 Monte-Carlo 不是去"数失败次数"，而是去问：
        E 能不能取到正值？最大能到多少？在什么条件下变大？

二、必须控制的三件事（否则实验结论不可信）
------------------------------------------
1) 误差也要随机：示向度 θ_i = 真方位角 + δ_i,  δ_i ~ U(-ε, ε)。
   若取 δ=0，所有楔形都关于真方向对称，区域过于规整，会低估失败率。
2) 物理可行性：源必须满足 |G - S_i| <= 1500（否则该站测不到该源）。
3) 离散化方向：圆盘既用【内接】也用【外接】多边形各跑一遍。
   内接 -> 区域偏小（乐观）；外接 -> 区域偏大（保守）。
   真实连续区域夹在两者之间，这样才能判断"失败"是真几何效应还是离散化假象。

三、还要做的两件事
------------------
4) 对抗性搜索：主动优化配置去最大化 E。随机采样打不到的角落，优化器能打到。
5) 统计不确定性：N 次试验 0 失败时，失败概率的 95% 置信上界约 3/N（"三倍法则"）。

作者备注：直接 python 运行即可。
"""

import math
import random

TARGET_R = 1800.0
RECV_R = 1500.0
NSEG = 90
TOL = 1e-12
JUNG_E = 2.0 / math.sqrt(3.0) - 1.0      # 正三角形情形 E = 2/sqrt(3) - 1 ≈ 0.1547


# ----------------------------------------------------------------------
# 几何内核
# ----------------------------------------------------------------------
def _clip(poly, a, b, c):
    """Sutherland–Hodgman：保留半平面 a x + b y + c >= 0。"""
    if not poly:
        return []
    out = []
    n = len(poly)
    for i in range(n):
        p = poly[i]
        q = poly[(i + 1) % n]
        fp = a * p[0] + b * p[1] + c
        fq = a * q[0] + b * q[1] + c
        if fp >= -TOL:
            out.append(p)
        if (fp > TOL and fq < -TOL) or (fp < -TOL and fq > TOL):
            t = fp / (fp - fq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def _hp(apex, deg, keep_left=True):
    """射线(顶点 apex, 方向 deg) 的半平面；保留 (P-apex)·n >= 0 一侧。"""
    a = math.radians(deg)
    nx, ny = -math.sin(a), math.cos(a)
    c = -(nx * apex[0] + ny * apex[1])
    return (nx, ny, c) if keep_left else (-nx, -ny, -c)


def _circle_polygon(center, radius, n, kind):
    """kind: 'inscribed' 内接（偏小） / 'circumscribed' 外接（偏大）。"""
    R = radius / math.cos(math.pi / n) if kind == "circumscribed" else radius
    return [(center[0] + R * math.cos(2 * math.pi * k / n),
             center[1] + R * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _poly_halfplanes(poly):
    """由凸多边形顶点（同向）生成其各边的内侧半平面。"""
    planes = []
    n = len(poly)
    # 判断顶点走向，保证"内侧"方向正确
    area2 = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1]
                for i in range(n))
    sgn = 1.0 if area2 > 0 else -1.0
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        ex, ey = q[0] - p[0], q[1] - p[1]
        nx, ny = -ey * sgn, ex * sgn
        m = math.hypot(nx, ny)
        nx, ny = nx / m, ny / m
        planes.append((nx, ny, -(nx * p[0] + ny * p[1])))
    return planes


def region(stations, bearings, eps, disk_kind="circumscribed", nseg=NSEG):
    """
    定位区域
        K = 目标圆域 ∩ ⋂楔形(S_i, θ_i ± ε) ∩ ⋂圆盘(S_i, 1500)
    disk_kind = 'none'  -> 不做物理裁剪（= 题图 2 定义下的纯交会区域）
                'inscribed' / 'circumscribed' -> 用内接/外接 Polygon 近似圆盘
    返回顶点列表；空列表表示约束不相容。
    """
    kind = "circumscribed" if disk_kind in ("circumscribed",) else "inscribed"
    poly = _circle_polygon((0.0, 0.0), TARGET_R, nseg, kind)
    for S, th in zip(stations, bearings):
        poly = _clip(poly, *_hp(S, th - eps, True))
        poly = _clip(poly, *_hp(S, th + eps, False))
        if not poly:
            return []
    if disk_kind == "none":
        return poly
    for S in stations:
        if max(math.dist(S, p) for p in poly) <= RECV_R:
            continue                       # 顶点全在真实圆内 -> 裁剪无效果
        for pl in _poly_halfplanes(_circle_polygon(S, RECV_R, nseg, kind)):
            poly = _clip(poly, *pl)
            if not poly:
                return []
    return poly


def _diam(poly):
    best = (0.0, None, None)
    n = len(poly)
    for i in range(n):
        for j in range(i + 1, n):
            d = math.dist(poly[i], poly[j])
            if d > best[0]:
                best = (d, poly[i], poly[j])
    return best


def _mec(poly):
    n = len(poly)
    best = [float("inf"), (0.0, 0.0)]

    def consider(cx, cy):
        r2 = max((p[0] - cx) ** 2 + (p[1] - cy) ** 2 for p in poly)
        if r2 < best[0]:
            best[0], best[1] = r2, (cx, cy)

    for i in range(n):
        consider(*poly[i])
        for j in range(i + 1, n):
            consider((poly[i][0] + poly[j][0]) / 2, (poly[i][1] + poly[j][1]) / 2)
            for k in range(j + 1, n):
                (ax, ay), (bx, by), (cx0, cy0) = poly[i], poly[j], poly[k]
                dd = 2 * (ax * (by - cy0) + bx * (cy0 - ay) + cx0 * (ay - by))
                if abs(dd) < 1e-14:
                    continue
                ux = ((ax * ax + ay * ay) * (by - cy0) + (bx * bx + by * by) * (cy0 - ay)
                      + (cx0 * cx0 + cy0 * cy0) * (ay - by)) / dd
                uy = ((ax * ax + ay * ay) * (cx0 - bx) + (bx * bx + by * by) * (ax - cx0)
                      + (cx0 * cx0 + cy0 * cy0) * (bx - ax)) / dd
                consider(ux, uy)
    return math.sqrt(best[0]), best[1]


def _area(poly):
    n = len(poly)
    return abs(sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1]
                   for i in range(n))) / 2.0


def excess(poly):
    """返回 (E, d, n_vert, aspect)；E = 2r*/d - 1 >= 0。"""
    d, _pa, _pb = _diam(poly)
    if d <= 1e-9:
        return None
    r, _c = _mec(poly)
    a = _area(poly)
    aspect = (d * d / a) if a > 1e-9 else float("inf")
    return (2.0 * r / d - 1.0), d, len(poly), aspect


def _bearing(S, G):
    return math.degrees(math.atan2(G[1] - S[1], G[0] - S[0]))


# ----------------------------------------------------------------------
# 采样器
# ----------------------------------------------------------------------
def sample_config(n, eps, rng, spread_max=2500.0):
    """在目标圆域内合法采样：源须在每站接收半径内；示向度含随机误差 δ∈[-ε,ε]。"""
    for _ in range(200):
        c0 = (rng.uniform(-1500, 1500), rng.uniform(-1500, 1500))
        spread = math.exp(rng.uniform(math.log(60), math.log(spread_max)))
        stations = []
        for _i in range(n):
            ang = rng.uniform(0, 2 * math.pi)
            rr = spread * math.sqrt(rng.random())
            stations.append((c0[0] + rr * math.cos(ang), c0[1] + rr * math.sin(ang)))
        G = (rng.uniform(-TARGET_R, TARGET_R), rng.uniform(-TARGET_R, TARGET_R))
        if math.hypot(*G) > TARGET_R:
            continue
        if all(math.dist(S, G) <= RECV_R for S in stations):
            bearings = [_bearing(S, G) + rng.uniform(-eps, eps) for S in stations]
            return stations, bearings
    return None


# ----------------------------------------------------------------------
# 实验 A：ε × n × 离散化方向 的失败率与最大超额量
# ----------------------------------------------------------------------
def _pct(v, q):
    if not v:
        return float("nan")
    v = sorted(v)
    i = min(len(v) - 1, max(0, int(round(q * (len(v) - 1)))))
    return v[i]


def exp_a(trials=6000, seed=20260911):
    rng = random.Random(seed)
    print("实验 A：随机 Monte-Carlo（每格 %d 次）" % trials)
    print("  ε(°) | n | 圆盘处理     | 有效 | 失败率 E>1e-9 | 失败率 E>1e-3 | 中位E | p99.9 E | 最大E")
    print("  " + "-" * 104)
    for eps in (0.25, 1.0, 2.0, 5.0, 15.0):
        for n in (2, 3, 4):
            for kind in ("none", "inscribed", "circumscribed"):
                Es = []
                for _ in range(trials):
                    cfg = sample_config(n, eps, rng)
                    if cfg is None:
                        continue
                    poly = region(*cfg, eps, disk_kind=kind)
                    if len(poly) < 3:
                        continue
                    r = excess(poly)
                    if r:
                        Es.append(r[0])
                if not Es:
                    continue
                f1 = sum(1 for e in Es if e > 1e-9) / len(Es)
                f3 = sum(1 for e in Es if e > 1e-3) / len(Es)
                print("  %5.2f | %d | %-12s | %4d | %13.4f%% | %13.4f%% | %.1e | %.1e | %.1e"
                      % (eps, n, kind, len(Es), f1 * 100, f3 * 100,
                         _pct(Es, 0.5), _pct(Es, 0.999), max(Es)))


# ----------------------------------------------------------------------
# 实验 B：对抗性搜索（主动最大化 E），并给出诊断信息
# ----------------------------------------------------------------------
def _config_to_poly(x, eps, n, disk_kind):
    r = x[0]
    G = (r, 0.0)
    stations = [(0.0, 0.0)]
    for k in range(n - 1):
        L, phi = x[1 + 2 * k], x[2 + 2 * k]
        stations.append((L * math.cos(phi), L * math.sin(phi)))
    if any(math.dist(S, G) > RECV_R for S in stations):
        return None
    offs = [x[1 + 2 * (n - 1) + k] for k in range(n)]
    if any(abs(o) > eps for o in offs):
        return None
    bearings = [_bearing(S, G) + offs[k] for k, S in enumerate(stations)]
    poly = region(stations, bearings, eps, disk_kind=disk_kind)
    return poly if len(poly) >= 3 else None


def exp_b(n, eps, disk_kind="circumscribed", restarts=30, iters=110, seed=7):
    rng = random.Random(seed + n * 131 + int(eps * 17))
    ndim = 1 + 2 * (n - 1) + n

    def rand_x():
        x = [rng.uniform(200, RECV_R)]
        for _k in range(n - 1):
            x += [rng.uniform(50, 3000), rng.uniform(0, 2 * math.pi)]
        x += [rng.uniform(-eps, eps) for _k in range(n)]
        return x

    def score(x):
        poly = _config_to_poly(x, eps, n, disk_kind)
        if poly is None:
            return None, None
        return excess(poly)[0], poly

    best, best_poly = 0.0, None
    for _ in range(restarts):
        x = rand_x()
        s, poly = score(x)
        if s is None:
            continue
        step = [RECV_R * 0.3] + [400.0, 0.6] * (n - 1) + [eps * 0.5] * n
        for _it in range(iters):
            improved = False
            for k in range(ndim):
                for sgn in (1.0, -1.0):
                    xt = list(x)
                    xt[k] += sgn * step[k]
                    if k == 0 and not (1.0 < xt[0] <= RECV_R):
                        continue
                    st, pt = score(xt)
                    if st is not None and st > s + 1e-15:
                        x, s, poly = xt, st, pt
                        improved = True
            if not improved:
                step = [v * 0.5 for v in step]
                if max(step) < 1e-9:
                    break
        if s > best:
            best, best_poly = s, poly
    info = ""
    if best_poly is not None:
        e, d, nv, asp = excess(best_poly)
        info = "顶点数=%d, d=%.1f, 细长度 d²/面积=%.1f" % (nv, d, asp)
    return best, info


def exp_b_report():
    print()
    print("实验 B：对抗性搜索（随机重启 + 坐标下降，主动最大化 E）")
    print("  参考：正三角形情形 E = 2/√3 − 1 = %.4f（Jung 上界）" % JUNG_E)
    print("  ε(°) | n | 搜索到的最大 E | 与 Jung 上界之比 | 最优区域诊断")
    print("  " + "-" * 92)
    for eps in (1.0, 5.0, 15.0):
        for n in (2, 3, 4):
            b, info = exp_b(n, eps, restarts=25 if n <= 3 else 10,
                            iters=90 if n <= 3 else 50)
            print("  %5.2f | %d | %.6e | %14.4f | %s" % (eps, n, b, b / JUNG_E, info))


# ----------------------------------------------------------------------
# 实验 C：平行四边形引理的数值核对（对照实验）
# ----------------------------------------------------------------------
def exp_c(trials=20000, seed=99):
    rng = random.Random(seed)
    mx = 0.0
    for _ in range(trials):
        u = (rng.uniform(-1e4, 1e4), rng.uniform(-1e4, 1e4))
        v = (rng.uniform(-1e4, 1e4), rng.uniform(-1e4, 1e4))
        poly = [(0.0, 0.0), u, (u[0] + v[0], u[1] + v[1]), v]
        if _area(poly) < 1e-6:
            continue
        e = excess(poly)[0]
        mx = max(mx, e)
    print()
    print("实验 C：随机平行四边形对照（引理：较长对角线为直径的圆必覆盖平行四边形）")
    print("  %d 个随机平行四边形，最大 E = %.3e  -> 恒为 0，引理成立" % (trials, mx))


if __name__ == "__main__":
    exp_c()
    exp_a(trials=6000)
    exp_b_report()
