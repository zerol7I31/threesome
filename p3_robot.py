# -*- coding: utf-8 -*-
"""
B 题 问题 3 —— 机器狗策略 + 演练驱动

用法（三个子命令）：
    python p3_robot.py mock   --episodes 20                离线演练：跑 20 局，输出两项统计
    python p3_robot.py sweep  --episodes 8                 参数扫描：对比不同参数组合
    python p3_robot.py live   --url http://127.0.0.1:2026 --team 你的参赛队号
                                                           连真实模拟器跑一局

策略结构（分层，便于逐层调参）
    L1 侦察    : 在若干侦察点扫全部频道，得到"候选频道 + 方位约束"
    L2 交会定位: 复用问题 1 的半平面裁剪求区域 K，用 diam(K) 作收敛判据
    L3 逼近清除: 按问题 2 的思路选下一个测点（侧向偏置，避免共线），收敛后 clear
    L4 验证退出: 对未清除频道做多方位确认，然后 /exit
"""

import math
import sys
import time

from p3_arena import MockArena, HttpArena
from problem1_clip import clip_polygon, halfplane

TARGET_R = 1800.0
RECV_R = 1500.0
NSEG = 180
EPS = 1.0

DEFAULT_PARAMS = {
    "robot_id": "T000",
    "eps": 1.0,
    "tol": 40.0,             # diam(K) 收敛阈值（m）：小于它才尝试 clear
    "k_ring": 6,             # 内侦察环点数（另有原点共 k_ring+1 个内圈侦察点）
    "r_ring": 1000.0,        # 内侦察环半径（覆盖中央区）
    "ring_phase": 30.0,      # 内侦察环起始方位角（度）
    "k_ring_outer": 12,      # 外侦察环点数（覆盖边界区；加密到 15° 间隔，兜住"可见区是边界窄缝"的朝外发射定向源）
    "r_ring_outer": 1780.0,  # 外侦察环半径（贴近 1800 边界，兜住"发射方向朝外、可见区大半在场地外"的定向源）
    "ring_phase_outer": 11.25,
    "ray_step": 200.0,       # 射线法线性扫描步长（m）
    "ray_binary_iters": 12,  # 射线法二分精化次数（≈ 1500/2^12 < 1 m）
    "k_verify": 3,           # （旧的固定验证环点数，已被覆盖缺口驱动策略取代）
    "r_verify": 1300.0,
    "verify_phase": 90.0,
    "recv_min": 1000.0,      # 有效接收半径的下界（附件：1000~1500）→ 覆盖缺口的判据
    "max_verify_points": 6,  # 覆盖缺口驱动的验证最多补几个点（旧固定环用，保留）
    "max_verify_iters": 80,  # L4 逐频道补缺最多迭代轮数（不限轮次直到全覆盖，避免漏清）
    "opportunistic": True,   # 逼近途中顺手扫未解频道（不额外移动）
    "poll_min_dist": 250.0,  # 两次顺手扫之间至少间隔的移动距离
    "max_polls": 14,         # 每个未解频道最多被顺手扫几次
    "max_iters": 40,         # 单个频道的最大迭代次数
    "max_rounds": 4,         # 主循环最大轮数
    "max_attempts": 2,       # 同一频道失败几次后放弃（防死循环）
    "max_vtime": 60000.0,    # 虚拟时间安全上限（秒），防止意外空转
    "step_ratio": 1.0,       # 新测点距离 = step_ratio × 当前估计距离（1.0 ≈ 直冲，收敛最快）
    "step_min": 30.0,
    "step_max": 1500.0,
    "step_angle": 10.0,      # 新测点相对视线的偏角（度）：兼顾逼近与交会角
    "real_budget": 1050.0,   # 现实时间预算（秒），超过就收工
}


# ----------------------------------------------------------------------
# 几何工具（问题 1 的复用版，针对本场景做了速度裁剪）
# ----------------------------------------------------------------------
def _disk_polygon(cx, cy, r, n=NSEG):
    return [(cx + r * math.cos(2 * math.pi * k / n),
             cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _disk_planes(cx, cy, r, n=NSEG):
    planes = []
    for k in range(n):
        a1 = 2 * math.pi * k / n
        a2 = 2 * math.pi * (k + 1) / n
        p1 = (cx + r * math.cos(a1), cy + r * math.sin(a1))
        p2 = (cx + r * math.cos(a2), cy + r * math.sin(a2))
        ex, ey = p2[0] - p1[0], p2[1] - p1[1]
        nx, ny = -ey, ex
        m = math.hypot(nx, ny)
        planes.append((nx / m, ny / m, -(nx / m * p1[0] + ny / m * p1[1])))
    return planes


def region_of(constraints, eps=EPS, recv_r=RECV_R, n=NSEG):
    """由 [(S, θ), ...] 求定位区域 K：目标圆域 ∩ 各楔形 ∩ 各接收半径圆。"""
    poly = _disk_polygon(0.0, 0.0, TARGET_R, n)
    for S, th in constraints:
        poly = clip_polygon(poly, halfplane(S, th - eps, True))
        poly = clip_polygon(poly, halfplane(S, th + eps, False))
        if len(poly) < 3:
            return []
    for S, _th in constraints:
        if all(math.dist(S, p) <= recv_r for p in poly):
            continue
        for pl in _disk_planes(S[0], S[1], recv_r, n):
            poly = clip_polygon(poly, pl)
            if len(poly) < 3:
                return []
    return poly


def poly_centroid(poly):
    sx = sum(p[0] for p in poly)
    sy = sum(p[1] for p in poly)
    return (sx / len(poly), sy / len(poly))


def poly_diameter(poly):
    best = (0.0, None, None)
    for i in range(len(poly)):
        for j in range(i + 1, len(poly)):
            d = math.dist(poly[i], poly[j])
            if d > best[0]:
                best = (d, poly[i], poly[j])
    return best


def rot90(u, sgn=1.0):
    return (-u[1] * sgn, u[0] * sgn)


def unit(v):
    m = math.hypot(v[0], v[1])
    return (0.0, 0.0) if m < 1e-12 else (v[0] / m, v[1] / m)


# ----------------------------------------------------------------------
# 机器狗
# ----------------------------------------------------------------------
class Robot(object):
    def __init__(self, arena, params=None):
        self.a = arena
        self.p = dict(DEFAULT_PARAMS)
        if params:
            self.p.update(params)
        self._rid = 0
        self.constraints = {c: [] for c in range(1, 21)}
        self.candidates = set()
        self.cleared = set()
        self.dead = set()                 # 判定为"无源"的频道
        self.gaveup = set()               # 反复失败后放弃的频道（防死循环）
        self.attempts = {}
        self.visited = [(0.0, 0.0)]       # 所有测过的位置
        self.scan_points = [(0.0, 0.0)]   # 真正"扫过未解频道"的位置（覆盖缺口判据用这个）
        self.channel_scan = {c: [(0.0, 0.0)] for c in range(1, 21)}  # 逐频道"扫过的位置"——L4 按频道独立补缺用
        self.last_poll = None
        self.poll_count = {}
        self.measured = {}                # (x,y,ch) -> 响应缓存：同点同频道必同结果，不必重测
        self.pos = (0.0, 0.0)
        self.vtime = 0.0
        self.t0 = None
        self._side = 1.0
        self.stats = {"measures": 0, "clears": 0, "misses": 0, "moves": 0.0}

    # ---------- 通信 ----------
    def call(self, path, x=None, y=None, ch=None, wait_s=0.0):
        self._rid += 1
        payload = {"arena_id": "default", "robot_id": self.p["robot_id"],
                   "request_id": "%s-%d" % (path[1:], self._rid)}
        if x is not None:
            payload["position"] = {"x": float(x), "y": float(y)}
        if ch is not None:
            payload["channel"] = int(ch)
        if wait_s > 0 and hasattr(self.a, "request_patient"):
            resp = self.a.request_patient(path, payload, wait_s)
        else:
            resp = self.a.request(path, payload)
        if resp.get("accepted") is True:
            if "virtual_time_s" in resp:
                self.vtime = max(self.vtime, float(resp["virtual_time_s"]))
            if x is not None:
                self.stats["moves"] += math.dist(self.pos, (float(x), float(y)))
                self.pos = (float(x), float(y))
        return resp

    def measure(self, P, ch):
        """
        (位置, 频道) 去重：题目说误差是"位置固定偏差"，同一位置对同一频道重复检测结果完全一样，
        再测一次纯属浪费 5 s + 1 s。命中缓存时直接返回上次结果，不发出任何指令。
        """
        key = (round(P[0], 3), round(P[1], 3), int(ch))
        if key in self.measured:
            self.stats["cached"] = self.stats.get("cached", 0) + 1
            return self.measured[key]
        r = self.call("/measure", P[0], P[1], ch)
        self.stats["measures"] += 1
        if r.get("accepted") is True:
            self.measured[key] = {"accepted": True, "cached": True,
                                  "measure_result": r.get("measure_result"),
                                  "svd_deg": r.get("svd_deg")}
            if P not in self.visited:
                self.visited.append((P[0], P[1]))
        return r

    def clear(self, P, ch):
        r = self.call("/clear", P[0], P[1], ch)
        self.stats["clears"] += 1
        if r.get("clear_result") != "success":
            self.stats["misses"] += 1
        return r

    def out_of_time(self):
        return self.t0 is not None and (time.time() - self.t0) > self.p["real_budget"]

    # ---------- 观测消化 ----------
    def ingest(self, ch, P, resp):
        """把一次 /measure 结果并入该频道的约束集合。返回 True 表示该频道已清除。"""
        res = resp.get("measure_result")
        if res == "near":
            c = self.clear(P, ch)
            if c.get("clear_result") == "success":
                self.cleared.add(ch)
                return True
            # 极少见：near 却没清掉（理论上不会）
            return False
        if res == "direction":
            self.constraints[ch].append((P, float(resp["svd_deg"])))
            self.candidates.add(ch)
        return False

    # ---------- 选点策略（对应问题 2） ----------
    def pick_next(self, ch, G, dist_to_G):
        """
        以【当前位置】为基准取下一个测点（关键：不要以旧测点为基准，否则会来回跳）。
        方向 = 朝估计点 G 前进、再偏转 step_angle；距离 = step_ratio × 当前估计距离。
        这样每走一步同时干两件事：逼近源 + 制造横向基线让交会角不至于退化成 0。
        左右交替偏转，避免一直朝同一侧绕圈。
        """
        P = self.pos
        u = unit((G[0] - P[0], G[1] - P[1]))
        if u == (0.0, 0.0):
            u = (1.0, 0.0)
        r_hat = max(math.dist(P, G), 1.0)
        L = self.p["step_ratio"] * r_hat
        L = max(self.p["step_min"], min(self.p["step_max"], L))
        psi = math.radians(self.p["step_angle"])
        n = rot90(u, self._side)
        self._side = -self._side
        d = (u[0] * math.cos(psi) + n[0] * math.sin(psi),
             u[1] * math.cos(psi) + n[1] * math.sin(psi))
        return (P[0] + L * d[0], P[1] + L * d[1])

    # ---------- 单频道：定位并清除 ----------
    def solve_channel(self, ch, max_iters=None):
        iters = max_iters or self.p["max_iters"]
        # 顺手扫：当前位置反正要停下来测，顺便把未解频道也测一遍 —— 零额外移动。
        self.maybe_opportunistic_scan()
        # 当前位置若尚未对该频道贡献过方位，且离已有测点足够远，先顺手测一次，
        # 可能直接拿到第 2 条方位（比射线法更省时），也可能恰好 near 直接清掉。
        if not self.constraints[ch] or all(math.dist(self.pos, P) > 50.0
                                           for P, _t in self.constraints[ch]):
            if self.ingest(ch, self.pos, self.measure(self.pos, ch)):
                return True
        if not self.constraints[ch]:
            return False                     # 当前位置无信号且无任何方位 → 交给后续补测

        # 仅 1 条方位（定向源典型情况）→ 先沿射线试"直达"；不行再用垂向补方位 + 解析交会
        if len(self.constraints[ch]) == 1:
            if self.ray_march(ch):
                return True
            return self.solve_directional(ch)

        # ≥2 条方位 → 标准 AOA 交会（全向源与"双侧可见"定向源更快）
        if self._solve_aoa(ch, iters):
            return True
        # AOA 因两条远方位夹成"长条状"区域迟迟不收敛 → 解析最小二乘交会兜底（定向源）
        return self.solve_directional(ch)

    # ---------- AOA 交会（≥2 条方位） ----------
    def _solve_aoa(self, ch, iters):
        for _ in range(iters):
            if self.out_of_time():
                return False
            poly = region_of(self.constraints[ch], eps=self.p["eps"])
            if len(poly) < 3:
                return False                     # 约束自相矛盾（越过覆盖边界）→ 交给定向射线法
            d, _pa, _pb = poly_diameter(poly)
            G = poly_centroid(poly)
            if d <= self.p["tol"]:
                r = self.clear(G, ch)
                if r.get("clear_result") == "success":
                    self.cleared.add(ch)
                    return True
                # 估计偏了 → 在区域附近换个点重测，补充约束
                off = max(8.0, d * 0.5)
                P = (G[0] + off, G[1])
                if self.ingest(ch, P, self.measure(P, ch)):
                    return True
                continue
            dist = math.dist(self.constraints[ch][-1][0], G)
            P = self.pick_next(ch, G, max(dist, self.p["tol"]))
            resp = self.measure(P, ch)
            if self.ingest(ch, P, resp):
                return True
            if resp.get("measure_result") == "no_signal":
                # 走过头/超出接收半径/越过覆盖边界 → 朝估计点回撤一步再测
                back = (self.pos[0] + 0.35 * (G[0] - self.pos[0]),
                        self.pos[1] + 0.35 * (G[1] - self.pos[1]))
                if self.ingest(ch, back, self.measure(back, ch)):
                    return True
        return False

    # ---------- 射线法 + 垂向补方位（仅 1 条方位的定向源） ----------
    def ray_march(self, ch):
        """
        仅拿到 1 条方位(检测点 V、方位 θ)时给定向源定位。

        关键物理：源在 V 的方位射线 L 上、距离 ≤ 接收半径。沿 L 向外扫描：
          · 若源落在"覆盖半球"内（多数情况），走到源前一直是 direction，越过后变
            no_signal 或直接 near → 直接清除（必中）。
          · 但若源正好处在"覆盖半球边界的盲侧"（边界发射型源），信号/无信号跃变点
            会落在源【之前】，源本身在盲区里、沿 L 够不着。此时跃变点与源都落在 L 上，
            单条射线无法定位源（自由度不足）。

        第二种情况的处理（重点）：用跃变点确定"覆盖半球仍在内侧"的区间，在该区间内
        取一个基点、向垂直方向偏移拿一条【非共线】的第 2 方位 → 回到标准 AOA 交会，
        区域被两个楔形夹小、收敛即清。这是定向源相对全向源"多出来"的信息的合规用法。
        """
        if not self.constraints[ch]:
            return False
        import os as _os
        dbg = _os.environ.get("P4DEBUG") == "1"
        V, theta = self.constraints[ch][0]
        a = math.radians(theta)
        ux, uy = math.cos(a), math.sin(a)
        px, py = -uy, ux                       # 垂直方向单位向量
        # 射线在 1800 圆内可达的最大 t
        VV = V[0] * V[0] + V[1] * V[1]
        bdot = V[0] * ux + V[1] * uy
        disc = bdot * bdot - (VV - TARGET_R * TARGET_R)
        t_reach = (-bdot + math.sqrt(disc)) if disc >= 0 else 1500.0
        t_reach = min(t_reach, 1500.0)
        if dbg:
            print("  [ray] ch=%d V=(%.0f,%.0f) theta=%.1f t_reach=%.1f"
                  % (ch, V[0], V[1], theta, t_reach))
        step = self.p["ray_step"]
        last_cov = 0.0
        t = 0.0
        while t < t_reach - 1e-6:
            nt = min(t + step, t_reach)
            P = (V[0] + nt * ux, V[1] + nt * uy)
            res = self.measure(P, ch).get("measure_result")
            if dbg:
                print("  [ray]   t=%.0f P=(%.0f,%.0f) -> %s" % (nt, P[0], P[1], res))
            if res == "near":
                self.clear(P, ch)
                self.cleared.add(ch)
                return True
            if res == "direction":
                if nt < t_reach - 1e-6:
                    last_cov = nt
                t = nt
                continue
            # 越过源（覆盖区内）或越过覆盖边界（盲侧）→ 出现"信号→无信号"跃变
            return self._ray_to_aoa(ch, V, ux, uy, px, py, last_cov, nt, dbg, True)
        # 全程都有信号（覆盖半球朝外、射线始终在覆盖区）→ 没有过源跃变
        return self._ray_to_aoa(ch, V, ux, uy, px, py, last_cov, t_reach, dbg, False)

    def _ray_binary(self, lo, hi, V, ux, uy, ch, dbg=False):
        """二分精化"信号→无信号"跃变点（沿射线参数 t）。返回最后有信号的位置 t*。"""
        for _ in range(self.p["ray_binary_iters"]):
            mid = 0.5 * (lo + hi)
            P = (V[0] + mid * ux, V[1] + mid * uy)
            res = self.measure(P, ch).get("measure_result")
            if dbg:
                print("  [bin]  t=%.1f P=(%.0f,%.0f) -> %s" % (mid, P[0], P[1], res))
            if res == "near":
                self.clear(P, ch)
                self.cleared.add(ch)
                return None
            if res == "direction":
                lo = mid
            else:
                hi = mid
        return lo

    def _ray_to_aoa(self, ch, V, ux, uy, px, py, lo, hi, dbg=False, transition=False):
        """仅 1 条方位时，造第 2 条非共线方位，转标准 AOA。

        两种情形：
          · transition=True  —— 沿射线出现了 信号→无信号 跃变（边界发射型源特征：
            源就在这条射线上）。先二分精化跃变点 t*，以其为基点做垂直偏移补方位，
            AOA 交点天然落在真源。
          · transition=False —— 全程有信号（覆盖半球朝外，射线一直在覆盖区）。
            以射线末端 t_reach（已知在覆盖区+接收半径内）为基点做垂直偏移补方位。
        关键修复：补到方位后必须 ingest 并入 constraints，否则 _solve_aoa 仍只有 1 条
        方位（退化为 2° 楔形）而失败。都补不到再退回其它侦察点重测（_grab_second_bearing）。
        """
        if transition and hi - lo > 1.0:
            t_star = self._ray_binary(lo, hi, V, ux, uy, ch, dbg)
            if t_star is not None:
                P = (V[0] + t_star * ux, V[1] + t_star * uy)
                r = self.clear(P, ch)
                if r.get("clear_result") == "success":
                    self.cleared.add(ch)
                    return True
                # 没清掉：源在覆盖侧、离跃变点有距离（覆盖边界/R 限制型），
                # 改用 lo（最后有信号点，必在覆盖半球内侧）作基点补第 2 方位
            base = lo
        else:
            base = 0.5 * (lo + hi)        # 无跃变时 lo==hi==t_reach
        base = max(60.0, base)
        Pb = (V[0] + base * ux, V[1] + base * uy)
        for d_off in (80.0, -80.0, 160.0, -160.0, 40.0, -40.0,
                     250.0, -250.0, 120.0, -120.0):
            Pp = (Pb[0] + d_off * px, Pb[1] + d_off * py)
            if math.hypot(Pp[0], Pp[1]) > TARGET_R - 10:
                continue
            resp = self.measure(Pp, ch)
            res = resp.get("measure_result")
            if dbg:
                print("  [perp]  d=%.0f P=(%.0f,%.0f) -> %s" % (d_off, Pp[0], Pp[1], res))
            if res == "near":
                self.clear(Pp, ch)
                self.cleared.add(ch)
                return True
            if res == "direction":        # 已有第 2 条非共线方位 → 并入约束集，标准 AOA 交会
                self.ingest(ch, Pp, resp)
                return self._solve_aoa(ch, self.p["max_iters"])
        # 垂直偏移没拿到第 2 方位 → 退回其它侦察点（那些点本来就能看到该源）重测
        if self._grab_second_bearing(ch):
            return self._solve_aoa(ch, self.p["max_iters"])
        mid = 0.5 * (lo + hi)
        P = (V[0] + mid * ux, V[1] + mid * uy)
        r = self.clear(P, ch)
        if r.get("clear_result") == "success":
            self.cleared.add(ch)
            return True
        return False

    # ---------- 解析最小二乘交会（定向源主定位器） ----------
    def _aoa_estimate(self, ch):
        """多条方位射线的最小二乘交点：解 min Σ ((X-V_i)×u_i)²。
        忽略 ±1° 固定偏差时各射线本应交于源点，故该估计距真源仅 ~t·sin1°（≤15 m），
        可直接用于 clear。"""
        cons = self.constraints[ch]
        if len(cons) < 2:
            return None
        ATA = [[0.0, 0.0], [0.0, 0.0]]
        ATb = [0.0, 0.0]
        for (Vv, th) in cons:
            a = math.radians(th)
            ux, uy = math.cos(a), math.sin(a)
            px, py = -uy, ux                    # 垂直于射线
            ATA[0][0] += px * px
            ATA[0][1] += px * py
            ATA[1][1] += py * py
            ATb[0] += px * (px * Vv[0] + py * Vv[1])
            ATb[1] += py * (px * Vv[0] + py * Vv[1])
        ATA[1][0] = ATA[0][1]
        det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0]
        if abs(det) < 1e-9:
            return None
        x = (ATA[1][1] * ATb[0] - ATA[0][1] * ATb[1]) / det
        y = (ATA[0][0] * ATb[1] - ATA[0][1] * ATb[0]) / det
        return (x, y)

    def solve_directional(self, ch, max_iters=None):
        """仅 1~2 条远方位时的定向源定位：
           1) 不足 2 条 → 垂向补一条非共线方位；
           2) 用 _aoa_estimate 求源点估计（距真源 ≤15 m）→ 直接 clear；
           3) 没清掉则在估计点再测一条方位，区域进一步夹小，迭代收敛。
        """
        iters = max_iters or self.p["max_iters"]
        import os as _os
        dbg = _os.environ.get("P4DEBUG") == "1"
        if len(self.constraints[ch]) < 2:
            if not self._ensure_two_bearings(ch):
                return False
        for _ in range(iters):
            if self.out_of_time():
                return False
            S_est = self._aoa_estimate(ch)
            if S_est is None:
                if dbg:
                    print("  [aoa] ch=%d estimate None (退化)" % ch)
                return False
            # 估计点出界 → 退回两测点连线中点（必在覆盖半球内）
            if math.hypot(S_est[0], S_est[1]) > TARGET_R + 30:
                V1 = self.constraints[ch][0][0]
                V2 = self.constraints[ch][-1][0]
                S_est = (0.5 * (V1[0] + V2[0]), 0.5 * (V1[1] + V2[1]))
            if dbg:
                print("  [aoa] ch=%d S_est=(%.0f,%.0f)" % (ch, S_est[0], S_est[1]))
            r = self.clear(S_est, ch)
            if r.get("clear_result") == "success":
                self.cleared.add(ch)
                return True
            # 没清掉（偏几米~十几米）→ 在估计点测一条新方位，区域进一步夹小
            if self.ingest(ch, S_est, self.measure(S_est, ch)):
                return True
            # 估计点无信号（恰在盲侧/偏太多）→ 在两测点连线上安全点补测
            V1 = self.constraints[ch][0][0]
            V2 = self.constraints[ch][-1][0]
            Smid = (0.5 * (V1[0] + V2[0]), 0.5 * (V1[1] + V2[1]))
            if self.ingest(ch, Smid, self.measure(Smid, ch)):
                return True
        return False

    def _ensure_two_bearings(self, ch):
        """垂向补一条非共线方位：用首条方位的射线边界信息，在覆盖半球内侧做垂直偏移
        取点测向。双向小→中多档，兼顾"半球朝外张开"的边界发射型源。"""
        V, theta = self.constraints[ch][0]
        a = math.radians(theta)
        ux, uy = math.cos(a), math.sin(a)
        px, py = -uy, ux
        VV = V[0] * V[0] + V[1] * V[1]
        bdot = V[0] * ux + V[1] * uy
        disc = bdot * bdot - (VV - TARGET_R * TARGET_R)
        t_reach = (-bdot + math.sqrt(disc)) if disc >= 0 else 1500.0
        t_reach = min(t_reach, 1500.0)
        # 先沿射线找信号→无信号边界，取内侧基点
        last_cov = 0.0
        t = 0.0
        step = self.p["ray_step"]
        while t < t_reach - 1e-6:
            nt = min(t + step, t_reach)
            res = self.measure((V[0] + nt * ux, V[1] + nt * uy), ch).get("measure_result")
            if res == "near":
                self.clear((V[0] + nt * ux, V[1] + nt * uy), ch)
                self.cleared.add(ch)
                return True
            if res == "direction":
                if nt < t_reach - 1e-6:
                    last_cov = nt
                t = nt
                continue
            break
        base = max(60.0, 0.5 * (last_cov + t_reach))
        Pb = (V[0] + base * ux, V[1] + base * uy)
        for d_off in (80.0, -80.0, 160.0, -160.0, 40.0, -40.0,
                     250.0, -250.0, 120.0, -120.0):
            Pp = (Pb[0] + d_off * px, Pb[1] + d_off * py)
            if math.hypot(Pp[0], Pp[1]) > TARGET_R - 10:
                continue
            if self.ingest(ch, Pp, self.measure(Pp, ch)):
                return True
        return len(self.constraints[ch]) >= 2

    def _grab_second_bearing(self, ch):
        """射线法失败时兜底：从其它侦察点再取一条方位（不共线优先）。"""
        for V in self.vantage_points():
            if V in [P for P, _t in self.constraints[ch]]:
                continue
            if self.ingest(ch, V, self.measure(V, ch)):   # near → 直接清
                return True
            if len(self.constraints[ch]) >= 2:
                return True
        return len(self.constraints[ch]) >= 2

    # ---------- 侦察点 / 验证点 ----------
    def vantage_points(self):
        """
        侦察点 = 原点 + 内环(k_ring, r_ring) + 外环(k_ring_outer, r_ring_outer)。
        内环兜中央源；外环贴近 1800 边界，把"发射方向朝外、可见区大半在场地外"的
        定向源也兜住。每个点只扫"尚未发现任何信号"的频道（见 run()）。
        """
        pts = [(0.0, 0.0)]
        ri, ki = self.p["r_ring"], self.p["k_ring"]
        for k in range(ki):
            a = math.radians(self.p["ring_phase"] + 360.0 * k / max(1, ki))
            pts.append((ri * math.cos(a), ri * math.sin(a)))
        ro, ko = self.p["r_ring_outer"], self.p["k_ring_outer"]
        for k in range(ko):
            a = math.radians(self.p["ring_phase_outer"] + 360.0 * k / max(1, ko))
            pts.append((ro * math.cos(a), ro * math.sin(a)))
        return pts

    @staticmethod
    def _tour(points, start=None):
        """
        最近邻巡回：从 start（默认第一个点）出发，每步走到"还没去过的最近点"。
        覆盖点集合不变 → 发现能力不变；只是把"原点→内环→外环"的固定顺序改成更短路径，
        直接砍掉移动距离（虚拟时间 70% 的构成）。
        """
        if not points:
            return []
        rem = list(points)
        if start is None:
            cur = rem.pop(0)
        else:
            cur = start
            rem = [p for p in rem if p != start]
        order = [cur]
        while rem:
            rem.sort(key=lambda p: math.dist(cur, p))
            nxt = rem.pop(0)
            order.append(nxt)
            cur = nxt
        return order

    def verify_points(self):
        pts = []
        for k in range(self.p["k_verify"]):
            a = math.radians(self.p["verify_phase"] + 360.0 * k / max(1, self.p["k_verify"]))
            pts.append((self.p["r_verify"] * math.cos(a), self.p["r_verify"] * math.sin(a)))
        return pts

    def uncovered_target(self):
        """
        逐频道覆盖缺口驱动的验证点选择 —— 取代"固定验证环"/"全局 scan_points"。

        原理：对每个【尚未清除】的频道，若某处存在尚未被发现的干扰源，它到我们【扫过该频道】
        的所有位置的距离必然都超过它的有效接收半径 R（否则早就测到了）。R ≥ 1000（附件下界）。
        所以对每个未解频道，找出"离它自己已扫位置都 > 1000 m"的圆内点；取所有频道里
        最欠覆盖的那个位置返回，去那里补测（一次扫所有未解频道）即可保证不漏。

        ⚠ 必须用 self.channel_scan[ch]（逐频道真正扫过的位置），不能用全局 scan_points——
          否则某频道其实从没在边界附近扫过，却被别的频道的扫描"假装已覆盖"而漏源。
        """
        r_min = self.p["recv_min"]
        R = TARGET_R
        step = 150.0
        best_p, best_d = None, -1.0
        for ch in self.unanswered():
            pts = self.channel_scan.get(ch, [])
            if not pts:
                cand, d = (0.0, 0.0), R   # 从没扫过 → 强制先扫原点
            else:
                cp, cd = None, -1.0
                y = -R
                while y <= R:
                    x = -R
                    while x <= R:
                        if x * x + y * y <= R * R:
                            d = min(math.dist((x, y), p) for p in pts)
                            if d > cd:
                                cd, cp = d, (x, y)
                        x += step
                    y += step * 0.866
                cand, d = cp, cd
            if d > best_d:
                best_d, best_p = d, cand
        return best_p if best_d > r_min else None

    def maybe_opportunistic_scan(self):
        """
        顺手扫：既然逼近途中本来就要停下来测当前位置，就顺便把"还没解决的频道"也测一遍。
        **不额外产生任何移动**，只多花 5 s 检测 + 1 s 切换/频道。

        限流：与上一次顺手扫相距 > poll_min_dist 才做；每个频道最多被扫 max_polls 次。
        这样既能把覆盖快速铺开，又不会把时间浪费在重复确认上。
        """
        if not self.p["opportunistic"]:
            return
        un = [c for c in self.unanswered() if self.poll_count.get(c, 0) < self.p["max_polls"]]
        if not un:
            return
        if self.last_poll is not None and math.dist(self.pos, self.last_poll) < self.p["poll_min_dist"]:
            return
        self._scan(self.pos, un)
        self.last_poll = self.pos
        self.stats["opp_scans"] = self.stats.get("opp_scans", 0) + 1
        for c in un:
            self.poll_count[c] = self.poll_count.get(c, 0) + 1

    def unanswered(self):
        """还没有任何方位约束、也没清除/判死的频道 —— 只扫这些，能省大量时间。"""
        return [c for c in range(1, 21)
                if c not in self.cleared and c not in self.dead and not self.constraints[c]]

    def _scan(self, V, channels):
        """在 V 上按频道升序扫一遍未解频道。外层必须是"点"、内层才是"频道"。

        同时把 V 记入 scan_points —— 这是"对该频道构成覆盖"的位置集合，
        覆盖缺口的判断只能用这个集合（见 uncovered_target 的说明）。
        """
        channels = [c for c in channels if c not in self.cleared and c not in self.dead]
        if not channels:
            return True
        for ch in channels:
            if self.out_of_time():
                return False
            self.ingest(ch, V, self.measure(V, ch))
            self.channel_scan[ch].append((V[0], V[1]))
        self.scan_points.append((V[0], V[1]))
        return True

    # ---------- 主流程 ----------
    def run(self):
        e = self.call("/enter", wait_s=self.p.get("enter_wait", 0.0))
        if e.get("accepted") is not True:
            return {"error": "enter 未被接受", "enter_response": e}
        self.t0 = time.time()
        self.remaining = float(e.get("remaining_real_duration_s", 1200))

        # L1 侦察：原点 + 内外侦察环。每个点扫"尚无约束"的频道；并对"只有 1 条方位
        # 约束"的频道继续补测，直到凑够第 2 条方位（定向源 AOA 必需）。这样定向源大多
        # 在侦察阶段就拿到 ≥2 条张角足够的方位，直接走标准 AOA，不必依赖射线法兜底。
        for V in self._tour(self.vantage_points(), (0.0, 0.0)):
            if self.out_of_time():
                break
            batch = self.unanswered() + [c for c in self.candidates
                    if len(self.constraints[c]) == 1 and c not in self.cleared]
            if not batch:
                break                      # 剩余频道要么已清/判死，要么已有 ≥2 方位
            if not self._scan(V, batch):
                break

        # L2+L3 定位清除，交替做 L4 发现/验证
        for _round in range(self.p["max_rounds"]):
            while True:
                if self.out_of_time() or self.vtime > self.p["max_vtime"]:
                    break
                pending = [c for c in self.candidates
                           if c not in self.cleared and c not in self.gaveup]
                if not pending:
                    break
                # 贪心最近邻排序：每次挑"估计位置离当前位置最近"的频道去做，
                # 避免按频道号乱穿（这一项对移动耗时影响最大）。
                def _est(ch):
                    pl = region_of(self.constraints[ch], eps=self.p["eps"])
                    return poly_centroid(pl) if len(pl) >= 3 else self.pos
                ch = min(pending, key=lambda c: math.dist(self.pos, _est(c)))
                ok = self.solve_channel(ch)
                if not ok:
                    # 必须记失败次数并最终放弃，否则这一轮会永远重试同一个频道（死循环）
                    self.attempts[ch] = self.attempts.get(ch, 0) + 1
                    if self.attempts[ch] >= self.p["max_attempts"]:
                        self.gaveup.add(ch)
            if self.out_of_time() or self.vtime > self.p["max_vtime"]:
                break
            # L4 验证：逐频道覆盖缺口补缺，直到所有未解频道都覆盖到 recv_min（或超轮次/超时），
            # 兜底漏清。注：纯缺口距离判据对"无侦察点落入的边界窄缝源"识别力有限，
            # 故保证 1.000 仍需足够密的外环（单圈 24 点，见 p4_stats 对比）；本兜底用于收尾。
            if not self.unanswered():
                break
            for _k in range(self.p["max_verify_iters"]):
                if self.out_of_time() or self.vtime > self.p["max_vtime"]:
                    break
                V = self.uncovered_target()
                if V is None:
                    break                              # 所有未解频道都已覆盖到 recv_min → 收敛
                self.stats["verify_trips"] = self.stats.get("verify_trips", 0) + 1
                self._scan(V, self.unanswered())
            # 跳出后：未解频道都已覆盖到 recv_min（或资源耗尽）→ 判为无源
            for c in self.unanswered():
                self.dead.add(c)
            break

        self.call("/exit")
        return self.report()

    def report(self):
        return {"cleared": len(self.cleared), "vtime": round(self.vtime, 1),
                "stats": dict(self.stats), "constraints_used":
                    {c: len(v) for c, v in self.constraints.items() if v}}


# ----------------------------------------------------------------------
# 演练驱动
# ----------------------------------------------------------------------
def run_episode(seed, params=None, n_sources=None, directional=False,
                dist="uniform_disk", verbose=False):
    arena = MockArena(seed=seed, n_sources=n_sources, directional=directional, dist=dist)
    r = Robot(arena, params)
    robot_rep = r.run()
    t = arena.truth()
    t["robot"] = robot_rep
    if verbose:
        print("    seed=%-5s 清除 %2d/%2d  虚拟时间 %8.1f s  平均 %7.1f s  "
              "(measure %d, clear %d, 移动 %.0f m)"
              % (seed, t["n_cleared"], t["n_total"], t["total_virtual_time_s"],
                 t["avg_time_per_cleared"] or float("nan"),
                 robot_rep["stats"]["measures"], robot_rep["stats"]["clears"],
                 robot_rep["stats"]["moves"]))
    return t


def summarize(results, title=""):
    n = len(results)
    ratios = [r["ratio"] for r in results]
    avgs = [r["avg_time_per_cleared"] for r in results if r["avg_time_per_cleared"]]
    perfect = sum(1 for x in ratios if x >= 0.999999)
    print("  %-28s 局数=%2d  平均清除比例=%.4f  完美局=%2d/%2d  平均定位清除时间=%.1f s"
          % (title, n, sum(ratios) / n, perfect, n,
             (sum(avgs) / len(avgs)) if avgs else float("nan")))
    return {"ratio": sum(ratios) / n, "avg": (sum(avgs) / len(avgs)) if avgs else None,
            "perfect": perfect, "n": n}


def cmd_mock(args):
    episodes = args.episodes
    s0 = getattr(args, "seed0", 1000)
    direc = getattr(args, "directional", False)
    print("离线演练（%s）：共 %d 局，seed 从 %d 起"
          % ("问题 4：含定向源" if direc else "问题 3：全向源", episodes, s0))
    results = []
    for i in range(episodes):
        r = run_episode(s0 + i, verbose=True, directional=direc,
                        dist=getattr(args, "dist", "uniform_disk"))
        results.append(r)
    print()
    summarize(results, "总体")
    bad = [r for r in results if r["ratio"] < 0.999999]
    if bad:
        print("  未清干净的局：")
        for r in bad[:6]:
            miss = r["n_total"] - r["n_cleared"]
            print("     漏 %d 个 | 清除 %d/%d | 虚拟时间 %.1f s"
                  % (miss, r["n_cleared"], r["n_total"], r["total_virtual_time_s"]))
    else:
        print("  所有局都清干净了（ratio = 1）。")


def cmd_sweep(args):
    print("参数扫描（每格 %d 局）" % args.episodes)
    grid = []
    for ang in (8.0, 12.0, 15.0, 20.0):
        for k_ring in (2, 3, 4):
            grid.append(("ang=%-4g k_ring=%d" % (ang, k_ring),
                         {"step_angle": ang, "tol": 40.0, "k_ring": k_ring}))
    rows = []
    for name, params in grid:
        results = [run_episode(2000 + i, params=params) for i in range(args.episodes)]
        rows.append((name, summarize(results, name)))
    print()
    print("按【平均定位清除时间】排序（越小越好）：")
    for name, s in sorted(rows, key=lambda z: (z[1]["avg"] or 1e18)):
        print("   %-30s ratio=%.4f  完美=%2d/%2d  平均时间=%7.1f s"
              % (name, s["ratio"], s["perfect"], s["n"], s["avg"] or float("nan")))


def _save_log(arena, tag="live"):
    """附件要求：机器狗程序应自行记录指令序列与响应信息（模拟器不提供这一功能）。"""
    import json
    import os
    stamp = time.strftime("%Y%m%d-%H%M%S")
    os.makedirs("logs", exist_ok=True)
    path = os.path.join("logs", "%s-%s.jsonl" % (tag, stamp))
    with open(path, "w", encoding="utf-8") as f:
        for rec in getattr(arena, "log", []):
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    return path


def cmd_live(args):
    arena = HttpArena(args.url, args.team)
    params = {"robot_id": args.team, "enter_wait": float(args.wait_enter)}
    if args.tol:
        params["tol"] = args.tol
    if args.k_ring is not None:
        params["k_ring"] = args.k_ring
    r = Robot(arena, params)
    print("连接真实模拟器 %s （robot_id = %s）" % (args.url, args.team))
    print("策略参数：tol=%s, k_ring=%s" % (params.get("tol", "默认40"),
                                          params.get("k_ring", "默认3")))
    print("正在等待接口开放（最多 %s 秒）——请先在模拟器里点开始并等完 5 秒倒计时。" % args.wait_enter)
    try:
        rep = r.run()
    except RuntimeError as e:
        print("失败：%s" % e)
        print("排查：1) 模拟器是否在运行且倒计时已结束；2) 端口是否被占用（可在模拟器设置里改）；")
        print("      3) 本机时间与服务器时间是否相差超过 60 秒；4) 是否已用同一队号在别的设备登录。")
        return
    lp = _save_log(arena)
    if "cleared" not in rep:
        print("未能开始测试：%s" % rep.get("error", "未知原因"))
        print("模拟器 /enter 的原始响应：", rep.get("enter_response"))
        print("已保存本机日志：", lp)
        return
    print("结束：清除 %d 个，虚拟时间 %.1f s" % (rep["cleared"], rep["vtime"]))
    print("平均定位清除时间：%.1f s   （= 虚拟时间 / 清除个数）"
          % (rep["vtime"] / rep["cleared"] if rep["cleared"] else float("nan")))
    print("动作统计：", rep["stats"])
    print("本机指令日志已保存：%s" % lp)
    print("提示：模拟器界面会显示本局干扰源总数与测试案例编码；"
          "正式测试请按表 1 记录，并从模拟器【日志列表】导出加密日志（不要改文件名）。")


def cmd_dist(args):
    """分布鲁棒性压力测试：同一套策略在不同空间分布下是否还能清干净。"""
    dists = ["uniform_disk", "uniform_radius", "boundary", "center", "clustered"]
    print("分布鲁棒性测试（每格 %d 局）" % args.episodes)
    rows = []
    for d in dists:
        results = [run_episode(args.seed0 + i, dist=d) for i in range(args.episodes)]
        s = summarize(results, d)
        rows.append((d, s))
    print()
    print("按【平均定位清除时间】排序（越小越好）：")
    for d, s in sorted(rows, key=lambda z: (z[1]["avg"] or 1e18)):
        mark = "OK " if s["perfect"] == s["n"] else "!! "
        print("   %s%-16s ratio=%.4f  完美=%2d/%2d  平均时间=%7.1f s"
              % (mark, d, s["ratio"], s["perfect"], s["n"], s["avg"] or float("nan")))


def cmd_analyze(args):
    """
    空间分布推断：题目没有给出干扰源的空间分布，但演练测试里一旦 ratio = 1，
    我们就清除了全部源 —— 此时"我们记录到的位置集合"就是该局的【完整样本】，
    没有选择性偏差，可以直接拿来做空间统计检验。
    """
    R = TARGET_R
    counts = {}
    cases = []
    for i in range(args.episodes):
        arena = MockArena(seed=args.seed0 + i, dist=args.dist)
        case = [(s.x, s.y) for _c, s in arena.sources.items()]
        counts[len(case)] = counts.get(len(case), 0) + 1
        cases.append(case)
    pts = [p for case in cases for p in case]
    tot = len(pts)
    print("分布假设 = %s；模拟 %d 局，共 %d 个源" % (args.dist, args.episodes, tot))

    print("\n1) 每局干扰源个数分布（题目说 10~16）：")
    for k in sorted(counts):
        print("     N=%2d : %3d 局  (%.1f%%)" % (k, counts[k], 100.0 * counts[k] / len(cases)))

    print("\n2) 径向密度：归一化半径 ρ=r/R 的占比（观测 vs 圆内面积均匀理论 b²−a²）")
    nb = 10
    obs = [0] * nb
    for (x, y) in pts:
        rho = math.hypot(x, y) / R
        obs[min(nb - 1, int(rho * nb))] += 1
    print("     %-14s %10s %10s" % ("ρ 区间", "观测占比", "理论占比"))
    chi2 = 0.0
    for k in range(nb):
        a, b = k / nb, (k + 1) / nb
        exp_frac = b * b - a * a
        o = obs[k] / tot
        if exp_frac > 1e-9:
            chi2 += (obs[k] - exp_frac * tot) ** 2 / (exp_frac * tot)
        print("     [%.1f, %.1f)      %10.4f %10.4f" % (a, b, o, exp_frac))
    print("     卡方统计量(9 自由度) = %.2f（临界值 16.92@5%%，27.88@0.1%%）" % chi2)

    print("\n3) 最近邻距离：判断「随机 / 聚集 / 规则」")
    nn = []
    for case in cases:
        m = len(case)
        for i in range(m):
            nn.append(min(math.dist(case[i], case[j]) for j in range(m) if j != i))
    mean_nn = sum(nn) / len(nn)
    nbar = tot / len(cases)
    lam = nbar / (math.pi * R * R)
    exp_nn = 0.5 / math.sqrt(lam)
    print("     观测平均最近邻距离 = %.1f m" % mean_nn)
    print("     泊松（圆内均匀）理论值 0.5/√λ = %.1f m" % exp_nn)
    print("     Clark-Evans 指数 R = %.3f   （>1 偏规则，≈1 随机，<1 聚集）" % (mean_nn / exp_nn))
    print("\n   注意：这只是流程演示（数据由本地模拟器按设定分布生成）。")
    print("         接真实模拟器时，把每局清掉的源坐标记下来，跑同样的三步即可反推其真实分布。")
    print("         技巧：只有 ratio=1 的局才无选择性偏差，务必只统计这类局。")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("mock")
    m.add_argument("--episodes", type=int, default=20)
    m.add_argument("--seed0", type=int, default=1000)
    m.add_argument("--dist", default="uniform_disk",
                   choices=["uniform_disk", "uniform_radius", "boundary", "center", "clustered"])
    m.add_argument("--directional", action="store_true",
                   help="生成问题 4 的混合案例（约一半为定向源）")
    m.set_defaults(func=cmd_mock)

    d = sub.add_parser("dist")
    d.add_argument("--episodes", type=int, default=6)
    d.add_argument("--seed0", type=int, default=40000)
    d.set_defaults(func=cmd_dist)

    a = sub.add_parser("analyze")
    a.add_argument("--episodes", type=int, default=200)
    a.add_argument("--seed0", type=int, default=90000)
    a.add_argument("--dist", default="uniform_disk",
                   choices=["uniform_disk", "uniform_radius", "boundary", "center", "clustered"])
    a.set_defaults(func=cmd_analyze)

    s = sub.add_parser("sweep"); s.add_argument("--episodes", type=int, default=8)
    s.set_defaults(func=cmd_sweep)

    l = sub.add_parser("live")
    l.add_argument("--url", default="http://127.0.0.1:2026")
    l.add_argument("--team", required=True, help="参赛队号，必须与模拟器登录的队号一致")
    l.add_argument("--wait-enter", type=float, default=60.0,
                   help="等待接口开放的最长秒数（模拟器有 5 秒倒计时，建议 ≥30）")
    l.add_argument("--tol", type=float, default=None, help="覆盖收敛阈值（默认 40 m）")
    l.add_argument("--k-ring", type=int, default=None, help="覆盖侦察环点数（默认 3）")
    l.set_defaults(func=cmd_live)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
