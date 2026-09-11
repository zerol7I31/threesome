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
    "k_ring": 3,             # 侦察环上的点数（另有原点共 k_ring+1 个侦察点）
    "r_ring": 1100.0,        # 侦察环半径
    "ring_phase": 30.0,      # 侦察环起始方位角（度）
    "k_verify": 3,           # 验证环点数
    "r_verify": 1300.0,
    "verify_phase": 90.0,
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
        self.pos = (0.0, 0.0)
        self.vtime = 0.0
        self.t0 = None
        self._side = 1.0
        self.stats = {"measures": 0, "clears": 0, "misses": 0, "moves": 0.0}

    # ---------- 通信 ----------
    def call(self, path, x=None, y=None, ch=None):
        self._rid += 1
        payload = {"arena_id": "default", "robot_id": self.p["robot_id"],
                   "request_id": "%s-%d" % (path[1:], self._rid)}
        if x is not None:
            payload["position"] = {"x": float(x), "y": float(y)}
        if ch is not None:
            payload["channel"] = int(ch)
        resp = self.a.request(path, payload)
        if resp.get("accepted") is True:
            if "virtual_time_s" in resp:
                self.vtime = max(self.vtime, float(resp["virtual_time_s"]))
            if x is not None:
                self.stats["moves"] += math.dist(self.pos, (float(x), float(y)))
                self.pos = (float(x), float(y))
        return resp

    def measure(self, P, ch):
        r = self.call("/measure", P[0], P[1], ch)
        self.stats["measures"] += 1
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
        # 先"顺手"在当前位置测一次：不产生移动耗时，只花 5 s；若恰好没信号就白花 5 s。
        # 但同一位置重复测同一频道结果完全一样（固定偏差），所以只在离已有测点足够远时才测。
        if self.constraints[ch] and all(math.dist(self.pos, P) > 50.0
                                        for P, _t in self.constraints[ch]):
            if self.ingest(ch, self.pos, self.measure(self.pos, ch)):
                return True
        for _ in range(iters):
            if self.out_of_time():
                return False
            poly = region_of(self.constraints[ch], eps=self.p["eps"])
            if len(poly) < 3:
                return False                     # 理论上不会：真实源必在所有楔形内
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
                resp = self.measure(P, ch)
                if self.ingest(ch, P, resp):
                    return True
                continue
            dist = math.dist(self.constraints[ch][-1][0], G)
            P = self.pick_next(ch, G, max(dist, self.p["tol"]))
            resp = self.measure(P, ch)
            if self.ingest(ch, P, resp):
                return True
            if resp.get("measure_result") == "no_signal":
                # 走过头/超出接收半径 → 朝估计点回撤一步再测
                back = (self.pos[0] + 0.35 * (G[0] - self.pos[0]),
                        self.pos[1] + 0.35 * (G[1] - self.pos[1]))
                resp2 = self.measure(back, ch)
                if self.ingest(ch, back, resp2):
                    return True
        return False

    # ---------- 侦察点 / 验证点 ----------
    def vantage_points(self):
        pts = [(0.0, 0.0)]
        for k in range(self.p["k_ring"]):
            a = math.radians(self.p["ring_phase"] + 360.0 * k / max(1, self.p["k_ring"]))
            pts.append((self.p["r_ring"] * math.cos(a), self.p["r_ring"] * math.sin(a)))
        return pts

    def verify_points(self):
        pts = []
        for k in range(self.p["k_verify"]):
            a = math.radians(self.p["verify_phase"] + 360.0 * k / max(1, self.p["k_verify"]))
            pts.append((self.p["r_verify"] * math.cos(a), self.p["r_verify"] * math.sin(a)))
        return pts

    def unanswered(self):
        """还没有任何方位约束、也没清除/判死的频道 —— 只扫这些，能省大量时间。"""
        return [c for c in range(1, 21)
                if c not in self.cleared and c not in self.dead and not self.constraints[c]]

    def _scan(self, V, channels):
        """在侦察点 V 上按频道升序测一遍（减少切换耗时）。注意：外层必须是"点"、内层才是"频道"。"""
        for ch in channels:
            if ch in self.cleared or ch in self.dead:
                continue
            if self.out_of_time():
                return False
            self.ingest(ch, V, self.measure(V, ch))
        return True

    # ---------- 主流程 ----------
    def run(self):
        e = self.call("/enter")
        if e.get("accepted") is not True:
            return {"error": "enter 未被接受", "enter_response": e}
        self.t0 = time.time()
        self.remaining = float(e.get("remaining_real_duration_s", 1200))

        # L1 侦察：原点 + 侦察环；每个点只扫"尚未发现任何信号"的频道
        for V in self.vantage_points():
            if not self._scan(V, self.unanswered()):
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
            before = set(self.candidates)
            for V in self.verify_points():
                if not self._scan(V, self.unanswered()):
                    break
            if set(self.candidates) == before:          # 没有新发现 → 收敛
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


def cmd_live(args):
    arena = HttpArena(args.url, args.team)
    params = {"robot_id": args.team}
    r = Robot(arena, params)
    print("连接真实模拟器 %s （robot_id = %s）..." % (args.url, args.team))
    print("提示：请先在模拟器界面点开始，等 5 秒倒计时结束、接口就绪后再运行本程序。")
    try:
        rep = r.run()
    except RuntimeError as e:
        print("失败：%s" % e)
        print("排查：1) 模拟器是否在运行且倒计时已结束；2) 端口是否被占用（可在模拟器设置里改）；")
        print("      3) 本机时间与服务器时间是否相差超过 60 秒；4) 是否已用同一队号在别的设备登录。")
        return
    if "cleared" not in rep:
        print("未能开始测试：%s" % rep.get("error", "未知原因"))
        print("模拟器 /enter 的原始响应：", rep.get("enter_response"))
        return
    print("结束：清除 %d 个，虚拟时间 %.1f s" % (rep["cleared"], rep["vtime"]))
    print("动作统计：", rep["stats"])
    print("提示：模拟器界面会显示本局干扰源总数与测试案例编码；正式测试请按表 1 记录并导出日志。")


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
    l.add_argument("--team", required=True)
    l.set_defaults(func=cmd_live)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
