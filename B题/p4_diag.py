# -*- coding: utf-8 -*-
"""
问题 4 诊断器：复现 directional 案例的漏清，并区分两类失败：
  (A) undiscovered  —— 侦察阶段（含后续补测）从未拿到该频道的任何 direction/near
  (B) localization   —— 拿到过方向约束，但最终没清掉（gaveup / 未收敛）
同时用几何验证"侦察点是否落在源的可见半球内"，定位问题根因。
"""
import math
import sys

import p3_arena
from p3_robot import Robot, DEFAULT_PARAMS, run_episode


def diag_one(seed, params=None, dist="uniform_disk"):
    arena = p3_arena.MockArena(seed=seed, directional=True, dist=dist)
    r = Robot(arena, params)
    rep = r.run()

    sources = arena.sources
    out = []
    for c, s in sources.items():
        discovered = (c in r.candidates) or (c in r.cleared)
        cleared = c in r.cleared
        ncon = len(r.constraints[c])
        status = "OK" if cleared else ("undiscovered" if not discovered else "localize_fail")
        if status == "localize_fail" and c in r.gaveup:
            status = "gaveup"
        # 几何：侦察环是否落在可见半球+半径内
        vpts = r.vantage_points()
        dmax = s.radius
        vis = []
        for V in vpts:
            d = math.dist(V, (s.x, s.y))
            if d > dmax:
                vis.append("OOR")          # 超接收半径
                continue
            if s.direction is None:
                vis.append("SEE")          # 全向源：在范围内即可见
                continue
            # bearing(src -> V)
            b = math.degrees(math.atan2(V[1] - s.y, V[0] - s.x))
            diff = abs((b - s.direction + 180.0) % 360.0 - 180.0)
            vis.append("SEE" if diff <= 90.0 else "BLIND")
        out.append({
            "ch": c, "dir": None if s.direction is None else round(s.direction, 0),
            "pos": (round(s.x), round(s.y)), "R": round(s.radius),
            "discovered": discovered, "cleared": cleared, "ncon": ncon,
            "status": status, "vantage": vis,
        })
    return out, rep, arena


def main():
    seeds = [int(x) for x in sys.argv[1:]] or [70000, 70001, 70002]
    tot = 0
    cleared = 0
    und = 0
    loc = 0
    for seed in seeds:
        rows, rep, arena = diag_one(seed)
        tot += len(rows)
        for x in rows:
            if x["cleared"]:
                cleared += 1
            elif x["status"] == "undiscovered":
                und += 1
            else:
                loc += 1
        print("=" * 78)
        print("seed=%d  清除 %d/%d  虚拟时间 %.1f s"
              % (seed, rep["cleared"], len(rows), rep["vtime"]))
        print("  %3s %5s  %-14s %5s %-13s %-9s %s"
              % ("ch", "dir°", "pos", "R", "status", "ncon", "vantage(OOR/BLIND/SEE)"))
        for x in sorted(rows, key=lambda z: (z["status"] != "OK", z["ch"])):
            print("  %3d %5s  %-14s %5d %-13s %-9d %s"
                  % (x["ch"], ("-" if x["dir"] is None else x["dir"]),
                     str(x["pos"]), x["R"], x["status"], x["ncon"], x["vantage"]))
    print("=" * 78)
    print("汇总：总源 %d | 清除 %d | 漏(未发现) %d | 漏(定位失败) %d"
          % (tot, cleared, und, loc))
    if tot:
        print("清除比例 = %.3f" % (cleared / tot))


if __name__ == "__main__":
    main()
