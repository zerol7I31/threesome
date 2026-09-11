# -*- coding: utf-8 -*-
"""对比不同侦察环配置在定向案例上的清除比例与虚拟耗时。"""
import sys
from p3_robot import run_episode

configs = {
    "dense(内6+外24@1780)": dict(k_ring=6, r_ring=1000.0, k_ring_outer=24, r_ring_outer=1780.0),
    "lean(内8+外12@1780)":  dict(k_ring=8, r_ring=1200.0, k_ring_outer=12, r_ring_outer=1780.0),
    "lean2(内6+外12@1700)": dict(k_ring=6, r_ring=1000.0, k_ring_outer=12, r_ring_outer=1700.0),
}
seeds = list(range(70000, 70030))
for name, params in configs.items():
    tot = clr = 0
    vt = 0.0
    ms = 0
    for s in seeds:
        t = run_episode(s, params=params, directional=True)
        tot += t["n_total"]; clr += t["n_cleared"]
        vt += t["total_virtual_time_s"]; ms += t["robot"]["stats"].get("measures", 0)
    print("%-22s 比例=%.4f (%d/%d)  平均虚拟时间=%.0f s  平均measures=%.0f"
          % (name, clr/tot, clr, tot, vt/len(seeds), ms/len(seeds)))
