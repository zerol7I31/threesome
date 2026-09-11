# -*- coding: utf-8 -*-
"""
问题4 优化前后对比：跑 50 局定向案例，统计清除比例 + 检测/移动/虚拟时间。
用法：python p4_stats.py [tag]
  tag=base  -> 当前代码（基线）
  tag=opt   -> 跑优化版（若已改）
直接 python p4_stats.py 即基线。
"""
import sys
import p3_robot as R

SEEDS = list(range(70000, 70050))  # 50 局

def main():
    override = {}
    if len(sys.argv) > 1:
        override["k_ring_outer"] = int(sys.argv[1])
    if len(sys.argv) > 2:
        override["k_ring"] = int(sys.argv[2])
    tag = "outer%s" % override.get("k_ring_outer", "def") if override else "def"
    total_cleared = total_src = 0
    sum_measures = sum_moves = sum_vtime = 0
    perfect = 0
    rows = []
    for s in SEEDS:
        t = R.run_episode(s, directional=True, verbose=False, params=(override or None))
        rep = t["robot"]
        st = rep["stats"]
        cleared = t["n_cleared"]; nt = t["n_total"]
        total_cleared += cleared; total_src += nt
        sum_measures += st.get("measures", 0)
        sum_moves += st.get("moves", 0)
        sum_vtime += rep.get("vtime", 0)
        if t["ratio"] >= 0.999999:
            perfect += 1
        rows.append((s, cleared, nt, st.get("measures", 0), st.get("moves", 0),
                     rep.get("vtime", 0), st.get("clears", 0)))
    ratio = total_cleared / total_src
    print("=== 问题4 50 局定向案例汇总 ===")
    print("清除比例       : %.4f  (%d/%d)" % (ratio, total_cleared, total_src))
    print("完美局(=1.0)   : %d/%d" % (perfect, len(SEEDS)))
    print("总检测次数     : %d" % sum_measures)
    print("总移动距离(m)  : %d  (%.1f km)" % (sum_moves, sum_moves / 1000.0))
    print("总虚拟时间(s)  : %.1f  (平均 %.1f s/局)" % (sum_vtime, sum_vtime / len(SEEDS)))
    print("平均定位清除时间: %.1f s/个" % (sum_vtime / total_cleared if total_cleared else float("nan")))
    # 漏清局
    miss = [(s, c, n) for (s, c, n, *_ ) in rows if c < n]
    if miss:
        print("漏清局：", miss)
    # 每局明细落盘
    with open("p4_stats_out.txt", "w", encoding="utf-8") as f:
        for (s, c, n, m, mv, vt, cl) in rows:
            f.write("seed=%-6d cleared=%2d/%2d measures=%3d moves=%5d vtime=%8.1f\n"
                    % (s, c, n, m, mv, vt))
    print("明细 -> p4_stats_out.txt")

if __name__ == "__main__":
    main()
