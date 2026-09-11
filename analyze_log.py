# -*- coding: utf-8 -*-
"""
行为日志分析器 —— 输入机器狗自己记录的 logs/live-*.jsonl，输出行为报告与冗余诊断。

用法：
    python analyze_log.py logs/live-20260911-123327.jsonl
    python analyze_log.py                # 自动取 logs/ 下最新的一个
"""

import glob
import json
import math
import os
import sys

R_RECON = 1100.0     # 侦察环半径（与 p3_robot 默认参数一致）
R_VERIFY = 1300.0    # 验证环半径


def load(path):
    acts = []
    prev_t = 0.0
    prev_pos = (0.0, 0.0)
    cam = 1
    for ln in open(path, encoding="utf-8"):
        r = json.loads(ln)
        pth, p, resp = r["path"], r["payload"], r["resp"]
        if resp.get("accepted") is not True:
            acts.append({"path": pth, "rejected": True, "req": p.get("request_id")})
            continue
        t = float(resp.get("virtual_time_s", 0.0))
        dt = max(0.0, t - prev_t)
        pos = p.get("position")
        pos = (float(pos["x"]), float(pos["y"])) if pos else None
        move = math.dist(prev_pos, pos) if pos else 0.0
        ch = p.get("channel")
        if pth == "/measure":
            sw = 0 if ch == cam else 1
            det = 5.0
            res = resp.get("measure_result")
        elif pth == "/clear":
            sw = 0
            det = 5.0 if resp.get("clear_result") == "success" else 3.0
            res = resp.get("clear_result")
        else:
            sw = det = 0
            res = resp.get("exit_reason")
        acts.append({"path": pth, "t": t, "dt": dt, "pos": pos, "ch": ch,
                     "move": move, "move_t": move / 5.0, "switch": sw, "detect": det,
                     "residual": dt - move / 5.0 - sw - det,
                     "result": res, "svd": resp.get("svd_deg")})
        if pth == "/measure":
            cam = ch
        if pos:
            prev_pos = pos
        prev_t = t
    return acts


def scan_blocks(acts):
    """把连续的、位置相同（<1m）的 /measure 合成一个"静止测站"。"""
    blocks = []
    cur = None
    for i, a in enumerate(acts):
        if a.get("rejected") or a["path"] != "/measure":
            if cur:
                blocks.append(cur); cur = None
            continue
        if cur and a["pos"] and math.dist(cur["pos"], a["pos"]) < 1.0:
            cur["items"].append((i, a))
        else:
            if cur:
                blocks.append(cur)
            cur = {"pos": a["pos"], "items": [(i, a)]}
    if cur:
        blocks.append(cur)
    for b in blocks:
        b["n"] = len(b["items"])
        b["t_first"] = b["items"][0][1]["t"]
        b["dt"] = sum(x[1]["dt"] for x in b["items"])
        b["move"] = b["items"][0][1]["move"]
        b["move_t"] = b["items"][0][1]["move_t"]
        b["switch"] = sum(x[1]["switch"] for x in b["items"])
        b["detect"] = sum(x[1]["detect"] for x in b["items"])
        b["dir"] = sum(1 for x in b["items"] if x[1]["result"] == "direction")
        b["no"] = sum(1 for x in b["items"] if x[1]["result"] == "no_signal")
        b["near"] = sum(1 for x in b["items"] if x[1]["result"] == "near")
        b["chs"] = [x[1]["ch"] for x in b["items"]]
        b["r"] = math.hypot(*b["pos"]) if b["pos"] else 0.0
    return blocks


def tour_len_open(pts, iters=400):
    """nearest-neighbour + 2-opt（开放路径，起点固定为 pts[0]）→ 良好上界。"""
    pts = list(pts)
    n = len(pts)
    if n < 3:
        return sum(math.dist(pts[i], pts[i + 1]) for i in range(n - 1))
    unv = list(range(1, n))
    order = [0]
    while unv:
        k = min(unv, key=lambda j: math.dist(pts[order[-1]], pts[j]))
        order.append(k); unv.remove(k)
    def L(o):
        return sum(math.dist(pts[o[i]], pts[o[i + 1]]) for i in range(len(o) - 1))
    improved = True
    it = 0
    while improved and it < iters:
        improved = False; it += 1
        for i in range(1, len(order) - 1):
            for j in range(i + 1, len(order)):
                o = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                if L(o) < L(order) - 1e-9:
                    order = o; improved = True
    return L(order)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob("logs/*.jsonl"))[-1]
    acts = load(path)
    ok = [a for a in acts if not a.get("rejected")]
    measures = [a for a in ok if a["path"] == "/measure"]
    clears = [a for a in ok if a["path"] == "/clear"]
    hits = [c for c in clears if c["result"] == "success"]
    T = ok[-1]["t"] if ok else 0.0

    print("=" * 78)
    print("行为报告   %s" % path)
    print("=" * 78)
    Mt = sum(a["move_t"] for a in ok)
    Dt = sum(a["detect"] for a in measures)      # /measure：每次 5 s
    Ct = sum(a["detect"] for a in clears)        # /clear：命中 5 s、未命中 3 s
    St = sum(a["switch"] for a in ok)
    moves = sum(a["move"] for a in ok)
    print("虚拟总时间 %.1f s   移动 %.0f m   清除 %d 个   平均 %.1f s/个"
          % (T, moves, len(hits), T / len(hits) if hits else float("nan")))
    print()
    print("【时间分解】")
    for nm, v in (("移动", Mt), ("检测(/measure)", Dt),
                  ("清除动作(/clear)", Ct), ("切换频道", St)):
        print("   %-16s %8.1f s   %5.1f%%" % (nm, v, 100 * v / T))
    print("   %-16s %8.1f s" % ("合计", Mt + Dt + Ct + St))
    print()
    print("【指令与结果】")
    print("   /measure %d 次：direction %d 次(%.0f%%)、no_signal %d 次(%.0f%%)、near %d 次"
          % (len(measures),
             sum(1 for a in measures if a["result"] == "direction"),
             100.0 * sum(1 for a in measures if a["result"] == "direction") / max(1, len(measures)),
             sum(1 for a in measures if a["result"] == "no_signal"),
             100.0 * sum(1 for a in measures if a["result"] == "no_signal") / max(1, len(measures)),
             sum(1 for a in measures if a["result"] == "near")))
    print("   /clear   %d 次：success %d、no_target_in_range %d"
          % (len(clears), len(hits), len(clears) - len(hits)))

    # ---- 静止测站 ----
    blocks = scan_blocks(acts)
    recon = [b for b in blocks if b["r"] < 5 or abs(b["r"] - R_RECON) < 8]
    verify = [b for b in blocks if abs(b["r"] - R_VERIFY) < 8 and b not in recon]
    oper = [b for b in blocks if b not in recon and b not in verify]
    print()
    print("【阶段划分】按「静止测站」聚合（同一位置连续检测算一站）")
    print("   共 %d 个静止测站" % len(blocks))
    for nm, grp in (("侦察站（原点/侦察环）", recon), ("验证站（验证环）", verify),
                    ("作业站（逼近过程中）", oper)):
        if not grp:
            continue
        print("   %-22s %2d 站  %4d 次检测  移动 %7.1f m (%.0f s)  检测 %6.1f s  无信号 %d"
              % (nm, len(grp), sum(b["n"] for b in grp), sum(b["move"] for b in grp),
                 sum(b["move_t"] for b in grp), sum(b["detect"] for b in grp),
                 sum(b["no"] for b in grp)))

    # ---- 逐频道 ----
    print()
    print("【逐频道代价】")
    print("   频道 | 检测次数 | 无信号 | 移动(m) | 移动(s) | 该频道检测(s) | 结果")
    for ch in sorted(set(a["ch"] for a in measures)):
        ms = [a for a in measures if a["ch"] == ch]
        mv = sum(a["move"] for a in ms)
        got = ch in set(int(c["ch"]) for c in hits)
        print("   %4d | %8d | %6d | %7.0f | %7.1f | %13.1f | %s"
              % (ch, len(ms), sum(1 for a in ms if a["result"] == "no_signal"),
                 mv, mv / 5.0, sum(a["detect"] for a in ms), "已清除" if got else "无源?"))
    print("   ※ 归属移动 = 该频道被检测的那些测站「进入时」的移动量。它反映「为了确认这个频道"
          "而跑到那些测站」的代价，不是该频道独占的行驶距离。")

    # ---- 冗余诊断 ----
    print()
    print("【冗余诊断】")
    dup = 0
    seen = {}
    for i, a in enumerate(measures):
        key = (round(a["pos"][0], 3), round(a["pos"][1], 3), a["ch"])
        if key in seen:
            dup += 1
        seen[key] = i
    print("   同一位置重复检测同一频道：%d 次（浪费 %.1f s）" % (dup, dup * 6.0))

    # 侦察站之间跨度（移动大头）
    if recon:
        rc_move = sum(b["move"] for b in recon)
        print("   侦察阶段移动 %.0f m（%.0f s），占全部移动 %.0f%%"
              % (rc_move, rc_move / 5.0, 100.0 * rc_move / max(1.0, moves)))
    if verify:
        vf_move = sum(b["move"] for b in verify)
        print("   验证阶段移动 %.0f m（%.0f s），占全部移动 %.0f%%"
              % (vf_move, vf_move / 5.0, 100.0 * vf_move / max(1.0, moves)))

    # ---- 路径效率：与"访问同样这些点"的最优巡回比较 ----
    pts = [(0.0, 0.0)] + [c["pos"] for c in hits]
    if len(pts) >= 3:
        best = tour_len_open(pts)
        core = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        print("   按实际清除点连成的路径长度 = %.0f m（仅参考，非真实行驶路线）" % core)
        print("   访问这些点的最优巡回(2-opt) = %.0f m" % best)
        print("   实际总移动 = %.0f m → 约为该下界的 %.2f 倍" % (moves, moves / best))

    print()
    print("【前 12 个静止测站明细】（看侦察与验证的代价）")
    for b in blocks[:12]:
        tag = "侦察" if b in recon else ("验证" if b in verify else "作业")
        print("   %-4s r=%6.0f  (%7.0f,%7.0f)  检测%3d 次  移动%6.0f m  用时%6.1f s  有向%2d 无信号%2d"
              % (tag, b["r"], b["pos"][0], b["pos"][1], b["n"], b["move"], b["dt"], b["dir"], b["no"]))


if __name__ == "__main__":
    main()
