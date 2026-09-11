import json, math, os

FILES = {
    "12:33本地(123327)": r"C:\Users\yangj\Desktop\B题\logs\live-20260911-123327.jsonl",
    "22:11正式(221133)": r"C:\Users\yangj\Desktop\B题\logs\live-20260911-221133.jsonl",
}

def load(path):
    acts = []
    for ln in open(path, encoding="utf-8").read().splitlines():
        ln = ln.strip()
        if not ln:
            continue
        try:
            acts.append(json.loads(ln))
        except Exception:
            pass
    return acts

def analyze(name, acts):
    clears = {}      # ch -> last success clear pos
    measures = {}    # ch -> list of (pos, result, svd)
    n_meas = 0
    near_ch = set()
    for a in acts:
        p = a.get("path")
        pl = a.get("payload", {})
        rp = a.get("resp", {})
        pos = pl.get("position")
        if p == "/clear":
            if rp.get("clear_result") == "success" and pos:
                clears[int(pl["channel"])] = (pos["x"], pos["y"])
        elif p == "/measure":
            n_meas += 1
            res = rp.get("measure_result")
            if res == "near" and pos:
                near_ch.add(int(pl["channel"]))
            measures.setdefault(int(pl["channel"]), []).append(
                (pos["x"], pos["y"], res, rp.get("svd_deg")))
    srcs = list(clears.values())
    n = len(srcs)
    rs = [math.hypot(x, y) for x, y in srcs]
    cx = sum(x for x, _ in srcs) / n if n else 0
    cy = sum(y for _, y in srcs) / n if n else 0
    # 标准差（散布）
    sx = math.sqrt(sum((x - cx) ** 2 for x, _ in srcs) / n) if n else 0
    sy = math.sqrt(sum((y - cy) ** 2 for _, y in srcs) / n) if n else 0
    print("=== %s ===" % name)
    print("日志动作数=%d  检测总数=%d  成功清除源数 N=%d" % (len(acts), n_meas, n))
    print("成功清除点坐标(x,y) [即真实干扰点, 误差<20m]:")
    for ch in sorted(clears):
        x, y = clears[ch]
        print("  ch%-2d  (%8.1f,%8.1f)  |原点|=%.1f" % (ch, x, y, math.hypot(x, y)))
    if n:
        print("到圆心距离: min=%.1f  max=%.1f  mean=%.1f" % (min(rs), max(rs), sum(rs)/n))
        print("质心=(%.1f, %.1f)  散布σx=%.1f σy=%.1f" % (cx, cy, sx, sy))
        print("边界源(|r|>1500)数=%d  占%.0f%%" % (sum(1 for r in rs if r > 1500), 100*sum(1 for r in rs if r>1500)/n))
    print("出现 near(≤5m) 的频道=%d 个" % len(near_ch))
    print()
    return {"srcs": srcs, "n": n, "rs": rs, "cx": cx, "cy": cy, "sx": sx, "sy": sy,
            "near_ch": near_ch, "n_meas": n_meas, "acts": len(acts)}

R = {}
for name, path in FILES.items():
    R[name] = analyze(name, load(path))

a, b = R["12:33本地(123327)"], R["22:11正式(221133)"]
print("==== 跨局对比 ====")
print("源数: 本地 %d  vs 正式 %d" % (a["n"], b["n"]))
print("检测总数: 本地 %d  vs 正式 %d" % (a["n_meas"], b["n_meas"]))
print("mean|r|: 本地 %.1f  vs 正式 %.1f" % (sum(a["rs"])/a["n"], sum(b["rs"])/b["n"]))
print("质心: 本地 (%.1f,%.1f)  vs 正式 (%.1f,%.1f)" % (a["cx"], a["cy"], b["cx"], b["cy"]))
# 坐标是否相同（同一组布局?）
set_a = {(round(x,1), round(y,1)) for x, y in a["srcs"]}
set_b = {(round(x,1), round(y,1)) for x, y in b["srcs"]}
print("两局坐标集合相同?", set_a == set_b)
print("本地源坐标:", sorted(set_a))
print("正式源坐标:", sorted(set_b))
