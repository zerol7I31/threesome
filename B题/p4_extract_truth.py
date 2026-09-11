import re, json, math, sys

FILES = {
    "12:33本地(123327)": r"C:\Users\yangj\Desktop\B题\logs\live-20260911-123327.html",
    "22:11正式(221133)": r"C:\Users\yangj\Desktop\B题\logs\live-20260911-221133.html",
}

def extract(path):
    txt = open(path, encoding="utf-8").read()
    idx = txt.find("const DATA")
    if idx < 0:
        raise SystemExit("DATA not found in " + path)
    start = txt.find("{", idx)
    depth = 0
    for i in range(start, len(txt)):
        c = txt[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
    blob = txt[start:end]
    return json.loads(blob)

def analyze(name, D):
    print("[DEBUG] %s top keys: %s" % (name, list(D.keys())))
    print("[DEBUG] meta:", json.dumps(D.get("meta", {}), ensure_ascii=False)[:200])
    print("[DEBUG] truth present:", D.get("truth") is not None)
    S = (D.get("truth") or {}).get("sources", {})
    if not S:
        # maybe sources at top
        S = D.get("sources", {})
    if not S:
        print("  (无 truth.sources 内嵌真实源坐标)")
        return None
    rows = []
    for k, s in S.items():
        pos = s.get("pos") or [s.get("x"), s.get("y")]
        rows.append({
            "ch": int(k),
            "x": pos[0], "y": pos[1],
            "r": s.get("R") or s.get("radius"),
            "dir": s.get("dir") if s.get("dir") is not None else s.get("direction"),
        })
    rows.sort(key=lambda r: r["ch"])
    n = len(rows)
    rs = [math.hypot(r["x"], r["y"]) for r in rows]
    dirs = [r["dir"] for r in rows if r["dir"] is not None]
    print("=== %s ===" % name)
    print("源数 N =", n)
    print("坐标(x,y) / 半径R / 方向φ_c(度, None=全向):")
    for r in rows:
        print("  ch%-2d  (%7.1f,%7.1f)  R=%-5s  φ=%s" % (r["ch"], r["x"], r["y"],
              ("%.0f" % r["r"]) if r["r"] is not None else "?",
              ("%.1f" % r["dir"]) if r["dir"] is not None else "全向"))
    print("到圆心距离: min=%.1f max=%.1f mean=%.1f" % (min(rs), max(rs), sum(rs)/n))
    print("半径R: min=%s max=%s" % (min((r['r'] for r in rows if r['r'] is not None), default='?'),
                                    max((r['r'] for r in rows if r['r'] is not None), default='?')))
    print("全向数=%d  定向数=%d (占%.0f%%)" % (n-len(dirs), len(dirs), 100*len(dirs)/n))
    print()
    return rows

results = {}
for name, path in FILES.items():
    results[name] = analyze(name, extract(path))

# 跨局对比
print("==== 跨局对比 ====")
a, b = results["12:33本地(123327)"], results["22:11正式(221133)"]
print("本地有真实源坐标:", a is not None, " | 正式有真实源坐标:", b is not None)
if a and b:
    print("源数: 本地 %d  vs 正式 %d" % (len(a), len(b)))
    set_a = {(round(r['x'],1), round(r['y'],1)) for r in a}
    set_b = {(round(r['x'],1), round(r['y'],1)) for r in b}
    print("坐标集合完全一致?", set_a == set_b)
    print("本地独有:", set_a - set_b)
    print("正式独有:", set_b - set_a)
