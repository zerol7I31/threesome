# -*- coding: utf-8 -*-
"""精确核查某个漏清源：每个侦察点对它的可见性（SEE/盲区BLIND/超距OOR）。"""
import math, sys
from p3_arena import MockArena
from p3_robot import Robot, DEFAULT_PARAMS

seed = int(sys.argv[1]) if len(sys.argv) > 1 else 70039
ch_target = int(sys.argv[2]) if len(sys.argv) > 2 else 11

arena = MockArena(seed=seed, directional=True)
S = arena.sources[ch_target]
print("源 ch=%d  pos=(%.0f,%.0f)  R=%.0f  dir=%s"
      % (ch_target, S.x, S.y, S.radius, "None" if S.direction is None else "%.1f" % S.direction))
print("源到原点距离 = %.1f" % math.hypot(S.x, S.y))

# 用 arena 自带的真值判定（与模拟器一致）
def classify(P):
    if math.dist(P, (S.x, S.y)) > S.radius:
        return "OOR"
    d = math.degrees(math.atan2(P[1] - S.y, P[0] - S.x))
    diff = abs((d - S.direction + 180.0) % 360.0 - 180.0)
    return "SEE" if diff <= 90.0 else "BLIND"

r = Robot(arena)
pts = r.vantage_points()
print("\n侦察点列表（共 %d 个）：" % len(pts))
any_see = False
for i, P in enumerate(pts):
    c = classify(P)
    if c == "SEE":
        any_see = True
    dd = math.dist(P, (S.x, S.y))
    print("  [%2d] (%.0f,%.0f)  dist=%.0f  -> %s" % (i, P[0], P[1], dd, c))
print("\n是否有侦察点能看见它：", any_see)

# 在整个场地内搜索"能看见它"的点，看其分布（用于判断是否需要补侦察点）
print("\n场地内可见区中心（150m 网格扫描，取可见点质心）：")
sx = sy = n = 0.0
R = 1800.0
step = 50.0
for y in range(-R, R + 1, step):
    for x in range(-R, R + 1, step):
        if x*x + y*y <= R*R and classify((x, y)) == "SEE":
            sx += x; sy += y; n += 1
if n > 0:
    print("  可见点 %d 个，质心=(%.0f,%.0f)" % (int(n), sx/n, sy/n))
else:
    print("  场地内没有任何点能看见它（真正的几何盲区）")
