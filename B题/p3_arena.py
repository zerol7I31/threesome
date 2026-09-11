# -*- coding: utf-8 -*-
"""
B 题 问题 3 / 问题 4 —— 通信层

包含两个"竞技场"，接口完全一致（都是 arena.request(path, payload) -> dict）：
    MockArena : 离线模拟器。严格按附件 1/附件 2 的物理规则与计时规则实现，
                用于不限次数的本地演练与参数调优（不联网、不消耗正式测试机会）。
    HttpArena : 真实模拟器的 HTTP 客户端（默认 http://127.0.0.1:2026）。

因此同一套机器狗策略可以在两者之间无缝切换 —— 离线调好参数，再上真实模拟器。

协议要点（照抄附件，别改）：
  · 只有 4 条指令：/enter /measure /clear /exit，POST + JSON。
  · 移动与切换频道没有独立指令：
        移动   <- /measure 或 /clear 的 position
        切频道 <- 只有 /measure 的 channel 会触发，耗时 1 s
  · /clear 的 channel 是"目标源频道"，不切换测向机频道、不产生切换耗时。
  · 耗时：检测 5 s；清除命中 5 s、未命中 3 s；移动 = 直线距离/5。
  · 必须同时看 HTTP 状态码与 accepted。
  · 每个新动作换新 request_id；仅重试同一动作时复用原 ID 与原请求体。
"""

import hashlib
import json
import math
import random
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

ARENA_ID = "default"
TARGET_R = 1800.0
SPEED = 5.0                      # m/s，附件规定
T_MEASURE = 5.0
T_SWITCH = 1.0
T_CLEAR_HIT = 5.0
T_CLEAR_MISS = 3.0
NEAR_R = 5.0
CLEAR_R = 20.0
MAX_REAL_DURATION = 1200.0
MAX_VIRTUAL_DURATION = 360000.0


# ======================================================================
# 离线模拟器
# ======================================================================
class Source(object):
    def __init__(self, channel, x, y, radius, direction=None):
        self.channel = channel
        self.x = x
        self.y = y
        self.radius = radius              # 有效接收半径 ∈ [1000, 1500]
        self.direction = direction        # None = 全向；否则为定向方向(度)


class MockArena(object):
    """
    离线模拟器。n_sources=None 时按 10~16 随机；directional=True 时生成问题 4 的混合案例。
    环境误差 δ 依赖【位置 + 频道】，同一位置重复测量结果完全一致（与题目一致）。
    """

    def __init__(self, seed=None, n_sources=None, directional=False,
                 radius_range=(1000.0, 1500.0), log_path=None, dist="uniform_disk"):
        """
        dist 指定干扰源的空间分布（题目没有规定，所以必须做多种假设的压力测试）：
            uniform_disk   面积均匀（圆内均匀）—— 最自然，径向密度 ∝ r
            uniform_radius 半径均匀 —— 中心更密
            boundary       边缘环带（0.85R~1.0R）—— 最难被"中心侦察"发现
            center         中心集中（0~0.35R）
            clustered      3 个簇 —— 检验"最近邻优先"排序的收益
        """
        self.rng = random.Random(seed)
        self.directional = directional
        self.dist = dist
        n = n_sources if n_sources is not None else self.rng.randint(10, 16)
        channels = self.rng.sample(range(1, 21), n)

        # 先定簇心（clustered 用）
        self.clusters = []
        for _k in range(3):
            a = self.rng.uniform(0, 2 * math.pi)
            rr = TARGET_R * 0.6 * math.sqrt(self.rng.random())
            self.clusters.append((rr * math.cos(a), rr * math.sin(a)))

        self.sources = {}
        for idx, c in enumerate(channels):
            x, y = self._sample_position(idx)
            direction = None
            if directional and self.rng.random() < 0.5:
                direction = self.rng.uniform(0.0, 360.0)
            self.sources[c] = Source(c, x, y, self.rng.uniform(*radius_range), direction)

        self.entered = False
        self.exited = False
        self.vtime = 0.0
        self.pos = (0.0, 0.0)
        self.camera_channel = 1          # 测向机当前频道
        self.cleared = set()
        self.n_requests = 0
        self.n_rejected = 0
        self._idem = {}                  # request_id -> (path, payload, response)
        self.log = []
        self.log_path = log_path
        self.wall_start = None

    # ---------- 位置采样 ----------
    def _sample_position(self, idx):
        for _ in range(200):
            if self.dist == "uniform_disk":
                r = TARGET_R * math.sqrt(self.rng.random())
                a = self.rng.uniform(0, 2 * math.pi)
                x, y = r * math.cos(a), r * math.sin(a)
            elif self.dist == "uniform_radius":
                r = TARGET_R * self.rng.random()
                a = self.rng.uniform(0, 2 * math.pi)
                x, y = r * math.cos(a), r * math.sin(a)
            elif self.dist == "boundary":
                r = TARGET_R * self.rng.uniform(0.85, 1.0)
                a = self.rng.uniform(0, 2 * math.pi)
                x, y = r * math.cos(a), r * math.sin(a)
            elif self.dist == "center":
                r = TARGET_R * 0.35 * math.sqrt(self.rng.random())
                a = self.rng.uniform(0, 2 * math.pi)
                x, y = r * math.cos(a), r * math.sin(a)
            elif self.dist == "clustered":
                cx, cy = self.clusters[idx % 3]
                x = cx + self.rng.gauss(0.0, 300.0)
                y = cy + self.rng.gauss(0.0, 300.0)
            else:
                raise ValueError("未知分布：" + self.dist)
            if math.hypot(x, y) <= TARGET_R:
                return x, y
        return (0.0, 0.0)

    # ---------- 内部 ----------
    def _env_delta(self, pos, channel):
        """环境固定偏差 δ ∈ [-1, 1] 度：只看 (位置, 频道)，所以同点重测结果不变。"""
        key = "%.6f,%.6f,%d" % (pos[0], pos[1], channel)
        h = hashlib.md5(key.encode()).digest()
        u = int.from_bytes(h[:4], "big") / 4294967295.0
        return (u * 2.0 - 1.0) * 1.0

    def _bearing(self, S, P):
        return (math.degrees(math.atan2(P[1] - S[1], P[0] - S[0]))) % 360.0

    def _in_coverage(self, src, pos):
        if src.direction is None:
            return True
        d = math.degrees(math.atan2(pos[1] - src.y, pos[0] - src.x))
        diff = abs((d - src.direction + 180.0) % 360.0 - 180.0)
        return diff <= 90.0 + 1e-9

    def _accept(self, request_id, path, payload):
        """幂等处理：返回 (ok, cached_response)。"""
        if request_id in self._idem:
            p0, pl0, resp = self._idem[request_id]
            if p0 == path and pl0 == payload:
                return False, resp                       # 复用首次响应
            return False, {"accepted": False, "real_timestamp_ms": self._now_ms(),
                           "virtual_time_s": 0.0, "_http": 409}
        return True, None

    def _now_ms(self):
        return int(time.time() * 1000)

    def _reject(self, request_id, path, payload):
        self.n_rejected += 1
        resp = {"accepted": False, "real_timestamp_ms": self._now_ms(), "virtual_time_s": 0.0}
        self._idem[request_id] = (path, payload, resp)
        return resp

    def _move_time(self, new_pos):
        return math.dist(self.pos, new_pos) / SPEED

    # ---------- 统一入口 ----------
    def request(self, path, payload):
        self.n_requests += 1
        rid = payload.get("request_id")
        fresh, cached = self._accept(rid, path, payload)
        if not fresh:
            return cached
        if payload.get("arena_id") != ARENA_ID:
            return self._reject(rid, path, payload)

        if path == "/enter":
            resp = self._do_enter(payload)
        elif path == "/measure":
            resp = self._do_measure(payload)
        elif path == "/clear":
            resp = self._do_clear(payload)
        elif path == "/exit":
            resp = self._do_exit(payload)
        else:
            resp = {"accepted": False, "real_timestamp_ms": self._now_ms(),
                    "virtual_time_s": 0.0, "_http": 404}
        self._idem[rid] = (path, payload, resp)
        self.log.append({"path": path, "payload": payload, "resp": resp})
        if self.log_path:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps({"path": path, "payload": payload, "resp": resp},
                                   ensure_ascii=False) + "\n")
        return resp

    # ---------- 四条指令 ----------
    def _do_enter(self, payload):
        if self.entered or self.exited:
            return self._reject(payload.get("request_id"), "/enter", payload)
        self.entered = True
        self.wall_start = time.time()
        return {"accepted": True, "real_timestamp_ms": self._now_ms(),
                "virtual_time_s": self.vtime,
                "max_virtual_duration_s": MAX_VIRTUAL_DURATION,
                "max_real_duration_s": MAX_REAL_DURATION,
                "remaining_real_duration_s": MAX_REAL_DURATION}

    def _do_measure(self, payload):
        if not self.entered or self.exited:
            return self._reject(payload.get("request_id"), "/measure", payload)
        pos = (float(payload["position"]["x"]), float(payload["position"]["y"]))
        ch = int(payload["channel"])
        if not (1 <= ch <= 20):
            return {"accepted": False, "real_timestamp_ms": self._now_ms(),
                    "virtual_time_s": 0.0, "_http": 400}
        mt = self._move_time(pos)
        st = 0.0 if ch == self.camera_channel else T_SWITCH
        self.vtime += mt + st + T_MEASURE
        self.pos = pos
        self.camera_channel = ch

        src = self.sources.get(ch)
        if src is None or ch in self.cleared:
            res = "no_signal"
        else:
            d = math.dist(pos, (src.x, src.y))
            if d > src.radius:
                res = "no_signal"
            elif not self._in_coverage(src, pos):
                res = "no_signal"
            elif d <= NEAR_R:
                res = "near"
            else:
                res = "direction"
        out = {"accepted": True, "real_timestamp_ms": self._now_ms(),
               "virtual_time_s": round(self.vtime, 6), "measure_result": res}
        if res == "direction":
            true_b = self._bearing(pos, (src.x, src.y))       # 测点 -> 源 的方位角
            svd = (true_b + self._env_delta(pos, ch)) % 360.0
            out["svd_deg"] = round(svd, 2)
        return out

    def _do_clear(self, payload):
        if not self.entered or self.exited:
            return self._reject(payload.get("request_id"), "/clear", payload)
        pos = (float(payload["position"]["x"]), float(payload["position"]["y"]))
        ch = int(payload["channel"])
        if not (1 <= ch <= 20):
            return {"accepted": False, "real_timestamp_ms": self._now_ms(),
                    "virtual_time_s": 0.0, "_http": 400}
        mt = self._move_time(pos)
        src = self.sources.get(ch)
        hit = (src is not None and ch not in self.cleared
               and math.dist(pos, (src.x, src.y)) <= CLEAR_R)
        self.vtime += mt + (T_CLEAR_HIT if hit else T_CLEAR_MISS)
        self.pos = pos                    # /clear 不改测向机频道
        if hit:
            self.cleared.add(ch)
        return {"accepted": True, "real_timestamp_ms": self._now_ms(),
                "virtual_time_s": round(self.vtime, 6),
                "clear_result": "success" if hit else "no_target_in_range"}

    def _do_exit(self, payload):
        if not self.entered or self.exited:
            return self._reject(payload.get("request_id"), "/exit", payload)
        self.exited = True
        return {"accepted": True, "real_timestamp_ms": self._now_ms(),
                "virtual_time_s": round(self.vtime, 6), "exit_reason": "user_exit"}

    # ---------- 供检验用的真值 ----------
    def truth(self):
        return {
            "n_total": len(self.sources),
            "n_cleared": len(self.cleared),
            "cleared_channels": sorted(self.cleared),
            "total_virtual_time_s": round(self.vtime, 3),
            "avg_time_per_cleared": round(self.vtime / len(self.cleared), 2) if self.cleared else None,
            "ratio": len(self.cleared) / len(self.sources),
            "n_requests": self.n_requests,
            "n_rejected": self.n_rejected,
            "wall_elapsed_s": round(time.time() - (self.wall_start or time.time()), 2),
            "sources": {c: {"pos": (round(s.x, 1), round(s.y, 1)),
                            "R": round(s.radius, 0),
                            "dir": None if s.direction is None else round(s.direction, 1)}
                        for c, s in self.sources.items()},
        }


# ======================================================================
# 真实模拟器的 HTTP 客户端
# ======================================================================
class HttpArena(object):
    def __init__(self, base_url, robot_id, timeout=5.0, retries=3):
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.timeout = timeout
        self.retries = retries
        self.log = []
        self.n_requests = 0

    def request(self, path, payload):
        self.n_requests += 1
        body = json.dumps(payload).encode("utf-8")
        last_err = None
        for attempt in range(self.retries):
            # 重试必须复用完全相同的请求体与 request_id
            req = Request(self.base_url + path, data=body,
                          headers={"Content-Type": "application/json"}, method="POST")
            try:
                with urlopen(req, timeout=self.timeout) as r:
                    resp = json.loads(r.read().decode("utf-8"))
                self.log.append({"path": path, "payload": payload, "resp": resp})
                return resp
            except HTTPError as e:
                raw = e.read().decode("utf-8", "replace")
                try:
                    resp = json.loads(raw)
                except Exception:
                    resp = {"accepted": False, "http_status": e.code, "_raw": raw}
                resp.setdefault("_http", e.code)
                self.log.append({"path": path, "payload": payload, "resp": resp})
                return resp
            except (URLError, OSError, TimeoutError) as e:
                last_err = e
                time.sleep(0.3 * (attempt + 1))
        raise RuntimeError("连接模拟器失败（已重试 %d 次）：%s" % (self.retries, last_err))

    def truth(self):
        return {"note": "真实模拟器不提供真值；演练测试结束时界面会显示干扰源总数。"}
