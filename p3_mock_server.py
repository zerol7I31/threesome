# -*- coding: utf-8 -*-
"""
本地 HTTP 模拟器 —— 把 p3_arena.MockArena 用真实 HTTP 协议暴露出来。

用途：在真正的官方模拟器不方便反复启停时，先离线把 "机器人程序 → HTTP → 模拟器"
这条完整链路（含 JSON 编码、request_id 幂等、accepted 判定、连接失败重试）跑通，
避免把链路 bug 带到正式测试里。

用法：
    python p3_mock_server.py                 # 默认 127.0.0.1:2026
    python p3_mock_server.py --port 2027     # 换端口（官方模拟器占用 2026 时用）
    python p3_mock_server.py --directional   # 生成含定向源的案例（问题 4）
    python p3_mock_server.py --dist boundary # 指定空间分布

然后另开一个终端：
    python p3_robot.py live --url http://127.0.0.1:2026 --team TESTTEAM --wait-enter 30

本脚本只用于本地自测，与官方模拟器的任何接口/协议无关，也不会被机器狗程序调用。
"""

import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import p3_arena

ARENA = None            # 每次 /enter 重新建局


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/__truth":
            self._send(200, ARENA.truth() if ARENA else {"note": "尚未开局"})
        else:
            self._send(404, {"accepted": False, "real_timestamp_ms": 0, "virtual_time_s": 0})

    def do_POST(self):
        global ARENA
        n = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(n) if n else b""
        try:
            payload = json.loads(raw.decode("utf-8"))
        except Exception as e:
            self._send(400, {"accepted": False, "real_timestamp_ms": 0,
                             "virtual_time_s": 0, "_raw": "bad json: %s" % e})
            return

        path = self.path
        if path not in ("/enter", "/measure", "/clear", "/exit"):
            self._send(404, {"accepted": False, "real_timestamp_ms": 0, "virtual_time_s": 0})
            return

        if path == "/enter":
            ARENA = p3_arena.MockArena(seed=FLAGS.seed, directional=FLAGS.directional,
                                       dist=FLAGS.dist)
            if FLAGS.seed is not None:
                FLAGS.seed += 1

        resp = ARENA.request(path, payload)
        code = resp.pop("_http", 200)
        if FLAGS.verbose:
            print("  <- %-9s %s -> %s" % (path, payload.get("request_id"),
                                          json.dumps(resp, ensure_ascii=False)[:150]))
        self._send(code, resp)


def main():
    global FLAGS
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2026)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--directional", action="store_true")
    ap.add_argument("--dist", default="uniform_disk",
                    choices=["uniform_disk", "uniform_radius", "boundary", "center", "clustered"])
    ap.add_argument("--verbose", action="store_true")
    FLAGS = ap.parse_args()

    srv = ThreadingHTTPServer((FLAGS.host, FLAGS.port), Handler)
    print("本地模拟器已启动： http://%s:%d   (Ctrl+C 退出)" % (FLAGS.host, FLAGS.port))
    print("分布=%s  定向=%s  逐条打印=%s" % (FLAGS.dist, FLAGS.directional, FLAGS.verbose))
    print("另开一个终端运行： python p3_robot.py live --url http://%s:%d --team TESTTEAM --wait-enter 30"
          % (FLAGS.host, FLAGS.port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止。")


if __name__ == "__main__":
    main()
