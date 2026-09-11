# -*- coding: utf-8 -*-
import os
os.environ["P4DEBUG"] = "1"
import p3_arena
from p3_robot import Robot

seed = int(__import__("sys").argv[1]) if len(__import__("sys").argv) > 1 else 70000
arena = p3_arena.MockArena(seed=seed, directional=True)
r = Robot(arena)
rep = r.run()
print("RESULT cleared %d / %d  vtime %.1f" % (rep["cleared"], len(arena.sources), rep["vtime"]))
