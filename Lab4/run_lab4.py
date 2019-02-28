from L4Bot import L4Bot
import threading
import time
import cv2
import numpy as np
import argparse
import imutils
import time
import copy
import math
from math import pi as PI
from math import *
from MotionTrack import run_motion_track
import settings


obs = [[100, 100, 30, 30], [210, 300, 200, 80]]

class Thread_A(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def run(self):
        run_motion_track()



class Thread_B(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def run(self):
        print("start")
        db = L4Bot(711, 482, 600.0, 300.0, 0.0)  # Test Area is 711mm by 482mm
        db.load_obstacles(obs)
        # db.plot()
        start = [600, 300, 0]
        goal = [100, 400, 0]


        db.run(start, goal, branches_per_evolution=10, num_evolutions=1)
        print("done run")

a = Thread_A("camera_position_tracking")
b = Thread_B("RRT_run")

a.start()
b.start()

a.join()
b.join()