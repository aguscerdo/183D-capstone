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

        experiment_idx = 1 # TODO change this to experiment number
        settings.set_idx(experiment_idx)


    def run(self):
        start, obs, goal = settings.experiment_obstacles()

        db = L4Bot(711, 482, 50.0, 50.0, 0.0)  # Test Area is 711mm by 482mm

        db.load_obstacles(obs)
        time.sleep(2)
        start = settings.get_state()
        db.run2(start, goal, branches_per_evolution=1000, num_evolutions=4, plots=True)
        print("done run")

b = Thread_B("RRT_run")
a = Thread_A("camera_position_tracking")

a.start()
b.start()

a.join()
b.join()