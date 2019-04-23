
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
from LineFollow import Follow
import settings
from socket_wrapper import SocketWrapper

def send_socket(self, uL, uR):
    print("sending uL,uR = " + str([uL, uR]))
    self.socket.send_motion(uL, uR)


server = SocketWrapper();

class Thread_A(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def run(self):
        Follow();




class Thread_B(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name


    def run(self):
        print("done run")
        while(1):
            server.send_motion(50,5);



b = Thread_B("RRT_run")
a = Thread_A("camera_position_tracking")

a.start()
b.start()

a.join()
b.join()




