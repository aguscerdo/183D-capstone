
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

#TODO These need to be static IP addresses so we dont have to keep changing them
#needs to be changed on the arduino IDE

Robot_1_IP = "ws://192.168.137.79:81/ws"
Robot_2_IP = "ws://192.168.137.9:81/ws"


Robot1 = SocketWrapper(Robot_1_IP);
Robot2 = SocketWrapper(Robot_2_IP);

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
        Robot1.send_motion('#','R',0)
        Robot2.send_motion('~',90,90)
        time.sleep(3)
        Robot1.send_motion('~',90,90)




b = Thread_B("RRT_run")
a = Thread_A("camera_position_tracking")

a.start()
b.start()

a.join()
b.join()




