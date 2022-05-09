import subprocess
import signal
import time
import sys
import os

import numpy as np

import maestrocar

import rospy
import tf2_ros

from ompl import util as ou
from ompl import base as ob
from ompl import control as oc

gamepadproc = subprocess.Popen("nc -l 9867 > gamepadpipe", shell=True, stdin=subprocess.PIPE)
import gamepadpipe

import util

#ou.setLogLevel(ou.LOG_ERROR)

CHUNK_DURATION = 0.1
CHUNK_DISTANCE = 5
GOAL_THRESHOLD = 0.5

class InputMap:
    def __init__(self):
        rospy.init_node("qcplan2")
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(transform_buffer)

    def get_transform(self):
        try:
            return self.transform_buffer.lookup_transform("map", "laser", rospy.Time(0)).transform
        except:
            return None

class QCPlan2:
    def __init__(self):
        pass

try:
    while True:
        if gamepadpipe.get_updated():
            gpdata = gamepadpipe.get_data()
            if gpdata is None:
                break
            accelerator = min(-gpdata["left_stick_y"], 0.25)
            steering = gpdata["right_stick_x"]
            maestrocar.set_control(accelerator, steering)
        time.sleep(0.001)
except KeyboardInterrupt:
    pass

maestrocar.set_control(-1, 0)

gamepadproc.send_signal(signal.SIGTERM)
gamepadproc.wait()
