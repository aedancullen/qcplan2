import subprocess
import signal
import time
import sys
import os

import numpy as np

import maestrocar

import rospy
import tf2_ros
from transformations inport euler_from_quaternion

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

last_transform = None
last_time = None

try:
    while True:
        transform = input_map.get_transform()
        time = rospy.Time()

        if transform is not None and last_transform is not None and time is not None and last_time is not None:
            time_diff = time - last_time
            sref = self.state()
            sref[0].setX(transform.translation.x)
            sref[0].setY(transform.translation.y)
            x, y, z = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ])
            sref[0].setYaw(z)
            sref[1][0] = 0 / time_diff
            sref[1][1] = 0 / time_diff
            rotation_diff = last_transform.rotation.inverseTimes(transform.rotation)
            x, y, z = euler_from_quaternion([
                rotation_diff.x,
                rotation_diff.y,
                rotation_diff.z,
                rotation_diff.w,
            ])
            sref[1][2] = z / time_diff

        last_transform = transform
        last_time = time

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

maestrocar.set_control(0, 0)

gamepadproc.send_signal(signal.SIGTERM)
gamepadproc.wait()
