import subprocess
import signal
import time
import sys
import os

import numpy as np

import maestrocar

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

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
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)

    def get_transform(self):
        try:
            return self.transform_buffer.lookup_transform("map", "laser", rospy.Time(0)).transform
        except:
            return None

class QCPlan2:
    def __init__(self, input_map):
        self.input_map = input_map
        self.last_transform = None
        self.last_timestamp = None

    def loop(self):
        transform = self.input_map.get_transform()
        timestamp = rospy.Time()

        if transform is not None and self.last_transform is not None and timestamp is not None and self.last_timestamp is not None:
            timestamp_diff = timestamp - self.last_timestamp
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
            rotation_diff = self.last_transform.rotation.inverseTimes(transform.rotation)
            x, y, z = euler_from_quaternion([
                rotation_diff.x,
                rotation_diff.y,
                rotation_diff.z,
                rotation_diff.w,
            ])
            sref[1][2] = z / timestamp_diff

        self.last_transform = transform
        self.last_timestamp = timestamp

        if gamepadpipe.get_updated():
            gpdata = gamepadpipe.get_data()
            if gpdata is None:
                return  -1
            accelerator = max(min(-gpdata["left_stick_y"], 0.25), -0.25)
            steering = gpdata["right_stick_x"]
            maestrocar.set_control(accelerator, steering)

        return 0

if __name__ == "__main__":
    rospy.init_node("qcplan2")
    input_map = InputMap()
    qc = QCPlan2(input_map)
    try:
        while not rospy.is_shutdown():
            if qc.loop() != 0:
                break
    except Exception as e:
        print(e)
    maestrocar.set_control(0, 0)
    gamepadproc.send_signal(signal.SIGTERM)
    gamepadproc.wait()
