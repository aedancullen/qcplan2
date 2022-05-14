import subprocess
import signal
import time
import sys
import os

import numpy as np

import maestrocar

import rospy
import tf2_ros
from tf.transformations import quaternion_multiply
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

        self.se2space = ob.SE2StateSpace()
        self.se2bounds = ob.RealVectorBounds(2)
        self.se2bounds.setLow(-99999) # don't care
        self.se2bounds.setHigh(99999)
        self.se2space.setBounds(self.se2bounds)

        self.vectorspace = ob.RealVectorStateSpace(3)
        self.vectorbounds = ob.RealVectorBounds(3)
        self.vectorbounds.setLow(-99999) # don't care
        self.vectorbounds.setHigh(99999)
        self.vectorspace.setBounds(self.vectorbounds)

        self.statespace = ob.CompoundStateSpace()
        self.statespace.addSubspace(self.se2space, 1) # weight 1
        self.statespace.addSubspace(self.vectorspace, 1) # weight 1

        self.controlspace = oc.RealVectorControlSpace(self.statespace, 2)

        self.state = ob.State(self.statespace)
        sref = self.state()
        sref[1][0] = 0
        sref[1][1] = 0
        sref[1][2] = 0
        self.se2space.setBounds(self.se2bounds)
        self.statespace.enforceBounds(sref)

    def loop(self):
        transform = self.input_map.get_transform()
        timestamp = rospy.Time.now()

        if transform is not None and self.last_transform is not None and timestamp is not None and self.last_timestamp is not None:
            timestamp_diff = (timestamp - self.last_timestamp).to_sec()
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
            sref[1][0] = (
                (transform.translation.x - self.last_transform.translation.x) * np.cos(-z) / timestamp_diff
                - (transform.translation.y - self.last_transform.translation.y) * np.sin(-z) / timestamp_diff
            )
            sref[1][1] = (
                (transform.translation.y - self.last_transform.translation.y) * np.cos(-z) / timestamp_diff
                + (transform.translation.x - self.last_transform.translation.x) * np.sin(-z) / timestamp_diff
            )
            rotation_diff = quaternion_multiply(
                [transform.rotation.x,
                 transform.rotation.y,
                 transform.rotation.z,
                 transform.rotation.w],
                 [self.last_transform.rotation.x,
                 self.last_transform.rotation.y,
                 self.last_transform.rotation.z,
                 -self.last_transform.rotation.w],
            )
            x, y, z = euler_from_quaternion(rotation_diff)
            sref[1][2] = z / timestamp_diff
            self.se2space.setBounds(self.se2bounds)
            self.statespace.enforceBounds(sref)
            print(self.state)

        self.last_transform = transform
        self.last_timestamp = timestamp

        gpdata = gamepadpipe.get_data()
        if gpdata is None:
            return -1
        if gpdata["a"] == 0:
            maestrocar.set_control(self.mode_teleop(gpdata))
        else:
            maestrocar.set_control(self.mode_auto(gpdata))

        return 0

    def mode_teleop(self, gpdata):
        accelerator = max(min(-gpdata["left_stick_y"], 0.25), -0.25)
        steering = gpdata["right_stick_x"]
        return accelerator, steering

    def mode_auto(self, gpdata):
        return 0, 0

if __name__ == "__main__":
    rospy.init_node("qcplan2")
    input_map = InputMap()
    qc = QCPlan2(input_map)
    try:
        while not rospy.is_shutdown():
            time.sleep(0.1)
            if qc.loop() != 0:
                break
    except Exception as e:
        print(e)
    maestrocar.set_control(0, 0)
    gamepadproc.send_signal(signal.SIGTERM)
    gamepadproc.wait()
