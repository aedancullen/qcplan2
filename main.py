import subprocess
import signal
import copy
import time
import sys
import os

import numpy as np
from scipy.interpolate import RegularGridInterpolator

import maestrocar

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

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

CONTROL0L = -0.25
CONTROL0H = 0.25
CONTROL1L = -1
CONTROL1H = 1

GRID_LENGTH = 10
GRID0L = -10
GRID0H = 10
GRID1L = -1
GRID1H = 1
GRID2L = -10
GRID2H = 10
GRID3L = CONTROL0L
GRID3H = CONTROL0H
GRID4L = CONTROL1L
GRID4H = CONTROL1H

class InputMap:
    def __init__(self):
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)
        self.laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.occupancygrid_sub = rospy.Subscriber("/map", OccupancyGrid, self.occupancygrid_callback)

    def get_transform(self):
        try:
            return self.transform_buffer.lookup_transform("map", "laser", rospy.Time(0)).transform
        except:
            return None

    def laserscan_callback(self, laserscan):
        pass

    def occupancygrid_callback(self, occupancygrid):
        pass

class QCPlan2:
    def __init__(self, input_map):
        self.input_map = input_map
        self.last_transform = None
        self.last_timestamp = None
        self.last_gpdata = None
        self.last_control = None

        self.auto_en = False

        self.grid0 = np.linspace(GRID0L, GRID0H, GRID_LENGTH)
        self.grid1 = np.linspace(GRID1L, GRID1H, GRID_LENGTH)
        self.grid2 = np.linspace(GRID2L, GRID2H, GRID_LENGTH)
        self.grid3 = np.linspace(GRID3L, GRID3H, GRID_LENGTH)
        self.grid4 = np.linspace(GRID4L, GRID4H, GRID_LENGTH)

        try:
            with np.load("f_values.npz") as data:
                self.f_values_x = data["f_values_x"]
                self.f_values_y = data["f_values_y"]
                self.f_values_yaw = data["f_values_yaw"]
                print("Loaded f_values")
        except:
            self.f_values_x = np.zeros((GRID_LENGTH, GRID_LENGTH, GRID_LENGTH, GRID_LENGTH, GRID_LENGTH))
            self.f_values_y = np.zeros((GRID_LENGTH, GRID_LENGTH, GRID_LENGTH, GRID_LENGTH, GRID_LENGTH))
            self.f_values_yaw = np.zeros((GRID_LENGTH, GRID_LENGTH, GRID_LENGTH, GRID_LENGTH, GRID_LENGTH))

        self.interp_x = RegularGridInterpolator((self.grid0, self.grid1, self.grid2, self.grid3, self.grid4), self.f_values_x, fill_value=None)
        self.interp_y = RegularGridInterpolator((self.grid0, self.grid1, self.grid2, self.grid3, self.grid4), self.f_values_y, fill_value=None)
        self.interp_yaw = RegularGridInterpolator((self.grid0, self.grid1, self.grid2, self.grid3, self.grid4), self.f_values_yaw, fill_value=None)

        try:
            self.waypoints = np.loadtxt("waypoints.csv", delimiter=',')
            print("Loaded waypoints")
        except:
            self.waypoints = np.zeros((0, 2))

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
        self.controlbounds = ob.RealVectorBounds(2)
        self.controlbounds.setLow(0, CONTROL0L)
        self.controlbounds.setHigh(0, CONTROL0H)
        self.controlbounds.setLow(1, CONTROL1L)
        self.controlbounds.setHigh(1, CONTROL1H)
        self.controlspace.setBounds(self.controlbounds)

        self.ss = oc.SimpleSetup(self.controlspace)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.state_validity_check))
        self.ss.setStatePropagator(oc.StatePropagatorFn(self.state_propagate))

        self.si = self.ss.getSpaceInformation()
        self.si.setPropagationStepSize(1)
        self.si.setMinMaxControlDuration(1, 1)

        self.planner = oc.SST(self.si)
        self.ss.clear()
        self.ss.setPlanner(self.planner)

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

            if self.last_control is not None:
                d0 = util.discretize(GRID0L, GRID0H, GRID_LENGTH, sref[1][0])
                d1 = util.discretize(GRID1L, GRID1H, GRID_LENGTH, sref[1][1])
                d2 = util.discretize(GRID2L, GRID2H, GRID_LENGTH, sref[1][2])
                d3 = util.discretize(GRID2L, GRID2H, GRID_LENGTH, self.last_control[0])
                d4 = util.discretize(GRID2L, GRID2H, GRID_LENGTH, self.last_control[1])
                if d0 is not None and d1 is not None and d2 is not None and d3 is not None and d4 is not None:
                    self.f_values_x[d0, d1, d2, d3, d4] = (self.f_values_x[d0, d1, d2, d3, d4] + sref[1][0]) / 2
                    self.f_values_y[d0, d1, d2, d3, d4] = (self.f_values_y[d0, d1, d2, d3, d4] + sref[1][1]) / 2
                    self.f_values_yaw[d0, d1, d2, d3, d4] = (self.f_values_yaw[d0, d1, d2, d3, d4] + sref[1][2]) / 2
                    self.interp_x = RegularGridInterpolator((self.grid0, self.grid1, self.grid2, self.grid3, self.grid4), self.f_values_x, fill_value=None)
                    self.interp_y = RegularGridInterpolator((self.grid0, self.grid1, self.grid2, self.grid3, self.grid4), self.f_values_y, fill_value=None)
                    self.interp_yaw = RegularGridInterpolator((self.grid0, self.grid1, self.grid2, self.grid3, self.grid4), self.f_values_yaw, fill_value=None)
                else:
                    print("Discretized point out of bounds")

        gpupdated = gamepadpipe.get_updated()
        gpdata = gamepadpipe.get_data()
        if gpdata is None:
            return -1

        if self.auto_en:
            control = self.mode_auto(gpupdated, gpdata)
            if gpdata['a'] == 0:
                self.auto_en = False
        else:
            control = self.mode_teleop(gpupdated, gpdata)
            if gpdata['a'] == 1:
                self.auto_en = True

        self.last_transform = copy.copy(transform)
        self.last_timestamp = copy.copy(timestamp)
        self.last_gpdata = copy.copy(gpdata)
        self.last_control = copy.copy(control)
        return 0

    def mode_teleop(self, gpupdated, gpdata):
        time.sleep(0.02)

        if self.last_gpdata is not None:
            if gpdata['x'] == 1 and self.last_gpdata['x'] == 0:
                np.savez("f_values.npz", f_values_x=self.f_values_x, f_values_y=self.f_values_y, f_values_yaw=self.f_values_yaw)
                print("Saved f_values")
            if gpdata['y'] == 1 and self.last_gpdata['y'] == 0:
                np.savetxt("waypoints.csv", self.waypoints, delimiter=',')
                print("Saved waypoints")
            if gpdata['b'] == 1 and self.last_gpdata['b'] == 0:
                sref = self.state()
                to_append = np.array([(sref[0].getX(), sref[0].getY())])
                self.waypoints = np.append(self.waypoints, to_append, 0)
                print("Appended waypoint")

        if gpupdated:
            accelerator = max(min(-gpdata["left_stick_y"], 0.25), -0.25)
            steering = gpdata["right_stick_x"]
            maestrocar.set_control(accelerator, steering)
            return accelerator, steering
        else:
            return self.last_control

    def mode_auto(self, gpupdated, gpdata):
        sref = self.state()

        if self.ss.getLastPlannerStatus():
            solution = self.ssh.getSolutionPath()
            controls = solution.getControls()
            accelerator = controls[0][0]
            steering = controls[0][1]
            maestrocar.set_control(accelerator, steering)
            if not self.validate_path(sref, controls):
                self.planner = oc.SST(self.si)
                self.ss.clear()
                self.ss.setPlanner(self.planner)
        else:
            accelerator = 0
            steering = 0
            maestrocar.set_control(accelerator, steering)
            self.planner = oc.SST(self.si)
            self.ss.clear()
            self.ss.setPlanner(self.planner)

        start_state = ob.State(self.statespace)
        start_state_ref = start_state()
        self.state_propagate(sref, (accelerator, steering), 1, start_state_ref)
        self.ss.setStartState(start_state)

        goal_state = ob.State(self.statespace)
        goal_state_ref = goal_state()
        start_point = np.array([start_state_ref[0].getX(), start_state_ref[0].getY()])
        nearest_point, nearest_dist, t, i = util.nearest_point_on_trajectory(start_point, self.waypoints)
        goal_point, goal_angle, t, i = util.walk_along_trajectory(self.waypoints, t, i, CHUNK_DISTANCE)
        goal_state_ref[0].setX(goal_point[0])
        goal_state_ref[0].setY(goal_point[1])
        goal_state_ref[0].setYaw(goal_angle)
        self.ss.setGoalState(goal_state, GOAL_THRESHOLD)

        self.se2bounds = ob.RealVectorBounds(2)
        self.se2bounds.setLow(0, min(goal_point[0], start_point[0]) - CHUNK_DISTANCE / 2)
        self.se2bounds.setLow(1, min(goal_point[1], start_point[1]) - CHUNK_DISTANCE / 2)
        self.se2bounds.setHigh(0, max(goal_point[0], start_point[0]) + CHUNK_DISTANCE / 2)
        self.se2bounds.setHigh(1, max(goal_point[1], start_point[1]) + CHUNK_DISTANCE / 2)
        self.se2space.setBounds(self.se2bounds)

        self.ss.solve(CHUNK_DURATION)

        return accelerator, steering

    def state_validity_check(self, state):
        return self.statespace.satisfiesBounds(state)

    def state_propagate(self, state, control, duration, future_state):
        result_x = self.interp_x((state[1][0], state[1][1], state[1][2], control[0], control[1]))
        result_y = self.interp_y((state[1][0], state[1][1], state[1][2], control[0], control[1]))
        result_yaw = self.interp_yaw((state[1][0], state[1][1], state[1][2], control[0], control[1]))
        future_state[1][0] = result_x
        future_state[1][1] = result_y
        future_state[1][2] = result_yaw
        new_yaw = state[0].getYaw() + result_yaw * CHUNK_DURATION
        future_state[0].setX(state[0].getX() +
            result_x * np.cos(new_yaw) * CHUNK_DURATION
            - result_y * np.sin(new_yaw) * CHUNK_DURATION
        )
        future_state[0].setY(state[0].getY() +
            result_y * np.cos(new_yaw) * CHUNK_DURATION
            + result_x * np.sin(new_yaw) * CHUNK_DURATION
        )
        future_state[0].setYaw(new_yaw)

    def validate_path(state, controls):
        for control in controls:
            self.state_propagate(state, control, 1, state)
            if not state_validity_check(state):
                return False
        return True

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
