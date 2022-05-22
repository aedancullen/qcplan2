import subprocess
import signal
import copy
import time
import sys
import os

import numpy as np

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

ou.setLogLevel(ou.LOG_INFO)

GP_DURATION = 0.02
GPACCEL = 0.25

CHUNK_DURATION = 0.2
CHUNK_DISTANCE = 2
GOAL_THRESHOLD = 0.5
GOAL_BIAS = 0.5

SPEEDL = -10
SPEEDH = 10

CONTROL0L = -0.25
CONTROL0H = 0.25
CONTROL1L = -1
CONTROL1H = 1

MASS = 0.1
COAST_THRESH = 0.1
COAST_EQUIV = 0.025

SPEED_REAL = 1
SPEED_INIT = 0.1
SPEED_STEP = 0.05
WHEELBASE = 0.324
PHI_MAX = 0.524

CAR_X_DIM = 0.45
CAR_Y_DIM = 0.30
CAR_X_EXTRA = 0.25
CAR_Y_EXTRA = 0.25

class InputMap:
    def __init__(self):
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)
        self.laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.occupancygrid_sub = rospy.Subscriber("/map", OccupancyGrid, self.occupancygrid_callback)

        self.laserscan = None
        self.occupancygrid = None

    def get_transform(self):
        try:
            return self.transform_buffer.lookup_transform("map", "rearaxle", rospy.Time(0)).transform
        except:
            return None

    def laserscan_callback(self, laserscan):
        self.laserscan = (
            np.array(laserscan.ranges),
            laserscan.angle_min,
            laserscan.angle_max,
            laserscan.angle_increment,
        )

    def occupancygrid_callback(self, occupancygrid):
        data = np.array(occupancygrid.data, dtype=np.int8).reshape(occupancygrid.info.height, occupancygrid.info.width)
        self.occupancygrid = (
            np.ma.array(data, mask=data==-1, fill_value=-1),
            occupancygrid.info.resolution,
            occupancygrid.info.width,
            occupancygrid.info.height,
            occupancygrid.info.origin.position.x,
            occupancygrid.info.origin.position.y,
        )

class QCPlanStatePropagator(oc.StatePropagator):
    def __init__(self, si, statespace):
        super().__init__(si)
        self.statespace = statespace

    def propagate(self, state, control, duration, result):
        #if np.abs(control[0]) < COAST_THRESH:
            #if state[1][0] > 0:
                #a = -COAST_EQUIV / MASS
            #else:
                #a = COAST_EQUIV / MASS
        #else:
            #a = control[0] / MASS

        s = state[1][0]# + a * CHUNK_DURATION
        distance = s * CHUNK_DURATION
        yaw = state[0].getYaw()

        if control[1] == 0:
            result[0].setX(state[0].getX() + distance * np.cos(yaw))
            result[0].setY(state[0].getY() + distance * np.sin(yaw))
            result[0].setYaw(yaw)
        else:
            radius = WHEELBASE / np.tan(-control[1] * PHI_MAX)
            curofs_x = radius * np.sin(yaw)
            curofs_y = -(radius * np.cos(yaw))
            center_x = state[0].getX() - curofs_x
            center_y = state[0].getY() - curofs_y
            traveled_angle = distance / radius
            newofs_x = radius * np.sin(yaw + traveled_angle)
            newofs_y = -(radius * np.cos(yaw + traveled_angle))
            result[0].setX(center_x + newofs_x)
            result[0].setY(center_y + newofs_y)
            result[0].setYaw(yaw + traveled_angle)
        result[1][0] = s

    def canSteer(self):
        return True

    def steer(self, from_state, to_state, result, duration):
        heading = np.arctan2(to_state[0].getY() - from_state[0].getY(), to_state[0].getX() - from_state[0].getX())
        rel = heading - from_state[0].getYaw()
        angle = np.arcsin(np.sin(rel))
        result[1] = -angle / PHI_MAX
        result[1] = min(max(result[1], CONTROL1L), CONTROL1H)

        if np.cos(rel) > 0:
            result[0] = np.random.uniform(0, CONTROL0H)
        else:
            result[0] = np.random.uniform(CONTROL0L, 0)

        return True

class QCPlan2:
    def __init__(self, input_map):
        self.input_map = input_map

        self.last_auto_en = None
        self.last_transform = None
        self.last_timestamp = None
        self.last_gpdata = None
        self.last_control = None

        self.auto_en = False

        self.extra_en = False

        self.speed_control = SPEED_INIT

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

        self.vectorspace = ob.RealVectorStateSpace(1)
        self.vectorbounds = ob.RealVectorBounds(1)
        self.vectorbounds.setLow(SPEEDL)
        self.vectorbounds.setHigh(SPEEDH)
        self.vectorspace.setBounds(self.vectorbounds)

        self.statespace = ob.CompoundStateSpace()
        self.statespace.addSubspace(self.se2space, 1)
        self.statespace.addSubspace(self.vectorspace, 1)

        self.controlspace = oc.RealVectorControlSpace(self.statespace, 2)
        self.controlbounds = ob.RealVectorBounds(2)
        self.controlbounds.setLow(0, CONTROL0L)
        self.controlbounds.setHigh(0, CONTROL0H)
        self.controlbounds.setLow(1, CONTROL1L)
        self.controlbounds.setHigh(1, CONTROL1H)
        self.controlspace.setBounds(self.controlbounds)

        self.ss = oc.SimpleSetup(self.controlspace)
        self.si = self.ss.getSpaceInformation()

        self.propagator = QCPlanStatePropagator(self.si, self.statespace)
        self.ss.setStatePropagator(self.propagator)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.state_validity_check))

        self.si.setPropagationStepSize(1)
        self.si.setMinMaxControlDuration(1, 1)

        self.state = ob.State(self.statespace)

        self.latched_laserscan = None
        self.latched_occupancygrid = None

    def loop(self):
        sref = self.state()

        transform = self.input_map.get_transform()
        timestamp = rospy.Time.now()

        if transform is not None and self.last_transform is not None and timestamp is not None and self.last_timestamp is not None:
            timestamp_diff = (timestamp - self.last_timestamp).to_sec()
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

        self.latched_laserscan = input_map.laserscan
        self.latched_occupancygrid = input_map.occupancygrid

        if self.latched_laserscan is not None and self.latched_occupancygrid is not None:
            np_laserstate = np.array([sref[0].getX(), sref[0].getY(), sref[0].getYaw()])
            util.scan_overlay(
                np_laserstate,
                self.latched_laserscan,
                self.latched_occupancygrid,
            )

        gpupdated = gamepadpipe.get_updated()
        gpdata = gamepadpipe.get_data()
        if gpdata is None:
            return -1

        if gpdata['a'] == 1:
            self.auto_en = True
        if gpdata['b'] == 1:
            self.auto_en = False

        if self.auto_en:
            control = self.mode_auto(gpupdated, gpdata)
        else:
            control = self.mode_teleop(gpupdated, gpdata)

        self.last_auto_en = self.auto_en
        self.last_transform = copy.copy(transform)
        self.last_timestamp = copy.copy(timestamp)
        self.last_gpdata = copy.copy(gpdata)
        self.last_control = copy.copy(control)
        return 0

    def mode_teleop(self, gpupdated, gpdata):
        time.sleep(GP_DURATION)

        if self.last_gpdata is not None:
            if gpdata['x'] == 1 and self.last_gpdata['x'] == 0:
                sref = self.state()
                to_append = np.array([(sref[0].getX(), sref[0].getY())])
                self.waypoints = np.append(self.waypoints, to_append, 0)
                print("Appended waypoint")
            if gpdata['y'] == 1 and self.last_gpdata['y'] == 0:
                np.savetxt("waypoints.csv", self.waypoints, delimiter=',')
                print("Saved waypoints")

        if gpupdated:
            accelerator = -gpdata["left_stick_y"] * GPACCEL
            steering = gpdata["right_stick_x"]
            maestrocar.set_control(accelerator, steering)
            return accelerator, steering
        else:
            return self.last_control

    def mode_auto(self, gpupdated, gpdata):
        sref = self.state()

        if self.ss.haveExactSolutionPath() and self.last_auto_en:
            print("Solved")
            solution = self.ss.getSolutionPath()
            controls = solution.getControls()
            self.speed_control = self.speed_control + SPEED_STEP if sref[1][0] < SPEED_REAL else self.speed_control - SPEED_STEP
            self.speed_control = min(max(self.speed_control, 0), CONTROL0H)
            accelerator = self.speed_control#controls[0][0]
            steering = controls[0][1]
            maestrocar.set_control(accelerator, steering)
        else:
            print("Unsolved")
            solution = None
            accelerator = 0
            steering = 0
            maestrocar.set_control(accelerator, steering)

        print(accelerator, steering)

        self.planner = oc.SST(self.si)
        self.planner.setGoalBias(GOAL_BIAS)
        self.planner.setPruningRadius(self.planner.getPruningRadius() / 10)

        if solution is not None:
            # Copy old path into a new PathControl (and keep a local reference) because it will be freed on self.ss.clear()
            seed_path = oc.PathControl(solution)
            self.planner.setSeedPath(seed_path, 1)

        self.ss.clear()

        start_state = ob.State(self.statespace)
        start_state_ref = start_state()
        self.propagator.propagate(sref, (accelerator, steering), 1, start_state_ref)
        start_state_ref[1][0] = SPEED_REAL

        start_point = np.array([start_state_ref[0].getX(), start_state_ref[0].getY()])
        nearest_point, nearest_dist, t, i = util.nearest_point_on_trajectory(start_point, self.waypoints)
        goal_point, goal_angle, t, i = util.walk_along_trajectory(self.waypoints, t, i, CHUNK_DISTANCE)

        goal_se2space = ob.SE2StateSpace()
        goal_se2bounds = ob.RealVectorBounds(2)
        goal_se2bounds.setLow(0, goal_point[0] - GOAL_THRESHOLD)
        goal_se2bounds.setHigh(0, goal_point[0] + GOAL_THRESHOLD)
        goal_se2bounds.setLow(1, goal_point[1] - GOAL_THRESHOLD)
        goal_se2bounds.setHigh(1, goal_point[1] + GOAL_THRESHOLD)
        goal_se2space.setBounds(goal_se2bounds)

        goal_vectorspace = ob.RealVectorStateSpace(1)
        goal_vectorbounds = ob.RealVectorBounds(1)
        goal_vectorbounds.setLow(-99999) # don't care
        goal_vectorbounds.setHigh(99999)
        goal_vectorspace.setBounds(goal_vectorbounds)

        goal_statespace = ob.CompoundStateSpace()
        goal_statespace.addSubspace(goal_se2space, 1)
        goal_statespace.addSubspace(goal_vectorspace, 1)

        goal_si = ob.SpaceInformation(goal_statespace)
        goal_space = ob.GoalSpace(goal_si)
        goal_space.setSpace(goal_statespace)

        self.se2bounds = ob.RealVectorBounds(2)
        self.se2bounds.setLow(0, min(goal_point[0], start_point[0]) - CHUNK_DISTANCE / 2)
        self.se2bounds.setLow(1, min(goal_point[1], start_point[1]) - CHUNK_DISTANCE / 2)
        self.se2bounds.setHigh(0, max(goal_point[0], start_point[0]) + CHUNK_DISTANCE / 2)
        self.se2bounds.setHigh(1, max(goal_point[1], start_point[1]) + CHUNK_DISTANCE / 2)
        self.se2space.setBounds(self.se2bounds)

        self.statespace.enforceBounds(start_state_ref)
        self.ss.setStartState(start_state)
        self.ss.setGoal(goal_space)

        self.extra_en = True
        self.extra_en = self.state_validity_check(start_state_ref)

        self.ss.setPlanner(self.planner)
        self.ss.solve(CHUNK_DURATION)

        return accelerator, steering

    def state_validity_check(self, state):
        sref = self.state()

        if not self.statespace.satisfiesBounds(state):
            return False

        if self.latched_laserscan is not None and self.latched_occupancygrid is not None:
            np_state = np.array([state[0].getX(), state[0].getY(), state[0].getYaw()])
            np_laserstate = np.array([sref[0].getX(), sref[0].getY(), sref[0].getYaw()])
            return util.state_validity_check(
                np_state,
                np_laserstate,
                self.latched_laserscan,
                self.latched_occupancygrid,
                CAR_X_DIM + CAR_X_EXTRA if self.extra_en else CAR_X_DIM,
                CAR_Y_DIM + CAR_Y_EXTRA if self.extra_en else CAR_Y_DIM,
            )
        else:
            return True

if __name__ == "__main__":
    rospy.init_node("qcplan2")
    input_map = InputMap()
    qc = QCPlan2(input_map)
    try:
        while not rospy.is_shutdown():
            if qc.loop() != 0:
                break
    finally:
        print("Exiting, zeroing controls")
        maestrocar.set_control(0, 0)
        gamepadproc.send_signal(signal.SIGTERM)
        gamepadproc.wait()
