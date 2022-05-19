import subprocess
import signal
import copy
import time
import sys
import os

import numpy as np
from scipy.ndimage import map_coordinates

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

CHUNK_DURATION = 0.1
CHUNK_DISTANCE = 5
GOAL_THRESHOLD = 1
GOAL_SPEED = 1

CONTROL0L = -0.25
CONTROL0H = 0.25
CONTROL1L = -1
CONTROL1H = 1

FOFS = 0
MASS = 0.1
WHEELBASE = 0.324
PHI_MAX = 0.785

CAR_X_DIM = 0.8
CAR_Y_DIM = 0.5

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
            return self.transform_buffer.lookup_transform("map", "laser", rospy.Time(0)).transform
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
        self.occupancygrid = (
            np.array(occupancygrid.data, dtype=np.int8).reshape(occupancygrid.info.height, occupancygrid.info.width),
            occupancygrid.info.resolution,
            occupancygrid.info.width,
            occupancygrid.info.height,
            occupancygrid.info.origin.translation.x,
            occupancygrid.info.origin.translation.y,
        )

class QCPlanStatePropagator(oc.StatePropagator):
    def __init__(self, si, statespace):
        super().__init__(si)
        self.statespace = statespace

    def propagate(self, state, control, duration, result):
        a = (control[0] + FOFS) / MASS
        s = state[1][0] + a * CHUNK_DURATION
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
        self.last_transform = None
        self.last_timestamp = None
        self.last_gpdata = None
        self.last_control = None

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
        self.vectorbounds.setLow(-99999) # don't care
        self.vectorbounds.setHigh(99999)
        self.vectorspace.setBounds(self.vectorbounds)

        self.statespace = ob.CompoundStateSpace()
        self.statespace.addSubspace(self.se2space, 1)
        self.statespace.addSubspace(self.vectorspace, 0)

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

        gpupdated = gamepadpipe.get_updated()
        gpdata = gamepadpipe.get_data()
        if gpdata is None:
            return -1

        if gpdata['a'] == 1:
            control = self.mode_auto(gpupdated, gpdata)
        else:
            control = self.mode_teleop(gpupdated, gpdata)

        self.last_transform = copy.copy(transform)
        self.last_timestamp = copy.copy(timestamp)
        self.last_gpdata = copy.copy(gpdata)
        self.last_control = copy.copy(control)
        return 0

    def mode_teleop(self, gpupdated, gpdata):
        time.sleep(0.02)

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
            accelerator = -gpdata["left_stick_y"] * 0.25
            steering = gpdata["right_stick_x"]
            maestrocar.set_control(accelerator, steering)
            return accelerator, steering
        else:
            return self.last_control

    def mode_auto(self, gpupdated, gpdata):
        sref = self.state()

        if self.ss.getLastPlannerStatus():
            print("Solved")
            solution = self.ss.getSolutionPath()
            controls = solution.getControls()
            accelerator = controls[0][0]
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
        self.planner.setGoalBias(0.5)
        self.planner.setPruningRadius(self.planner.getPruningRadius() / 10)

        if solution is not None:
            # Copy old path into a new PathControl (and keep a local reference) because it will be freed on self.ss.clear()
            seed_path = oc.PathControl(solution)
            self.planner.setSeedPath(seed_path, 1)

        self.ss.clear()

        start_state = ob.State(self.statespace)
        start_state_ref = start_state()
        self.propagator.propagate(sref, (accelerator, steering), 1, start_state_ref)

        goal_state = ob.State(self.statespace)
        goal_state_ref = goal_state()
        start_point = np.array([start_state_ref[0].getX(), start_state_ref[0].getY()])
        nearest_point, nearest_dist, t, i = util.nearest_point_on_trajectory(start_point, self.waypoints)
        goal_point, goal_angle, t, i = util.walk_along_trajectory(self.waypoints, t, i, CHUNK_DISTANCE)
        goal_state_ref[0].setX(goal_point[0])
        goal_state_ref[0].setY(goal_point[1])
        goal_state_ref[0].setYaw(goal_angle)
        goal_state_ref[1][0] = GOAL_SPEED

        self.se2bounds = ob.RealVectorBounds(2)
        self.se2bounds.setLow(0, min(goal_point[0], start_point[0]) - CHUNK_DISTANCE / 2)
        self.se2bounds.setLow(1, min(goal_point[1], start_point[1]) - CHUNK_DISTANCE / 2)
        self.se2bounds.setHigh(0, max(goal_point[0], start_point[0]) + CHUNK_DISTANCE / 2)
        self.se2bounds.setHigh(1, max(goal_point[1], start_point[1]) + CHUNK_DISTANCE / 2)
        self.se2space.setBounds(self.se2bounds)

        self.statespace.enforceBounds(start_state_ref)
        self.statespace.enforceBounds(goal_state_ref)
        self.ss.setStartState(start_state)
        self.ss.setGoalState(goal_state, GOAL_THRESHOLD)

        self.ss.setPlanner(self.planner)
        self.ss.solve(CHUNK_DURATION)

        return accelerator, steering

    def state_validity_check(self, state):
        if not self.statespace.satisfiesBounds(state):
            return False
        np_state = np.array([state[0].getX(), state[0].getY(), state[0].getYaw()])
        np_laserstate = np.array([self.state[0].getX(), self.state[0].getY(), self.state[0].getYaw()])
        if self.input_map.laserscan is not None and self.input_map.occupancygrid is not None:
            return util.fast_state_validity_check(
                np_state,
                np_laserstate,
                self.input_map.laserscan,
                self.input_map.occupancygrid,
                CAR_X_DIM,
                CAR_Y_DIM,
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
