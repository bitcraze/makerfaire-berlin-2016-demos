#!/usr/bin/env python
#
# The MIT License (MIT)
# Copyright (c) 2016 Bitcraze AB
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#



import rospy
import std_srvs.srv
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from crazyflie_driver.srv import UpdateParams
import math
import random
import copy
from roslib.message import Message
from sensor_msgs.msg import Joy
from std_msgs.msg._ColorRGBA import ColorRGBA

TAKEOFF_SPEED = 0.01
LANDING_SPEED = 0.01

HOVER_DISTANCE = 0.2

UPDATE_RATE = 30
RAD_RATE = 2 * math.pi / 60 / 4 / 2 # Temporary....
RADIUS = 0.4 # Meters # 0.4 good
#CENTER_X = 3
#CENTER_Y = 2.2


STEP = 0.015

LINE_START_X = 4
LINE_START_Y = 2
LINE_STOP_X = 2

RECT_START_X = 3
RECT_START_Y = 1.5
RECT_SIDE_X = 1
RECT_SIDE_Y = 1

RANDOM_QUBE_X_MIN = 2.00
RANDOM_QUBE_X_MAX = 4.00

RANDOM_QUBE_Y_MIN = 1.50
RANDOM_QUBE_Y_MAX = 2.50

RANDOM_QUBE_Z_MIN = 0.75
RANDOM_QUBE_Z_MAX = 1.50

RANDOM_REPLOT_DISTANCE = 0.2

LISEBERG_STEP = 0.01
LISEBERG_STEP_UD = 0.05
LISEBERG_MAX_ARC = 1
LISEBERG_UD_RADIUS = 0.5

SPACE_OFFSET_X = 0  # Meters
SPACE_OFFSET_Y = 0  # Meters
SPACE_OFFSET_Z = 0

CENTER_X = SPACE_OFFSET_X + 1
CENTER_Y = SPACE_OFFSET_Y + 1

DEBUG = False
DEBUG_SPEED = 0.01

class Swarmfly:

    STATE_GROUNDED = "grounded"
    STATE_LANDING = "landing"
    STATE_TAKEOFF = "takeoff"
    STATE_ASSISTED_TAKEOFF = "assistedtakeoff"
    STATE_HOVERING = "hovering"
    STATE_ASSISTED_LANDING = "assistedlanding"
    STATE_SEQUENCE_RUNNING = "sequence_running"
    STATE_SEQUENCE_STOPPING = "sequence_stopping"
    STATE_ESTOP = "estop"

    STATE_WAYPONTS = "waypoints"

    def __init__(self, sc, cfid, color, hoverpoint, takeoffpoint, landingpoint):
        self._sc = sc

        self._goal_marker = Marker()
        self._goal_marker.header.frame_id = "/world"
        self._goal_marker.header.stamp = rospy.Time.now()
        self._goal_marker.scale.x = 0.1
        self._goal_marker.scale.y = 0.1
        self._goal_marker.scale.z = 0.1
        self._goal_marker.color = color
        self._goal_marker.type = 1
        self._goal_marker.pose.position = None

        self._goal_pose = PoseStamped()
        self._goal_pose.header.frame_id = "/world"
        self._goal_pose.header.stamp = rospy.Time.now()
        self._goal_pose.pose.position = None

        self.goal = None
        self.truegoal = None

        self._truegoal_marker = copy.deepcopy(self._goal_marker)
        self._truegoal_marker.color.a = 0.5

        self.hoverpoint = hoverpoint
        self.landingpoint = landingpoint
        self.takeoffpoint = takeoffpoint
        self.position = None

        self.position = None

        self.state = Swarmfly.STATE_GROUNDED
        self.nextState = None
        self.id = cfid
        self._chaser_distance = 0.5  # Meters

        self._truegoal_maker_topic = rospy.Publisher("/crazyflie{}/truegoalmarker".format(cfid), Marker, queue_size=1)
        self._goal_pose_topic = rospy.Publisher("/crazyflie{}/goal".format(cfid), PoseStamped, queue_size=1)
        self._goal_marker_topic = rospy.Publisher("/crazyflie{}/goalmarker".format(cfid), Marker, queue_size=1)

        rospy.Subscriber("/crazyflie{}/crazyflie_position".format(cfid), Point, self._updatePosition)
        self.trigger_assisted_landing = rospy.ServiceProxy("/crazyflie{}/land".format(cfid), Empty)
        self.trigger_assisted_takeoff = rospy.ServiceProxy("/crazyflie{}/takeoff".format(cfid), Empty)
        self.reconnect = rospy.ServiceProxy("/crazyflie{}/reconnect".format(cfid), Empty)
        self._lowbatt_service = rospy.Service("/crazyflie{}/lowbatt".format(cfid), Empty, self._lowbatt_callback)

    @staticmethod
    def distance_xyz(p1, p2):
        return math.sqrt(math.pow(p2.x-p1.x, 2) + math.pow(p2.y-p1.y, 2) + math.pow(p2.z-p1.z, 2))

    @staticmethod
    def distance_xy(p1, p2):
        return math.sqrt(math.pow(p2.x-p1.x, 2) + math.pow(p2.y-p1.y, 2))

    def _updatePosition(self, data):
        self.position = Point(data.x, data.y, data.z)

    def _chaser_cap(self):
        new_goal = None
        if self.position != None:
            diff = Swarmfly.distance_xyz(self.position, self.truegoal)
            if  diff > self._chaser_distance:
                factor = self._chaser_distance/diff
                x = self.position.x + factor * (self.truegoal.x - self.position.x)
                y = self.position.y + factor * (self.truegoal.y - self.position.y)
                z = self.position.z + factor * (self.truegoal.z - self.position.z)
                new_goal = Point(x, y, z)
            else:
                new_goal = self.truegoal
        return new_goal

    def publish(self):
        self._goal_marker.header.stamp = rospy.Time.now()
        self._truegoal_marker.header.stamp = rospy.Time.now()

        if self.truegoal != None:
            self._truegoal_marker.pose.position = self.truegoal
            self._truegoal_maker_topic.publish(self._truegoal_marker)
            self.goal = self._chaser_cap()

        if self.goal != None:
            self._goal_marker.pose.position = self.goal
            self._goal_pose.pose.position = self.goal
            self._goal_marker_topic.publish(self._goal_marker)
            self._goal_pose_topic.publish(self._goal_pose)

    def isAt(self, point):
        if self.position != None:
            return Swarmfly.distance_xyz(point, self.position) < HOVER_DISTANCE
        else:
            return False

    def _lowbatt_callback(self, data):
        rospy.loginfo("{} LOW BATT".format(self.id))
        self._sc.land((self.id, ))
        return ()

    def __str__(self):
        posstr = "None"
        if self.position != None:
            posstr = "pos=(x={},y={},z={})".format(self.position.x, self.position.y, self.position.z)
        truegoalstr = "None"
        if self.truegoal != None:
            truegoalstr = "truegoal=(x={},y={},z={})".format(self.truegoal.x, self.truegoal.y, self.truegoal.z)
        goalstr = "None"
        if self.goal != None:
            goalstr = "goal=(x={},y={},z={})".format(self.goal.x, self.goal.y, self.goal.z)
        return "{}: {}, {}, {} (state={},next={})".format(self.id, posstr, truegoalstr, goalstr, self.state, self.nextState)


class SwarmController:

    def __init__(self):
        self._angle = 0;

        self._step = STEP * -1
        self._x = LINE_START_X
        self._y = LINE_START_Y

        self._x = RECT_START_X
        self._y = RECT_START_Y
        self._step_x = STEP
        self._step_y = STEP
        self._q_side = 0

        self._cf = [Swarmfly(sc=self, cfid=0, color=ColorRGBA(1, 0, 1, 1), hoverpoint=Point(SPACE_OFFSET_X+1.2, SPACE_OFFSET_Y+1.2, SPACE_OFFSET_Z+1), takeoffpoint=Point(SPACE_OFFSET_X+1.25, SPACE_OFFSET_Y+0.15, SPACE_OFFSET_Z+0.5), landingpoint=Point(SPACE_OFFSET_X+0.25, SPACE_OFFSET_Y+0.15, SPACE_OFFSET_Z+0.5)),
                    Swarmfly(sc=self, cfid=1, color=ColorRGBA(1, 1, 0, 1), hoverpoint=Point(SPACE_OFFSET_X+0.7, SPACE_OFFSET_Y+0.7, SPACE_OFFSET_Z+1), takeoffpoint=Point(SPACE_OFFSET_X+1.75, SPACE_OFFSET_Y+0.15, SPACE_OFFSET_Z+0.5), landingpoint=Point(SPACE_OFFSET_X+0.75, SPACE_OFFSET_Y+0.15, SPACE_OFFSET_Z+0.5))]

        for cf in self._cf:
            cf.hoverpoint = self._calc_circle_position(cf.id)

        self._rand_p1 = None
        self._rand_p2 = None

        self.timer = rospy.Timer(rospy.Duration(1.0/UPDATE_RATE),
                                 self._run)

    def land(self, cfids):
        # If not all Crazyflies are either hovering, running the sequence or grounded we should not triggger landing
        for cf in self._cf:
            if cf.state != Swarmfly.STATE_HOVERING and cf.state != Swarmfly.STATE_SEQUENCE_RUNNING and cf.state != Swarmfly.STATE_GROUNDED:
                rospy.loginfo("Should LAND, but at least {} is not in a good state ({})".format(cf.id, cf.state))
                return
        for cf in self._cf:
            if cf.state == Swarmfly.STATE_SEQUENCE_RUNNING or cf.state == Swarmfly.STATE_HOVERING:
                cf.state = Swarmfly.STATE_SEQUENCE_STOPPING
                # Set all to hover and then set the effected to landing
                cf.nextState = Swarmfly.STATE_HOVERING
        for cfid in cfids:
            rospy.loginfo("{} LAND!".format(cfid))
            if self._cf[cfid].nextState:
                self._cf[cfid].nextState = Swarmfly.STATE_LANDING

    def takeoff(self, cfids):
        # If not all Crazyflies are either hovering, running the sequence or grounded we should not triggger landing
        for cf in self._cf:
            if cf.state != Swarmfly.STATE_HOVERING and cf.state != Swarmfly.STATE_SEQUENCE_RUNNING and cf.state != Swarmfly.STATE_GROUNDED:
                rospy.loginfo("Should TAKEOFF, but at least {} is not in a good state ({})".format(cf.id, cf.state))
                return
        for cfid in cfids:
            if self._cf[cfid].state == Swarmfly.STATE_GROUNDED:
                rospy.loginfo("{} TAKEOFF!".format(cfid))
                if not DEBUG:
                    self._cf[cfid].reconnect()
                self._cf[cfid].truegoal = copy.deepcopy(self._cf[cfid].takeoffpoint)
                if not DEBUG:
                    self._cf[cfid].trigger_assisted_takeoff()
                self._cf[cfid].state = Swarmfly.STATE_ASSISTED_TAKEOFF
            else:
                rospy.loginfo("{} NOT TAKING OFF, NEED TO BE GROUNDED!".format(cfid))

    def emergency(self):
        rospy.loginfo("EMERGENCY STOP!")
        for i in range(len(self._cf)):
            self._cf[i].state = Swarmfly.STATE_ESTOP

    def _calc_circle_position(self, cfid):
        offset = ((2 * math.pi) / len(self._cf)) * cfid
        p = Point()
        p.x = CENTER_X + (math.cos(self._angle + offset)) * RADIUS
        p.y = CENTER_Y + (math.sin(self._angle + offset)) * RADIUS
        p.z = 1

        return p

    def _generate_random_sp(self, xmin, ymin, zmin, xmax, ymax, zmax, rmin):
        # We should also use rmin to make sure the vector to the new Point
        # is not too small..
        return  Point(random.uniform(xmin, xmax), random.uniform(ymin, ymax), random.uniform(zmin, zmax))
        #return  Point(random.uniform(xmin, xmax), random.uniform(ymin, ymax), 1)

    def _get_random_sp(self, xmin, ymin, zmin, xmax, ymax, zmax, rmin):
        if self._cf[0] is not None:
            if (self._rand_p1 is None or self._distance_points(self._rand_p1, self._cf[0]) < RANDOM_REPLOT_DISTANCE):
                self._rand_p1 = self._generate_random_sp(xmin, ymin, zmin,
                                                         xmax, ymax, zmax,
                                                         rmin)
        if self._cf[1] is not None:
            if (self._rand_p2 is None or self._distance_points(self._rand_p2, self._cf[1]) < RANDOM_REPLOT_DISTANCE):
                self._rand_p2 = self._generate_random_sp(xmin, ymin, zmin,
                                                         xmax, ymax, zmax,
                                                         rmin)


        return (self._rand_p1, self._rand_p2)


    def _runStateMachine(self, cfid):
        cf = self._cf[cfid]

        if cf.state == Swarmfly.STATE_GROUNDED:
            return

        if cf.state == Swarmfly.STATE_ASSISTED_TAKEOFF:
            if cf.isAt(cf.takeoffpoint):
                cf.truegoal = copy.deepcopy(cf.hoverpoint)
                cf.state = Swarmfly.STATE_TAKEOFF

        if cf.state == Swarmfly.STATE_TAKEOFF:
            if cf.isAt(cf.hoverpoint):
                cf.state = Swarmfly.STATE_HOVERING
                cf.nextState = Swarmfly.STATE_SEQUENCE_RUNNING

        if cf.state == Swarmfly.STATE_HOVERING:
            return

        if cf.state == Swarmfly.STATE_SEQUENCE_STOPPING:
            self._angle += RAD_RATE
            cf.truegoal = self._calc_circle_position(cfid)
            #rospy.loginfo(self._angle)
            if -0.1 < self._angle%math.pi < 0.1:
                cf.state = cf.nextState
                cf.nextState = None
                if cf.state == Swarmfly.STATE_LANDING:
                    cf.truegoal = copy.deepcopy(cf.landingpoint)
                else:
                    cf.nextState = Swarmfly.STATE_SEQUENCE_RUNNING

        if cf.state == Swarmfly.STATE_SEQUENCE_RUNNING:
            self._angle += RAD_RATE
            cf.truegoal = self._calc_circle_position(cfid)

        if cf.state == Swarmfly.STATE_LANDING:
            if cf.isAt(cf.landingpoint):
                if not DEBUG:
                    cf.trigger_assisted_landing()
                else:
                    cf.truegoal.z = 0.01
                cf.state = Swarmfly.STATE_GROUNDED

        #rospy.loginfo(cf)
        #rospy.loginfo("GOAL: {}/{}/{}".format(cf.goal.x, cf.goal.y, cf.goal.z))
        #rospy.loginfo("POS : {}/{}/{}".format(cf.position.x, cf.position.y, cf.position.z))
        #rospy.loginfo("HOV : {}/{}/{}".format(cf.hoverpoint.x, cf.hoverpoint.y, cf.hoverpoint.z))

        cf.publish()

    def _run(self, event):

        # Check if all CFs are hovering, then switch to sequence or landing
        allhovering = True
        for cf in self._cf:
            if cf.state != Swarmfly.STATE_HOVERING:
                allhovering = False
        if allhovering:
            for cf in self._cf:
                cf.state = cf.nextState
                cf.nextState = None

        for i in range(len(self._cf)):
            self._runStateMachine(i)
            # Fake always moving towards the goal
            cf = self._cf[i]
            if DEBUG and cf.truegoal != None:
                if cf.position == None:
                    rospy.loginfo("No position set, fake takeoff position")
                    cf.position = copy.deepcopy(cf.takeoffpoint)
                    cf.position.z = 0.01
                diff = Swarmfly.distance_xyz(cf.position, cf.truegoal)
                if  diff > DEBUG_SPEED:
                    factor = DEBUG_SPEED/diff
                    x = cf.position.x + factor * (cf.truegoal.x - cf.position.x)
                    y = cf.position.y + factor * (cf.truegoal.y - cf.position.y)
                    z = cf.position.z + factor * (cf.truegoal.z - cf.position.z)
                    cf.position = Point(x, y, z)


class JoystickReader:

    XBOX_A = 0
    XBOX_B = 1
    XBOX_X = 2
    XBOX_Y = 3
    XBOX_START = 7
    XBOX_XBOX = 8

    def __init__(self, swarmcontroller):
        self._sc = swarmcontroller
        topic = rospy.get_param("~joy_topic", "joy")
        rospy.Subscriber(topic, Joy, self._joystickUpdate)
        self._buttons = None

    def _isPressed(self, data, button):
        return data.buttons[button] == 1 and self._buttons[button] != data.buttons[button]

    def _joystickUpdate(self, data):
        if self._buttons != None:
            if self._isPressed(data, JoystickReader.XBOX_Y):
                self._sc.takeoff((0, ))
            if self._isPressed(data, JoystickReader.XBOX_X):
                self._sc.takeoff((1, ))
            if self._isPressed(data, JoystickReader.XBOX_B):
                self._sc.land((0, ))
            if self._isPressed(data, JoystickReader.XBOX_A):
                self._sc.land((1, ))
            if self._isPressed(data, JoystickReader.XBOX_XBOX):
                self._sc.emergency()
            if self._isPressed(data, JoystickReader.XBOX_START):
                self._sc.takeoff((0, 1))


        self._buttons = data.buttons

if __name__ == "__main__":
    rospy.init_node("swarmctrl")
    sc = SwarmController()
    jr = JoystickReader(sc)
    rospy.spin()
