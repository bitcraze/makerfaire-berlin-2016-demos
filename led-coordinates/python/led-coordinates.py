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


#
# This script subscribes to the position of the Crazyflie in ROS and sends it
# to the arduino via serial port to display the posigion on the LED-strips
# representing the axis in a coordinate system of the stand.
#

ROS_ENABLED = True

import serial
import time
import sys
import threading

if len(sys.argv) < 2:
    print("usage: {} <serial_port>".format(sys.argv[0]))
    sys.exit(1)

SERIAL_PORT = sys.argv[1]

if ROS_ENABLED:
    import rospy
    from geometry_msgs.msg import Point, PoseStamped

lock = threading.Lock()


def float_to_ser(m):
    if m<0:
        m=0
    cm = min(int(round(m * 100)), 256 * 256)
    low = cm & 0x0ff
    high = int(cm / 256)
    return bytearray([high, low])


def sendFloat(port, val):
    data = float_to_ser(val)
    port.write(data)


def send(port, pos, x, y, z):
    with lock:
        port.write(bytearray([0xbc]))
        port.write(bytearray([pos]))
        sendFloat(port, x)
        sendFloat(port, y)
        sendFloat(port, z)
        port.flushOutput()
        # Allow the Arduion to handle data. Might need to be increased if nr of
        # concurrent positions is changed.
        time.sleep(0.01)


def update_strip_position(data):
    send(port, 0, data.x, data.y, data.z)
    # print(data)

def update_strip_goal(data):
    send(port, 1, data.pose.position.x,
                  data.pose.position.y,
                  data.pose.position.z)

port = serial.Serial()
port.port = SERIAL_PORT
port.baudrate = 115200
port.open()

if ROS_ENABLED:
    rospy.init_node('led_coordinate')
    position_sub = rospy.Subscriber("/crazyflie/crazyflie_position", Point,
                                    update_strip_position)
    # goal_sub = rospy.Subscriber("/crazyflie/goal", PoseStamped,
    #                             update_strip_goal)
    rospy.spin()
else:
    while True:
        for x in range(0, 200):
            v = x / 100.0
            vv = 2.0 - v

            send(port, 1, vv, vv, vv)
            send(port, 0, v, v, v)

port.close()
