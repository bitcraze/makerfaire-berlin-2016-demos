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
# Emergency stop button
#

import rospy
from std_srvs.srv import Empty
from rospy import ServiceException

from sensor_msgs.msg import Joy

def joystickUpdated(data):
    rospy.logfatal("Emergency stop butoon pressed!")
    if data.buttons[0] == 1:
        for e in emergencies:
            try:
                e()
            except ServiceException:
                pass


if __name__ == "__main__":
    rospy.init_node("e_stop")

    sub = rospy.Subscriber("/joy_estop", Joy, joystickUpdated)
    emergencies = []
    emergencies += [rospy.ServiceProxy("/crazyflie/emergency", Empty)]
    emergencies += [rospy.ServiceProxy("/crazyflie0/emergency", Empty)]
    emergencies += [rospy.ServiceProxy("/crazyflie1/emergency", Empty)]

    rospy.spin()
