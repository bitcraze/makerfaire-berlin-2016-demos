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
from std_msgs.msg import Float32
from std_srvs.srv import Empty

ALPHA = 0.95
VOLTAGE_LIMIT = 3.15

filtered_bat = 3.7

def batteryUpdated(data):
    global filtered_bat
    filtered_bat = ALPHA*filtered_bat + (1-ALPHA)*data.data
    # print("Bat: {}".format(filtered_bat))
    if filtered_bat < VOLTAGE_LIMIT:
        lowbat()
        rospy.logwarn("Low battery detected, calling lowbat service!")

if __name__ == "__main__":
    rospy.init_node("bat_supervisor")

    battery_sub = rospy.Subscriber("battery", Float32, batteryUpdated)
    lowbat = rospy.ServiceProxy("lowbat", Empty)

    rospy.spin()
