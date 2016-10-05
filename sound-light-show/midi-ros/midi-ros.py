#!/usr/bin/env python
ROS_ENABLED = True


#
# Listens for MIDI messages that are translated to ROS events and published in
# ROS to control the position and color of the LED-ring of the Crazyflie.
#

import mido
import colorsys
import time

if ROS_ENABLED:
    import rospy

    from geometry_msgs.msg import PoseStamped, Point
    from crazyflie_driver.srv import UpdateParams
    from std_srvs.srv import Empty

# Uses Mido and portmidi
# Install portmidi with brew or similar
# Install Mido with pip install mido

MIDI_PORT = 'CME U2MIDI MIDI 1'

CHANNEL_LED = 0
CHANNEL_X = 1
CHANNEL_Y = 2
CHANNEL_Z = 3

# The range of MIDI notes to use.
MIDI_NOTE_MIN = 0
MIDI_NOTE_MAX = 107

X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2

# The physical space the MIDI notes are mapped to
SPACE_MIN = [0.6, 0.6, 3.0]
SPACE_MAX = [1.4, 1.4, 4.0]
FLIGHT_FLOOR = SPACE_MIN[2] - 0.5

isFirstMidiMessageHandled = False
initialDataPublishTime = 0.0
INTIAL_PUB_INTERVAL = 1.0

setpoint = [1.0, 1.0, 1.0]
bellow_floor = True

def publishPosition(setpoint):
    if ROS_ENABLED:
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.pose.position.x = setpoint[X_AXIS]
        goal.pose.position.y = setpoint[Y_AXIS]
        goal.pose.position.z = setpoint[Z_AXIS]
        goalPublisher.publish(goal)
    else:
        print("[ROS disabled] Publish position ({}, {}, {})".format(
            setpoint[X_AXIS],
            setpoint[Y_AXIS],
            setpoint[Z_AXIS]))


def publishLedColor(r, g, b):
    global bellow_floor
    if not bellow_floor and ROS_ENABLED:
        rospy.set_param("/crazyflie/ring/effect", 7)
        update_params(["ring/effect"])
        rospy.set_param("/crazyflie/ring/solidRed", r)
        update_params(["ring/solidRed"])
        rospy.set_param("/crazyflie/ring/solidGreen", g)
        update_params(["ring/solidGreen"])
        rospy.set_param("/crazyflie/ring/solidBlue", b)
        update_params(["ring/solidBlue"])
    else:
        print("[ROS disabled] Publish LED color ({}, {}, {})".format(r, g, b))


def handleSetPointMessage(axis, message):
    global bellow_floor
    if bellow_floor:
        rospy.logwarn("Bellow floor mode, ignoring position update")
    if message.type == 'note_on':
        distancePerNote = (SPACE_MAX[axis] - SPACE_MIN[axis]) / \
                          (MIDI_NOTE_MAX - MIDI_NOTE_MIN)
        value = (message.note - MIDI_NOTE_MIN) * distancePerNote + SPACE_MIN[
            axis]
        setpoint[axis] = value
        # print("Setpoint ({}, {}, {})".format(setpoint[0], setpoint[1],
        #                                     setpoint[2]))

        publishPosition(setpoint)


def handleLedMessage(message):
    if message.type == 'note_on' or message.type == 'note_off':
        h = message.note / 127.0
        s = message.velocity / 127.0

        if message.type == 'note_on':
            v = 1.0
        else:
            v = 0.0

        rgb = colorsys.hsv_to_rgb(h, s, v)

        r = int(rgb[0] * 255)
        g = int(rgb[1] * 255)
        b = int(rgb[2] * 255)
        print("LED ring r:{} g:{} b:{}".format(r, g, b))

        publishLedColor(r, g, b)


def handleMessage(message):
    # print(message)
    if hasattr(message, 'channel'):
        if message.channel == CHANNEL_LED:
            handleLedMessage(message)
        if message.channel == CHANNEL_X:
            handleSetPointMessage(X_AXIS, message)
        if message.channel == CHANNEL_Y:
            handleSetPointMessage(Y_AXIS, message)
        if message.channel == CHANNEL_Z:
            handleSetPointMessage(Z_AXIS, message)


def isRosAlive():
    if ROS_ENABLED:
        return not rospy.is_shutdown()
    else:
        return True

def lowbat_callback(data):
    # global lowbat
    # if not lowbat:
    #     rospy.logwarn("Lowbat called!")
    #     lowbat = True
    #     publishBaselData()
    rospy.loginfo("Calling landing!")
    landProxy()
    return ()

def position_updated(data):
    global bellow_floor
    if data.z < FLIGHT_FLOOR:
        bellow_floor = True
        publishBaselData()
        rospy.loginfo("Bellow floor!")
    else:
        bellow_floor = False

def publishBaselData():
    centerX = (SPACE_MAX[X_AXIS] + SPACE_MIN[X_AXIS]) / 2.0
    centerY = (SPACE_MAX[Y_AXIS] + SPACE_MIN[Y_AXIS]) / 2.0
    centerZ = (SPACE_MAX[Z_AXIS] + SPACE_MIN[Z_AXIS]) / 2.0

    publishPosition([centerX, centerY, centerZ])
    publishLedColor(0, 0, 0)


if ROS_ENABLED:
    rospy.init_node('ros_midi')
    goalPublisher = rospy.Publisher('/crazyflie/goal', PoseStamped,
                                    queue_size=10)
    update_params = rospy.ServiceProxy('/crazyflie/update_params',
                                       UpdateParams)
    lowbatService = rospy.Service("/crazyflie/lowbat", Empty, lowbat_callback)
    landProxy = rospy.ServiceProxy("/crazyflie/land", Empty)

    positionSubscriber = rospy.Subscriber('/crazyflie/crazyflie_position',
                                          Point, position_updated)

print("Midi in ports available:")
for name in mido.get_input_names():
    print("    '{}'".format(name))
print()

print("Opening midi port " + MIDI_PORT)
print("")

with mido.open_input(MIDI_PORT) as inPort:
    while isRosAlive():
        for message in inPort.iter_pending():
            try:
                isFirstMidiMessageHandled = True
                handleMessage(message)
            except AttributeError as e:
                print("Oups!")
                print(message)
                print(e)

        mido.ports.sleep()

        if not isFirstMidiMessageHandled:
            if (time.time() - initialDataPublishTime) > INTIAL_PUB_INTERVAL:
                publishBaselData()
                initialDataPublishTime = time.time()
