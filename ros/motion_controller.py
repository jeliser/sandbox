#!/usr/bin/env python

import curses

import math
import rospy
import traceback

from enum import Enum

from std_msgs.msg import Float64, Int32, String
from geometry_msgs.msg import Vector3

## This will all get read from a configuration file
MENU = 'Use the arrow keys to drive: Up/Down = Throttle, Left/Right = Steering\n'
ROBOT = 'scout_1'
SUBSYSTEM = 'wwr/motion_controller'

WHEEL_CONTROLLER = 'wheel_controller'
STEERING_ARM_CONTROLLER = 'steering_arm_controller'

FR = 'fr'
FL = 'fl'
BR = 'br'
BL = 'bl'

class State(Enum):
    UNKNOWN = 0
    INVALID = 0
    INACTIVE = 1
    BUSY = 2
    STATIONARY = 3
    ACTIVE_BRAKING = 4
    IN_MOTION = 5

class DriveMode(Enum):
    UNKNOWN = 0
    ACKERMANN = 1
    SKID_STEER = 2


class BaseRoverMotionController():

    def __init__(self, robot):
        rospy.init_node('motor_controller', anonymous=True)

        # Initialize the controller
        self.state = State.UNKNOWN
        self.drive_mode = DriveMode.UNKNOWN
        self.wheels = dict()
        self.steer_arms  = dict()
        for _id in [FR, FL, BR, BL]:
            self.wheels[_id] = rospy.Publisher('/{}/{}_{}/command'.format(robot, _id, WHEEL_CONTROLLER), Float64, queue_size=1)
            self.steer_arms[_id] = rospy.Publisher('/{}/{}_{}/command'.format(robot, _id, STEERING_ARM_CONTROLLER), Float64, queue_size=1)

    def set_motion_command(self, data):
        ## This will probably be a normalized unit vector for the commanded direction/magnitude of motion
        throttle = data.x
        steer    = data.y

        if self.drive_mode == DriveMode.UNKNOWN:
            self.send_josh_steer_command(throttle, steer)
        elif self.drive_mode == DriveMode.SKID_STEER:
            self.send_skid_steer_command(throttle, steer)
        else: # Default ACKERMANN steering
            self.send_ackermann_command(throttle, steer)

    def send_josh_steer_command(self, throttle, steer):
        ## Because I need my own steering type :D ... this is my sandbox steering tests
        for _, v in self.wheels.items():
            v.publish(throttle)
        for k, v in self.steer_arms.items():
            v.publish(-steer if 'f' in k else steer)

    def send_ackermann_command(self, throttle, steer):
        ## Because I need my own steering type :D ... this is my sandbox steering tests
        for _, v in self.wheels.items():
            v.publish(throttle)
        for k, v in self.steer_arms.items():
            v.publish(-steer if 'f' in k else 0.0)

    def send_skid_steer_command(self, throttle, steer):
        ## Because I need my own steering type :D ... this is my sandbox steering tests
        for _, v in self.wheels.items():
            v.publish(throttle)
        for k, v in self.steer_arms.items():
            v.publish(-steer)

    def get_state(self):
        return self.state

    def set_drive_mode(self, mode):
        for e in DriveMode:
            if mode.data == e.name:
                self.drive_mode = e
                return
        print('Failed to set drive mode to {} -- {}'.format(mode, [e.name for e in DriveMode]))

    def get_drive_mode(self):
        return self.drive_mode.name

class MotionSubsystem():

    def __init__(self, robot):
        self.motion_controller = BaseRoverMotionController(robot)

        self.motion_command = rospy.Subscriber("/{}/{}/motion/command".format(robot, SUBSYSTEM), Vector3, self.motion_controller.set_motion_command);
        self.drive_mode_command = rospy.Subscriber("/{}/{}/drive_mode/command".format(robot, SUBSYSTEM), String, self.motion_controller.set_drive_mode);
        self.drive_mode = rospy.Publisher("/{}/{}/drive_mode".format(robot, SUBSYSTEM), String, queue_size=1)

    def run(self):
        rate = rospy.Rate(1.0) # 1Hz
        while not rospy.is_shutdown():
            self.drive_mode.publish(self.motion_controller.get_drive_mode())
            rate.sleep()

if __name__ == '__main__':
    try:
        control = MotionSubsystem(ROBOT)
        control.run()
    except rospy.ROSInterruptException:
        pass
    except:
        traceback.print_exc()
