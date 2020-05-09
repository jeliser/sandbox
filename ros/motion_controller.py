#!/usr/bin/env python

import curses

import math
import rospy
import traceback

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

MENU = 'Use the arrow keys to drive: Up/Down = Throttle, Left/Right = Steering\n'
ROBOT = 'scout_1'
SUBSYSTEM = 'wwr/motion_controller'

class ScoutMotorControl():

    def __init__(self, robot):
        self.FR_WHEEL_TOPIC = '/{}/fr_wheel_controller/command'.format(ROBOT)
        self.FL_WHEEL_TOPIC = '/{}/fl_wheel_controller/command'.format(ROBOT)
        self.BR_WHEEL_TOPIC = '/{}/br_wheel_controller/command'.format(ROBOT)
        self.BL_WHEEL_TOPIC = '/{}/bl_wheel_controller/command'.format(ROBOT)

        self.FR_STEER_TOPIC = '/{}/fr_steering_arm_controller/command'.format(ROBOT)
        self.FL_STEER_TOPIC = '/{}/fl_steering_arm_controller/command'.format(ROBOT)
        self.BR_STEER_TOPIC = '/{}/br_steering_arm_controller/command'.format(ROBOT)
        self.BL_STEER_TOPIC = '/{}/bl_steering_arm_controller/command'.format(ROBOT)

        rospy.init_node('motor_controller', anonymous=True)
        self.fr_wheel = rospy.Publisher(self.FR_WHEEL_TOPIC, Float64, queue_size=10)
        self.fl_wheel = rospy.Publisher(self.FL_WHEEL_TOPIC, Float64, queue_size=10)
        self.br_wheel = rospy.Publisher(self.BR_WHEEL_TOPIC, Float64, queue_size=10)
        self.bl_wheel = rospy.Publisher(self.BL_WHEEL_TOPIC, Float64, queue_size=10)

        self.fr_steer = rospy.Publisher(self.FR_STEER_TOPIC, Float64, queue_size=10)
        self.fl_steer = rospy.Publisher(self.FL_STEER_TOPIC, Float64, queue_size=10)
        self.br_steer = rospy.Publisher(self.BR_STEER_TOPIC, Float64, queue_size=10)
        self.bl_steer = rospy.Publisher(self.BL_STEER_TOPIC, Float64, queue_size=10)

    def set_command(self, data):
        throttle = data.x
        steer    = data.y

        self.fr_wheel.publish(throttle)
        self.fl_wheel.publish(throttle)
        self.br_wheel.publish(throttle)
        self.bl_wheel.publish(throttle)

        self.fr_steer.publish(-steer)
        self.fl_steer.publish(-steer)
        self.br_steer.publish(steer)
        self.bl_steer.publish(steer)


class MotionSubsystem():

    def __init__(self, robot):
        self.motion_system = ScoutMotorControl(robot)

    def run(self):
        self.motion_command = rospy.Subscriber("/{}/{}/motion_command".format(ROBOT, SUBSYSTEM), Vector3, self.motion_system.set_command);
        rospy.spin()

if __name__ == '__main__':
    try:
        control = MotionSubsystem(ROBOT)
        control.run()
    except rospy.ROSInterruptException:
        traceback.print_exc()
