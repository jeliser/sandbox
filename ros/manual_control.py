#!/usr/bin/env python

import curses

import math
import rospy
import traceback
from std_msgs.msg import Float64

MENU = 'Use the arrow keys to drive: Up/Down = Throttle, Left/Right = Steering\n'
ROBOT = 'scout_1'

class ManualControl() :

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

        self.rate = rospy.Rate(10) # 10Hz

    def send_command(self, throttle, steer):
        self.fr_wheel.publish(throttle)
        self.fl_wheel.publish(throttle)
        self.br_wheel.publish(throttle)
        self.bl_wheel.publish(throttle)

        self.fr_steer.publish(-steer)
        self.fl_steer.publish(-steer)
        self.br_steer.publish(steer)
        self.bl_steer.publish(steer)

    def run(self):
        curses.wrapper(self._run)

    def _run(self, stdscr):
        throttle = 0.0
        steering = 0.0

        # do not wait for input when calling getch
        #stdscr.nodelay(1)
        stdscr.addstr(MENU)
        while not rospy.is_shutdown():
            # get keyboard input, returns -1 if none available
            c = stdscr.getch()
            if c != -1:
                if c == 259: # Up
                    throttle = min(throttle + 10.0, 100.0)
                elif c == 258: # Down
                    throttle = max(throttle - 10.0, -100.0)
                elif c == 260: # Left
                    steering = max(steering - (math.pi / 20.0), -math.pi)
                elif c == 261: # Right
                    steering = min(steering + (math.pi / 20.0), math.pi)
                
                self.send_command(throttle, steering)

                # print numeric value
                stdscr.move(0, 0)
                stdscr.addstr(MENU)
                stdscr.addstr('throttle: {}    steering: {}\n'.format(throttle, steering))
                stdscr.refresh()
                # return curser to start position
                stdscr.move(0, 0)

if __name__ == '__main__':
    try:
        control = ManualControl(ROBOT)
        control.run()
    except rospy.ROSInterruptException:
        traceback.print_exc()
