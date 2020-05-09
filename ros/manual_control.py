#!/usr/bin/env python

import curses

import math
import rospy
import traceback

from geometry_msgs.msg import Vector3

MENU = 'Use the arrow keys to drive: Up/Down = Throttle, Left/Right = Steering\n'
ROBOT = 'scout_1'
SUBSYSTEM = 'wwr/motion_controller'

class ManualControl() :

    def __init__(self, robot):
        rospy.init_node('manual_input', anonymous=True)
        self.command = rospy.Publisher("/{}/{}/motion_command".format(ROBOT, SUBSYSTEM), Vector3, queue_size=1)

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
                
                self.command.publish(x=throttle, y=steering, z=0.0)

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
