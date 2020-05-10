#!/usr/bin/env python

import curses

import math
import rospy
import traceback

from std_msgs.msg import String
from geometry_msgs.msg import Vector3

MENU = 'Use the arrow keys to drive: Up/Down = Throttle, Left/Right = Steering\nPress \'m\' to change drive modes\n\n'
ROBOT = 'scout_1'
SUBSYSTEM = 'wwr/motion_controller'

DRIVE_MODES = ['UNKNOWN', 'ACKERMANN', 'CRABBING', 'SKID_STEER']
        
class ManualControl() :

    def __init__(self, robot):
        rospy.init_node('manual_input', anonymous=True)
        self.motion_command = rospy.Publisher("/{}/{}/motion/command".format(ROBOT, SUBSYSTEM), Vector3, queue_size=1)
        self.drive_mode_command = rospy.Publisher("/{}/{}/drive_mode/command".format(ROBOT, SUBSYSTEM), String, queue_size=1)

    def show_menu(self, stdscr, drive_mode, throttle, steering):
        # print numeric value
        stdscr.move(0, 0)
        stdscr.addstr(MENU)
        stdscr.addstr('drive_mode: {}\n'.format(drive_mode))
        stdscr.addstr('throttle: {}    steering: {}\n'.format(throttle, steering))
        stdscr.refresh()
        # return curser to start position
        stdscr.move(0, 0)

    def run(self):
        curses.wrapper(self._run)

    def _run(self, stdscr):
        throttle = 0.0
        steering = 0.0
        drive_mode = 0

        # do not wait for input when calling getch
        self.show_menu(stdscr, DRIVE_MODES[drive_mode], throttle, steering)
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
                elif c == ord('m'): # Drive Mode
                    drive_mode = (drive_mode + 1) % len(DRIVE_MODES)
                    self.drive_mode_command.publish(data=DRIVE_MODES[drive_mode])
                
                self.motion_command.publish(x=throttle, y=steering, z=0.0)

                # Print out the menu
                self.show_menu(stdscr, DRIVE_MODES[drive_mode], throttle, steering)

if __name__ == '__main__':
    try:
        control = ManualControl(ROBOT)
        control.run()
    except rospy.ROSInterruptException:
        traceback.print_exc()
