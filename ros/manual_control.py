#!/usr/bin/env python

import curses

import rospy
import traceback
from std_msgs.msg import Float64

MENU = 'Use the arrow keys to drive: Up/Down = Throttle, Left/Right = Steering\n'
ROBOT = 'scout_1'

class ManualControl() :

    def __init__(self, robot):
        self.FR_TOPIC = '/{}/fr_wheel_controller/command'.format(ROBOT)
        self.FL_TOPIC = '/{}/fl_wheel_controller/command'.format(ROBOT)
        self.BR_TOPIC = '/{}/br_wheel_controller/command'.format(ROBOT)
        self.BL_TOPIC = '/{}/bl_wheel_controller/command'.format(ROBOT)

        rospy.init_node('motor_controller', anonymous=True)
        self.fr = rospy.Publisher(self.FR_TOPIC, Float64, queue_size=10)
        self.fl = rospy.Publisher(self.FL_TOPIC, Float64, queue_size=10)
        self.br = rospy.Publisher(self.BR_TOPIC, Float64, queue_size=10)
        self.bl = rospy.Publisher(self.BL_TOPIC, Float64, queue_size=10)
        self.rate = rospy.Rate(10) # 10Hz

    def send_command(self, fr, br, fl, bl):
        self.fr.publish(fr)
        self.fl.publish(fl)
        self.br.publish(br)
        self.bl.publish(bl)

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
                    steering = max(steering - 10.0, -100.0)
                elif c == 261: # Right
                    steering = min(steering + 10.0, 100.0)

                # Blend the commands together to get something vaguely steerable
                if steering > 0.0:
                    left  = throttle;
                    right = throttle - ((steering / 100.0) * 2.0 * throttle);
                else:
                    right = throttle
                    left  = throttle + ((steering / 100.0) * 2.0 * throttle);

                right = min(right, 100.0) if right > 0.0 else max(right, -100.0)
                left  = min(left, 100.0) if left > 0.0 else max(left, -100.0)

                self.send_command(right, right, left, left)

                # print numeric value
                stdscr.move(0, 0)
                stdscr.addstr(MENU)
                stdscr.addstr('throttle: {}    steering: {}\n'.format(throttle, steering))
                stdscr.addstr('left: {}    right: {}\n'.format(left, right))
                stdscr.refresh()
                # return curser to start position
                stdscr.move(0, 0)

if __name__ == '__main__':
    try:
        control = ManualControl(ROBOT)
        control.run()
    except rospy.ROSInterruptException:
        traceback.print_exc()
