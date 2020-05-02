#!/usr/bin/env python

import rospy
import traceback
from std_msgs.msg import Float64

def all_forward():
    rospy.init_node('motor_controller', anonymous=True)
    fr = rospy.Publisher('/scout_1/fr_wheel_controller/command', Float64, queue_size=10)
    fl = rospy.Publisher('/scout_1/fl_wheel_controller/command', Float64, queue_size=10)
    br = rospy.Publisher('/scout_1/br_wheel_controller/command', Float64, queue_size=10)
    bl = rospy.Publisher('/scout_1/bl_wheel_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        fr.publish(100.0)
        fl.publish(100.0)
        br.publish(100.0)
        bl.publish(100.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        all_forward()
    except rospy.ROSInterruptException:
	traceback.print_exc()
