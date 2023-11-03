#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16

def control_servo(angle):
    msg = UInt16()
    msg.data = angle
    servo_pub.publish(msg)
   

if __name__ == '__main__':
    rospy.init_node('servo_control')
    servo_pub = rospy.Publisher('servo', UInt16, queue_size=10)
   
    angle = 10  # Set the desired angle here
    while not rospy.is_shutdown():
        control_servo(angle)