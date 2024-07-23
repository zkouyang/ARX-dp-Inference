#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from arm_control.msg import PosCmd
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
pos = PosCmd()
def callback(msg):
    global pos
    pos.x = msg.x
    pos.y = msg.y
    pos.z = msg.z
    pos.roll = msg.roll
    pos.pitch = msg.pitch
    pos.yaw = msg.yaw
    pos.gripper = msg.gripper


def main():
    rospy.init_node('pos_publisher', anonymous=True)
    rate = rospy.Rate(10)
    rate2 = rospy.Rate(1)
    global pos
    pos_cmd_sub = rospy.Subscriber("/follow1_pos_back", PosCmd, callback)
    pos_cmd_pub = rospy.Publisher('/follow_pos_cmd_1', PosCmd, queue_size=10)

    i = 0
    k = 1


    while not rospy.is_shutdown():
        
        pos_cmd_msg = PosCmd()
        pos_cmd_msg.x = 0
        pos_cmd_msg.y = 0.6
        pos_cmd_msg.z = 0.6
        pos_cmd_msg.roll = 0.5
        pos_cmd_msg.pitch = 0
        pos_cmd_msg.yaw = 0
        pos_cmd_msg.gripper = 0

        # 发布PosCmd消息
        pos_cmd_pub.publish(pos_cmd_msg)
        
        i+=1

        if i > 10:
            i = 0
            k = -k
        
        # input()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass