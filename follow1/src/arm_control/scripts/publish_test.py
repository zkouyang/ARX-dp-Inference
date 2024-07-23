#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from arm_control.msg import PosCmdWithHeader
import cv2
from cv_bridge import CvBridge
import numpy as np

def main():
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(10)

    # 创建一个Publisher，发布Image类型的消息
    image_pub = rospy.Publisher('left_camera', Image, queue_size=10)
    
    # 创建一个Publisher，发布自定义消息PosCmd
    pos_cmd_pub = rospy.Publisher('follow1_pos_back', PosCmdWithHeader, queue_size=10)

    bridge = CvBridge()

    image_file = '/home/kaijun/Downloads/images/GX010325/GX010325_0.jpg'
    image = cv2.imread(image_file)

    while not rospy.is_shutdown():
        # # 创建Image消息
        # image_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        # image_msg.header.stamp = rospy.Time.now()

        # # 发布Image消息
        # image_pub.publish(image_msg)

        # 创建自定义消息PosCmd
        pos_cmd_msg = PosCmdWithHeader()
        pos_cmd_msg.x = 1.0
        pos_cmd_msg.y = 2.0
        pos_cmd_msg.z = 3.0
        pos_cmd_msg.roll = 0.1
        pos_cmd_msg.pitch = 0.2
        pos_cmd_msg.yaw = 0.3
        pos_cmd_msg.gripper = 0.5
        pos_cmd_msg.mode1 = 1
        pos_cmd_msg.mode2 = 2

        pos_cmd_msg.header.stamp = rospy.Time.now()
        # 发布PosCmd消息
        pos_cmd_pub.publish(pos_cmd_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
