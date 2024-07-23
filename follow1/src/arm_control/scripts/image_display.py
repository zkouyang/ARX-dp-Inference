#!/usr/bin/env python

import sys
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.mid_image_sub = rospy.Subscriber("mid_camera", Image, self.mid_image_callback)
        self.right_image_sub = rospy.Subscriber("right_camera", Image, self.right_image_callback)
        self.mid_image = None
        self.right_image = None

    def mid_image_callback(self, data):
        try:
            self.mid_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def right_image_callback(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def display_images(self):
        while not rospy.is_shutdown():
            if self.mid_image is not None:
                cv2.imshow("Mid Camera", self.mid_image)
            if self.right_image is not None:
                cv2.imshow("Right Camera", self.right_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_viewer', anonymous=True)
    image_viewer = ImageViewer()
    try:
        image_viewer.display_images()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
