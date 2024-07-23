import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber

def image_callback(image_mid, image_right):
    bridge = CvBridge()
    mid = bridge.imgmsg_to_cv2(image_mid, "bgr8")
    right = bridge.imgmsg_to_cv2(image_right, "bgr8")

    canvas = np.zeros((480, 1280, 3), dtype=np.uint8)
    canvas[:, :640, :] = mid
    canvas[:, 640:1280, :] = right

    cv2.imshow('Multi Camera Viewer', canvas)
    if cv2.waitKey(1) == ord("q"):
        rospy.signal_shutdown("User requested shutdown")

def main():
    rospy.init_node('camera_calibration_node')
    
    image_mid_sub = Subscriber('mid_camera', Image)
    image_right_sub = Subscriber('right_camera', Image)

    ats = ApproximateTimeSynchronizer([image_mid_sub, image_right_sub], slop=0.1, queue_size=2)
    ats.registerCallback(image_callback)

    print("Press 'q' to quit and proceed with deployment.")
    rospy.spin()

if __name__ == '__main__':
    main()

