#!/home/dc/anaconda3/envs/dc/bin/python
import time
import rospy
import sys
from message_filters import ApproximateTimeSynchronizer,Subscriber
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
import numpy as np
import cv2
import h5py
from cv_bridge import CvBridge
from arm_control.msg import JointInformation
from arm_control.msg import JointControl
from sensor_msgs.msg import Image
import os

global data_dict, step, Max_step, dataset_path 

def callback(image_mid):
    bridge = CvBridge()
    image_mid = bridge.imgmsg_to_cv2(image_mid, "bgr8")

    canvas = np.zeros((480, 640, 3), dtype=np.uint8)

    # 将图像复制到画布的特定位置
    canvas[:, :640, :] = image_mid

    # 在一个窗口中显示排列后的图像
    cv2.imshow('check', canvas)
    cv2.waitKey(1)
  

if __name__ =="__main__":
    #config my camera
    time.sleep(1)
    
    rospy.init_node("video_check")
    a=time.time()
    image_mid = Subscriber("mid_camera",Image)
    ats = ApproximateTimeSynchronizer([image_mid],slop=0.03,queue_size=2)
    ats.registerCallback(callback)
    
    rospy.spin()
    
