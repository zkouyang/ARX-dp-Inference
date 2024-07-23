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

def callback(JointCTR1,JointCTR2,JointInfo1,JointInfo2):
    print("Master1's Joint Information:",JointCTR1)
    print("Master2's Joint Information:",JointCTR2)
    print("Follow1's Joint Information:",JointInfo1)
    print("Follow2's Joint Information:",JointInfo2)

if __name__ =="__main__":

    
    rospy.init_node("My_node1")
    master1 = Subscriber("/joint_control",JointControl)
    master2 = Subscriber("/joint_control2",JointControl)
    follow1 = Subscriber("joint_information",JointInformation)
    follow2 = Subscriber("joint_information2",JointInformation) 
    ats = ApproximateTimeSynchronizer([master1,master2,follow1,follow2],slop=0.01,queue_size=10)
    ats.registerCallback(callback)

    rospy.spin()