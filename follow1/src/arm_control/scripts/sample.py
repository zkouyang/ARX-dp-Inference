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

global data_dict, step, Max_step, dataset_path 
step = 0



def callback(JointCTR1,JointCTR2,JointInfo1,JointInfo2):
    global data_dict, step, Max_step, dataset_path 
    rate=rospy.Rate(90)
    action = np.concatenate((np.array(JointCTR1.joint_pos),np.array(JointCTR2.joint_pos)))
    qpos = np.concatenate((np.array(JointInfo1.joint_pos),np.array(JointInfo2.joint_pos)))
    data_dict["/action"].append(action)
    data_dict["/observations/qpos"].append(qpos)
    print(action)
    step = step+1
    
    if step >= Max_step:
        with h5py.File(dataset_path,'w',rdcc_nbytes=1024 ** 2 * 10) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            _ = obs.create_dataset('qpos',(Max_step,14))
            _ = root.create_dataset('action',(Max_step,14))
            for name, array in data_dict.items():
                root[name][...] = array
        rospy.signal_shutdown("n************************signal_shutdown********sample successfully!*************************************")
        quit("sample successfully!")
    rate.sleep()      

if __name__ =="__main__":
    #config my camera
    rospy.init_node("My_node1")
    
    master1 = Subscriber("joint_control",JointControl)
    master2 = Subscriber("joint_control2",JointControl)
    follow1 = Subscriber("joint_information",JointInformation)
    follow2 = Subscriber("joint_information2",JointInformation)
    Max_step = 1800
    dataset_path = '/home/dc/Desktop/arx-follow-V2/arx-follow/follow1/src/arm_control/scripts/test.hdf5'
    data_dict = {
        '/observations/qpos': [],
        '/action': [],
        }
    ats = ApproximateTimeSynchronizer([master1,master2,follow1,follow2],slop=0.02,queue_size=1)
    ats.registerCallback(callback)
   
    rospy.spin()
