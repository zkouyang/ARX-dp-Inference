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
Max_step = 450
episode_idx = 999
dataset_path = f'/home/dc/data_set/transport_square/{episode_idx}.hdf5'
data_dict = {
        '/observations/qpos': [],
        '/action': [],
        '/observations/images/left' : [],
        '/observations/images/mid' : [],
        '/observations/images/right' : [],
        }


def callback(JointCTR1,JointCTR2,JointInfo1,JointInfo2,image_mid,image_left,image_right):
    global data_dict, step, Max_step, dataset_path 
    print("Master1's Joint Information:",JointCTR1)
    print("Master2's Joint Information:",JointCTR2)
    print("Follow1's Joint Information:",JointInfo1)
    print("Follow2's Joint Information:",JointInfo2)
    print("image_mid's stamp:",image_mid.header)
    print("image_left's stamp:",image_left.header)
    print("image_right's stamp:",image_right.header)# 创建一个空白图像，作为最终展示的画布
    bridge = CvBridge()
    image_mid = bridge.imgmsg_to_cv2(image_mid, "bgr8")
    image_left = bridge.imgmsg_to_cv2(image_left, "bgr8")
    image_right = bridge.imgmsg_to_cv2(image_right, "bgr8")

    action = np.concatenate((np.array(JointCTR1.joint_pos),np.array(JointCTR2.joint_pos)))
    qpos = np.concatenate((np.array(JointInfo1.joint_pos),np.array(JointInfo2.joint_pos)))
    data_dict["/action"].append(action)
    data_dict["/observations/qpos"].append(qpos)
    data_dict["/observations/images/left"].append(image_left)
    data_dict["/observations/images/mid"].append(image_mid)
    data_dict["/observations/images/right"].append(image_right)

    canvas = np.zeros((480, 1920, 3), dtype=np.uint8)

    # 将图像复制到画布的特定位置
    canvas[:, :640, :] = image_left
    canvas[:, 640:1280, :] = image_mid
    canvas[:, 1280:, :] = image_right

    # 在一个窗口中显示排列后的图像
    cv2.imshow('Multi Camera Viewer', canvas)
    cv2.waitKey(1)
    step = step+1
    if step >= Max_step:
        with h5py.File(dataset_path,'w',rdcc_nbytes=1024 ** 2 * 10) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            _ = image.create_dataset('left', (Max_step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            _ = image.create_dataset('mid', (Max_step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            _ = image.create_dataset('right', (Max_step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            _ = obs.create_dataset('qpos',(Max_step,14))
            _ = root.create_dataset('action',(Max_step,14))
            for name, array in data_dict.items():
                root[name][...] = array
        rospy.signal_shutdown("\n************************signal_shutdown********sample successfully!*************************************")
        quit("sample successfully!")
            

if __name__ =="__main__":
    #config my camera

    
    rospy.init_node("My_node1")
    master1 = Subscriber("joint_control",JointControl)
    master2 = Subscriber("joint_control2",JointControl)
    follow1 = Subscriber("joint_information",JointInformation)
    follow2 = Subscriber("joint_information2",JointInformation)
    image_mid = Subscriber("mid_camera",Image)
    image_left = Subscriber("left_camera",Image)
    image_right = Subscriber("right_camera",Image)


    ats = ApproximateTimeSynchronizer([master1,master2,follow1,follow2,image_mid,image_left,image_right],slop=0.03,queue_size=1)
    ats.registerCallback(callback)

    rospy.spin()
