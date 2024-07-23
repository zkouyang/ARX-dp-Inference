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
from arm_control.msg import PosCmd
from sensor_msgs.msg import Image
import os

def count_files_with_extension(directory, extension):
    count = 0
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                count += 1
    return count

global data_dict, step, Max_step, dataset_path 

# parameters
step = 0
Max_step = 100
directory_path = f'/media/dc/HP ZHAN SSD/ATM/data/Task6_Sweeping/Human'
extension = '.hdf5' 
episode_idx = count_files_with_extension(directory_path, extension)
dataset_path = f'{directory_path}/episode_{episode_idx}.hdf5'
video_path=f'/media/dc/HP ZHAN SSD/ATM/data/Task6_Sweeping/Human/video/{episode_idx}'
data_dict = {
        '/observations/images/mid' : [],
        }


def callback(JointCTR2,JointInfo2,f2p,image_mid,image_right):
    global data_dict, step, Max_step, dataset_path,video_path
    
    save=True
    bridge = CvBridge()
    image_mid = bridge.imgmsg_to_cv2(image_mid, "bgr8")
    image_right = bridge.imgmsg_to_cv2(image_right, "bgr8")
    eef_qpos=np.array([f2p.x,f2p.y,f2p.z,f2p.roll,f2p.pitch,f2p.yaw,f2p.gripper])
    action = np.array(JointCTR2.joint_pos)
    qpos =np.array(JointInfo2.joint_pos)
    if save:
        data_dict["/observations/images/mid"].append(image_mid)

    canvas = np.zeros((480, 1280, 3), dtype=np.uint8)

    # 将图像复制到画布的特定位置
    # canvas[:, :640, :] = image_left
    # canvas[:, 640:1280, :] = image_mid
    # canvas[:, 1280:, :] = image_right
    canvas[:, :640, :] = image_mid
    canvas[:, 640:1280, :] = image_right

    # 在一个窗口中显示排列后的图像
    cv2.imshow('Multi Camera Viewer', canvas)
  
    cv2.waitKey(1)
    step = step+1
    print(step)
    if step >= Max_step and save:
        print('end__________________________________')
        with h5py.File(dataset_path,'w',rdcc_nbytes=1024 ** 2 * 10) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            _ = image.create_dataset('mid', (Max_step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            for name, array in data_dict.items():
                root[name][...] = array
            mid_images = root['/observations/images/mid'][...]
            images = mid_images

            video_path = f'{video_path}video.mp4'  # Assuming dataset_path ends with ".hdf5"
            height, width, _ = images[0].shape
            fps = 10  # 发布频率为10Hz
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            for img in images:
                video_writer.write(img)
            video_writer.release()
        rospy.signal_shutdown("\n************************signal_shutdown********sample successfully!*************************************")
        quit("sample successfully!")
        

if __name__ =="__main__":
    #config my camera
    time.sleep(2)  # wait 2s to start
    
    rospy.init_node("My_node1")
    a=time.time()
    # master1_pos = Subscriber("master1_pos_back",PosCmd)
    # master2_pos = Subscriber("master2_pos_back",PosCmd)
    # follow1_pos = Subscriber("follow1_pos_back",PosCmd)
    follow2_pos = Subscriber("follow2_pos_back",PosCmd)
    # master1 = Subscriber("joint_control",JointControl)
    master2 = Subscriber("joint_control2",JointControl)
    # follow1 = Subscriber("joint_information",JointInformation)
    follow2 = Subscriber("joint_information2",JointInformation)
    image_mid = Subscriber("mid_camera",Image)
    # image_left = Subscriber("left_camera",Image)
    image_right = Subscriber("right_camera",Image)
    print(image_right)
    ats = ApproximateTimeSynchronizer([master2,follow2,follow2_pos,image_mid,image_right],slop=0.03,queue_size=2)
    ats.registerCallback(callback)
    
    rospy.spin()
    
