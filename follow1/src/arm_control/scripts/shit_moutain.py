import sys
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
sys.path.append("/home/dc/Desktop/arx-follow-V2/arx-follow/follow_control/follow1/src/arm_control/scripts/act")


import torch
from arm_control.msg import JointInformation
from arm_control.msg import JointControl
from sensor_msgs.msg import Image
import numpy as np
import os
import pickle
import argparse
import rospy
from message_filters import ApproximateTimeSynchronizer,Subscriber
from policy import CNNMLPPolicy,ACTPolicy4infer
from visualize_episodes import save_videos
import cv2
from cv_bridge import CvBridge
import re
import time
import sys,select,termios,tty
import threading


def count_subdirectories(directory):
    count = 0
    for root, dirs, files in os.walk(directory):
        count += len(dirs)  # 直接累加子目录的数量
        break  # 如果只想计算顶层目录下的文件夹数量，就在这里中断循环
    return count




def callback(JointInfo2,image_mid,image_right):
    global pre_process, post_process,policy ,t, all_time_actions,flag, control_right,max_timesteps,picture_path
    if flag == True and t<=max_timesteps:
        with torch.inference_mode():
            flag = False
            mid = image_mid
            right = image_right
            bridge = CvBridge()
            mid = bridge.imgmsg_to_cv2(mid, "bgr8")
    
            right = bridge.imgmsg_to_cv2(right, "bgr8")

            canvas = np.zeros((480, 1280, 3), dtype=np.uint8)

        # 将图像复制到画布的特定位置
            canvas[:, :640, :] = mid
            canvas[:, 640:1280, :] = right

        # 在一个窗口中显示排列后的图像
            cv2.imshow('Multi Camera Viewer', canvas)
            cv2.imwrite(f'{picture_path}/{t}.jpg', mid)
            if cv2.waitKey(1) == ord("q"):
                pass
            color_images=[]
          
            mid=mid.transpose((2, 0, 1))
            right=right.transpose((2, 0, 1))

        
            color_images.append(mid)
            color_images.append(right)
        
            curr_image = np.stack(color_images, axis=0)
            curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
          
            # qpos_raw=np.concatenate((JointInfo2.joint_pos))
            qpos_raw=JointInfo2.joint_pos
            qpos = pre_process(qpos_raw)
            
            qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
            
            # if t%450==0:
            #     all_time_actions = torch.zeros([max_timesteps, max_timesteps+120, 14]).cuda()

            all_actions = policy(qpos, curr_image)
            all_time_actions[[t], t:t+query_frequency] = all_actions
            actions_for_curr_step = all_time_actions[:, t]
            actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
            actions_for_curr_step = actions_for_curr_step[actions_populated]
            k = 0.01
            exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
            exp_weights = exp_weights / exp_weights.sum()
            exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
            raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
            raw_action = raw_action.squeeze(0).cpu().numpy()
            action = post_process(raw_action)
            right_control = JointControl()
            print(action[:7])
            if t <300:
                action[6]=-0.05 if action[6]<0.5 else action[6]
            right_control.joint_pos = action[:7]
            control_right.publish(right_control)

            t = t+1
            flag = True

    
def main(args):
    global pre_process, post_process,policy,all_time_actions,t,query_frequency,flag,control_right,max_timesteps,picture_path
    flag = True
    ckpt_name='policy_last.ckpt'
    ckpt_dir='/media/dc/CLEAR/train_data/real_drawer/checkpoints/ACT'
    save_path='/media/dc/CLEAR/train_data/real_drawer/checkpoints/ACT'
    # os.mkdir(save_path)
  
    num= count_subdirectories(save_path)
    picture_path=f'{save_path}/apply{num+1}'
    os.mkdir(picture_path)
    chunk=50
    enc_layers = 4
    dec_layers = 7
    nheads = 8
    state_dim=7
    max_timesteps=1200
    query_frequency=chunk
    t = 0
    lr_backbone = 1e-5
    backbone = 'resnet18'
    policy_config = {'lr': 1e-5,
                         'num_queries': chunk,
                         'kl_weight': 10,
                         'hidden_dim': 512,
                         'dim_feedforward': 3200,
                         'lr_backbone': lr_backbone,
                         'backbone': backbone,
                         'enc_layers': enc_layers,
                         'dec_layers': dec_layers,
                         'nheads': nheads,
                         'camera_names': ['mid','right'],
                         }

    ckpt_path = os.path.join(ckpt_dir, ckpt_name)
    policy =ACTPolicy4infer(policy_config)
    loading_status = policy.load_state_dict(torch.load(ckpt_path))
    policy.cuda()
    policy.eval()
    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'rb') as f:
        stats = pickle.load(f)
    pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
    post_process = lambda a: a * stats['action_std'] + stats['action_mean']
    
    

    all_time_actions = torch.zeros([max_timesteps, max_timesteps+query_frequency, state_dim]).cuda()
    rospy.init_node("My_node1")
 
    follow2 = Subscriber("joint_information2",JointInformation)
    image_mid = Subscriber("mid_camera",Image)
 
    image_right = Subscriber("right_camera",Image)

    control_right = rospy.Publisher('follow_joint_control_2',JointControl,queue_size=10)
    ats = ApproximateTimeSynchronizer([follow2,image_mid,image_right],slop=0.1,queue_size=2)
    ats.registerCallback(callback)

    rospy.spin()
    
    


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval', action='store_true')
    parser.add_argument('--onscreen_render', action='store_true')

    # for ACT
    parser.add_argument('--chunk_size', action='store', type=int, help='chunk_size', required=False)
    parser.add_argument('--hidden_dim', action='store', type=int, help='hidden_dim', required=False)
    parser.add_argument('--dim_feedforward', action='store', type=int, help='dim_feedforward', required=False)
    parser.add_argument('--temporal_agg', action='store_true')
    
    main(vars(parser.parse_args()))
