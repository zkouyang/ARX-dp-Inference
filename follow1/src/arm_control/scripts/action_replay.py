import sys
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
from arm_control.msg import JointInformation
from arm_control.msg import JointControl
from message_filters import ApproximateTimeSynchronizer,Subscriber
import rospy
import h5py
import time
import numpy as np

def main():
    rospy.init_node("action_replay")
    control_right = rospy.Publisher('/test_right',JointControl,queue_size=1)
    rate = rospy.Rate(10)
    right_action = JointControl()
    while not rospy.is_shutdown():
        joint_angles_rad = np.random.uniform(0, np.pi, 7)
        right_action.joint_pos = joint_angles_rad
        control_right.publish(right_action)
        rate.sleep()
           
if __name__ == "__main__":
    main()

