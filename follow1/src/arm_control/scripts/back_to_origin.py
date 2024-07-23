import sys
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
import rospy
from arm_control.msg import JointInformation
from arm_control.msg import JointControl
import numpy as np

def main():
    rospy.init_node("initing")

    arm_right = rospy.Subscriber("joint_information2",JointInformation)

    control_right = rospy.Publisher('/follow_joint_control_2',JointControl,queue_size=1)
    Zero = np.zeros((7))
    lim = np.ones((7))*0.07

    right = JointControl()
    rate = rospy.Rate(30)
    while True:
  
        right_state = rospy.wait_for_message("joint_information2",JointInformation)

        right.joint_pos = np.clip(Zero, a_min=np.array(right_state.joint_pos)-lim, a_max=np.array(right_state.joint_pos)+lim)
      
        right.joint_pos[6]=0
       
       
        control_right.publish(right)
        rate.sleep()

if __name__ == "__main__":
    main()



 
