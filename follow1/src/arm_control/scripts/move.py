
import sys
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
from arm_control.msg import JointInformation
from arm_control.msg import JointControl
from message_filters import ApproximateTimeSynchronizer,Subscriber
import rospy
import h5py

def main():
    rospy.init_node("action_replay")
    control_left = rospy.Publisher('joint_control',JointControl,queue_size=1)
    control_right = rospy.Publisher('joint_control2',JointControl,queue_size=1)
    rate = rospy.Rate(30)
    left_action = JointControl()
    right_action = JointControl()
    with h5py.File("/home/dc/data_set/transport_square/1.hdf5") as data:
        for step, action in enumerate(data["action"]):
            left_action.joint_pos = action[:7]
            right_action.joint_pos = action[7:]
            print(action)
            control_left.publish(left_action)
            control_right.publish(right_action)
            rate.sleep()
if __name__ == "__main__":
    main()
                               