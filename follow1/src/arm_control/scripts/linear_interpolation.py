import sys
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
sys.path.append("/media/dc/HP2024/dp_inference/follow1/dp")
from arm_control.msg import JointInformation
from arm_control.msg import JointControl
from message_filters import ApproximateTimeSynchronizer,Subscriber
import rospy
import h5py
import time
import numpy as np
from common.joint_trajectory_interpolator import JposTrajectoryInterpolator 

def main():
    global jpos_interp, t_now, dt

    ros_frequency = 300
    frequency = 200
    dt = 1.0 / frequency
    curr_t = time.monotonic()
    target_pose = np.zeros(7)
    jpos_interp = JposTrajectoryInterpolator(
        times=[curr_t], jposes=[target_pose]
    )

    rospy.init_node("action_replay1")
    control_sub = rospy.Subscriber("test_right", JointControl, callback)
    control_pub = rospy.Publisher("follow_joint_control_2", JointControl, queue_size = 1)
    rate = rospy.Rate(ros_frequency)

    right_action = JointControl()
    while not rospy.is_shutdown():
        t_start = time.perf_counter()
        t_now = time.monotonic()
        jpos_command = jpos_interp(t_now)

        right_action.joint_pos = jpos_command
        control_pub.publish(right_action)
        t_end = time.perf_counter()
        if t_end - t_start < dt:
            time.sleep(dt - t_end + t_start)

        rate.sleep()

def callback(msg):
    global jpos_interp, t_now, dt
    target_jpos = msg.joint_pos
    duration = 0.01
    curr_time = t_now + dt
    t_insert = curr_time + duration

    jpos_interp = jpos_interp.drive_to_waypoint(
        jpos=target_jpos,
        time=t_insert,
        curr_time=curr_time,
        max_rot_speed=np.inf,
    )

           
if __name__ == "__main__":
    main()

