import sys
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
import rospy
from arm_control.msg import JointInformation
from arm_control.msg import JointControl
from constant import target_position
import numpy as np
def main():
    rospy.init_node("Process_node")
    control_right = rospy.Publisher('joint_control2',JointControl,queue_size=1)
    target = target_position()
    Current_state = rospy.wait_for_message("joint_information2",JointInformation)
    lim = 0.05
    assert all(Current_state.joint_pos[i] < target[i] + lim and Current_state.joint_pos[i] > target[i] - lim for i in range(5))
    while(1):
        raw_control = rospy.wait_for_message("joint_control2_raw",JointControl)
        raw_control.joint_pos[0:5] = target[0:5]
        raw_control.joint_pos[-1] = target[-1]
        control_right.publish(raw_control)
        
