import sys
sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
import rospy
from arm_control.msg import JointInformation
from arm_control.msg import JointControl
from constant import target_position
import numpy as np

def main():
    rospy.init_node("Target")
    control_right = rospy.Publisher('joint_control2',JointControl,queue_size=1)
    Target = target_position()
    lim = np.ones((7))*0.07
    right = JointControl()
    rate = rospy.Rate(30)
    while True:
        right_state = rospy.wait_for_message("joint_information2",JointInformation)
        right.joint_pos = np.clip(Target, a_min=np.array(right_state.joint_pos)-lim, a_max=np.array(right_state.joint_pos)+lim)
        print(f"action:{right.joint_pos},state:{right_state.joint_pos}\n")
        control_right.publish(right)
        rate.sleep()
if __name__ == "__main__":
    main()



 
