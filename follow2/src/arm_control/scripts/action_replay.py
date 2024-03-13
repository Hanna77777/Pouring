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
    rate = rospy.Rate(200)
    left_action = JointControl()
    right_action = JointControl()
    with h5py.File("/home/dc/Desktop/arx-follow-V2/arx-follow/follow_control/follow1/src/arm_control/scripts/dataset/200.hdf5") as data:
        for step, action in enumerate(data["action"]):
            left_action.joint_pos = action[:7]
            
            right_action.joint_pos = action[7:]
            control_left.publish(left_action)
            control_right.publish(right_action)
            rate.sleep()
def main1():
    rospy.init_node("action_replay")
    control_left = rospy.Publisher('joint_control',JointControl,queue_size=1)
    rate = rospy.Rate(200)
    left_action = JointControl()

    with h5py.File("/home/dc/Desktop/arx-follow-V2/arx-follow/follow1/src/arm_control/scripts/test.hdf5") as data:
        for step, action in enumerate(data["action"]):
            left_action.joint_pos = action[:7]
            print(left_action.joint_pos)
            control_left.publish(left_action) 
            rate.sleep()
if __name__ == "__main__":
    main()
