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
from Reward_label.reward_label import calculate_reward
import os

def count_files_with_extension(directory, extension):
    count = 0
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                count += 1
    return count

global data_dict, step, Max_step, dataset_path 
step = 0
Max_step = 900
directory_path = f'/media/dc/T9/act/sim_transfer_cube_human'
extension = '.hdf5' 
episode_idx = count_files_with_extension(directory_path, extension)
dataset_path = f'{directory_path}/episode_{episode_idx}.hdf5'
video_path=f'/media/dc/T9/act/data/video/Clean_desk_diff/{episode_idx}'
data_dict = {
        '/observations/qpos': [],
        '/action': [],
        '/observations/images/mid' : [],
        '/rewards' : []
        }


def callback(JointCTR2,JointInfo2,image_mid):
    global data_dict, step, Max_step, dataset_path,video_path
    save=False
    bridge = CvBridge()
    image_mid = bridge.imgmsg_to_cv2(image_mid, "bgr8")

    action = np.concatenate(np.array(JointCTR2.joint_pos)[-1])
    qpos = np.concatenate(np.array(JointInfo2.joint_pos)[-1])
    if save:
        data_dict["/action"].append(action)
        data_dict["/observations/qpos"].append(qpos)
        data_dict["/observations/images/mid"].append(image_mid)

    # 在一个窗口中显示排列后的图像
    cv2.imshow('Camera Image', image_mid)
    cv2.waitKey(1)
    step = step+1
    if step >= Max_step and save:
        imgs = data_dict["/observations/images/mid"]
        data_dict['/rewards'] = calculate_reward(imgs)
        print('________________________________end__________________________________')
        with h5py.File(dataset_path,'w',rdcc_nbytes=1024 ** 2 * 10) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            _ = image.create_dataset('mid', (Max_step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            _ = obs.create_dataset('qpos',(Max_step,1))
            _ = root.create_dataset('action',(Max_step,1))
            _ = root.create_dataset('rewards',(Max_step,1))
            for name, array in data_dict.items():
                root[name][...] = array
            mid_images = root['/observations/images/mid'][...]
            images = np.concatenate([mid_images],axis=2)

            video_path = f'{video_path}video.mp4'  # Assuming dataset_path ends with ".hdf5"
            height, width, _ = images[0].shape
            fps = 30
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            for img in images:
                video_writer.write(img)
            video_writer.release()
        rospy.signal_shutdown("\n************************signal_shutdown********sample successfully!*************************************")
        quit("sample successfully!")
        

if __name__ =="__main__":
    #config my camera
    time.sleep(1)
    
    rospy.init_node("My_node1")
    a=time.time()
    master2 = Subscriber("joint_control2",JointControl)
    follow2 = Subscriber("joint_information2",JointInformation)
    image_mid = Subscriber("mid_camera",Image)
    ats = ApproximateTimeSynchronizer([master2,follow2,image_mid],slop=0.03,queue_size=2)
    ats.registerCallback(callback)
    
    rospy.spin()
    
