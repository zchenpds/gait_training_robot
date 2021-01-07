#!/usr/bin/python

# roscd gait_training_robot/bags/new/
# rosbag play

import math
import rosbag
import rospy
import rospkg
import scipy.io as sio
import numpy as np

import time, sys

# update_progress() : Displays or updates a console progress bar
## Accepts a float between 0 and 1. Any int will be converted to a float.
## A value under 0 represents a 'halt'.
## A value at 1 or bigger represents 100%
def update_progress(progress, text):
    barLength = 10 # Modify this to change the length of the progress bar
    status = ""
    if isinstance(progress, int):
        progress = float(progress)
    if not isinstance(progress, float):
        progress = 0
        status = "error: progress var must be float\r\n"
    if progress < 0:
        progress = 0
        status = "Halt...\r\n"
    if progress >= 1:
        progress = 1
        status = "Done...\r\n"
    block = int(round(barLength*progress))
    text = "\r" + text + ": [{0}] {1:.2f}% {2}".format( "#"*block + "-"*(barLength-block), progress*100, status)
    sys.stdout.write(text)
    sys.stdout.flush()

rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot')
bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(194,195)]]
def main():
    for bag_name in bag_names:
        bag_path = path + '/bags/optitrack/' + bag_name + '.bag'
        mat_path = path + '/matlab/mat/' + bag_name + '_imu.mat'
        with rosbag.Bag(bag_path, 'r') as inbag:
            t = []
            A = []
            W = []
            cnt_total = inbag.get_message_count('/imu')
            i = 0
            for topic, msg, ts in inbag.read_messages(topics = '/imu'):
                i += 1
                update_progress(float(i) / cnt_total, '/imu')
                t.append([msg.header.stamp.to_sec()])
                A.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
                W.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            KIMU = {'t': np.array(t), 'A': np.array(A), 'W': np.array(W)}
            sio.savemat(mat_path, {'KIMU': KIMU})
            print('Saved to ' + mat_path)



if __name__ == '__main__':
    main()