#!/usr/bin/python

# roscd gait_training_robot/bags/new/
# rosbag play

import math
import rosbag
import rospy
import rospkg
import scipy.io as sio
from scipy.interpolate import interp1d
import numpy as np
import time
import yaml

import time, sys
import os.path
import bag_ops
import logging
logging.basicConfig(filename=os.path.expanduser('~/.ros/calc_ga.log'), encoding='utf-8', level=logging.INFO)

import argparse
parser = argparse.ArgumentParser(description="Run the gait analyzer and foot pose estimator on data from both OptiTrack and Kinect to get the MoS RMSE.")
parser.add_argument('trial_id', type=int)
parser.add_argument('-l', '--len', type=int, default=1, help='how many trials to process in range(trial_id, trial_id + len)')
parser.add_argument('--rate', type=float, default=1.0, help='playback rate.')
parser.add_argument('-r', '--use_raw_foot_pose',      action='store_true')
parser.add_argument('-o', '--transform_to_optitrack', action='store_true')
parser.add_argument('-g', '--enable_rviz_ga',         action='store_true')
parser.add_argument('-e', '--enable_rviz_3d',         action='store_true')
parser.add_argument('-s', '--skip_bag_gen',           action='store_true')
args = parser.parse_args()

rp = rospkg.RosPack()
gta_path = rp.get_path('gait_training_robot')
bag_path = os.path.join(gta_path, 'bags/optitrack/ga/')
# bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(213,230)]]
bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(args.trial_id, args.trial_id + args.len)]]


def main():
    bag_info = {}
    with open(os.path.join(gta_path, 'bags', 'optitrack', 'bag_info.yaml'), 'r') as stream:
        try:
            bag_info = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    if not args.skip_bag_gen:
        run_both()
    for bag_name in bag_names:
        bag_file = os.path.join(bag_path, bag_name + '.bag')
        if not os.path.exists(bag_file):
            print("Skipping nonexistent bag: " + bag_file)
            continue
        mat_file = gta_path + '/matlab/mat/' + bag_name + '.mat'
        with rosbag.Bag(bag_file, 'r') as inbag:
            KINECT    = get_mos_vec_dict(inbag, '/gait_analyzer/estimate/mos_vec')
            OPTITRACK = get_mos_vec_dict(inbag, '/gait_analyzer_optitrack/estimate/mos_vec')
            t_min = OPTITRACK['t'][0]
            t_max = OPTITRACK['t'][-1]
            if bag_name in bag_info.keys() and 'time_range' in bag_info[bag_name].keys():
                t_min = max(t_min, bag_info[bag_name]['time_range'][0])
                t_max = min(t_max, bag_info[bag_name]['time_range'][1])
            trim_time(KINECT, t_min, t_max)
            KINECT['MOS_OPTITRACK'] = interp1d(OPTITRACK['t'], OPTITRACK['MOS'], axis=0, copy=True, assume_sorted=True)(KINECT['t'])
            KINECT['MOS_RMSE'] = np.sqrt(np.mean((KINECT['MOS_OPTITRACK'] - KINECT['MOS'])**2, axis=0))
            mos_arr = KINECT['MOS_RMSE'] * 100
            str_output = '{3:s}: RMSE [cm] {{MOS: {0:3.2f}, MOSAP: {1:3.2f}, MOSML: {2:3.2f}}}: '.format(mos_arr[0], mos_arr[1], mos_arr[2], bag_name)
            logging.info(str_output)
            print(str_output)

            continue

            sio.savemat(mat_file, {'KINECT': KINECT, 'OPTITRACK': OPTITRACK})
            print('Saved to ' + mat_file)

def trim_time(A, t_min, t_max):
    idx = np.logical_and(A['t'] >= t_min, A['t'] <= t_max)
    A.update({k: v[idx] for k, v in A.items()})
    
def run_both():
    bag_ops.rectify_bag_names(bag_path)

    launch_options = ['record_gait_analytics:=true', 'play_back_rate:=' + str(args.rate)]

    if args.use_raw_foot_pose: launch_options.append('foot_pose_topic_prefix:=raw')
    else: launch_options.append('foot_pose_topic_prefix:=fused')

    if args.transform_to_optitrack: launch_options.append('global_frame:=optitrack')
    else: launch_options.append('global_frame:=fused_odom')

    if args.enable_rviz_ga: launch_options.append('enable_rviz:=true rviz_config_file:=slam_rtabmap_ekf_ga.rviz')
    elif args.enable_rviz_3d: launch_options.append('enable_rviz:=true rviz_config_file:=slam_rtabmap_ekf.rviz')
    else: launch_options.append('enable_rviz:=false')

    for bag_name in bag_names:
        os.system('roslaunch gait_training_robot play_both.launch bag_name:=' + bag_name + ' ' + ' '.join(launch_options))
        time.sleep(3)
    bag_ops.rectify_bag_names(bag_path)
    # print(os.listdir(bag_path))

def get_mos_vec_dict(inbag, topic):
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics=topic)]
    return {"t": np.array([msg.header.stamp.to_sec() for msg in msg_list]),
            "MOS": np.array([[msg.vector.x, msg.vector.y, msg.vector.z] for msg in msg_list])}

if __name__ == '__main__':
    main()