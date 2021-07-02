#!/usr/bin/python

import rospkg
import rosbag

import numpy as np
import scipy.io as sio

import os
import glob
import argparse
import time
import yaml

import bag_ops
import ga

parser = argparse.ArgumentParser(description="Play a bag file of user selection:")
parser.add_argument('-r', '--record_all',                action='store_true')
parser.add_argument('-p', '--export_mat',                action='store_true')
parser.add_argument('-s', '--skip_bag_gen',              action='store_true')
args = parser.parse_args()

rp = rospkg.RosPack()
gta_path = rp.get_path('gait_training_robot')
bag_path = os.path.join(gta_path, 'bags')

launch_options = []

if args.record_all:
    launch_options.append('record_all:=true')
else:
    launch_options = ['enable_rviz:=false', 'enable_kpe:=false', 'enable_fpe:=false', 
                  'enable_ga:=false']

bag_names = [os.path.basename(f) for f in sorted(glob.glob(bag_path + '/*.bag'))]
bag_indices = range(0, len(bag_names))
print('\n'.join(['[{0: <5}] '.format(i) + n for i, n in zip(bag_indices, bag_names)]))
while True:
    bag_index_sel = input('Please enter the index of bag to be selected:')
    if bag_index_sel >= 0 and bag_index_sel < len(bag_names):
        break
    print('Input must be within [0, ' + str(len(bag_names)) + ')')


in_bag_path = os.path.join(bag_path, bag_names[bag_index_sel])
print(in_bag_path)


launch_options.append("bag_path1:=" + in_bag_path)
launch_options.append("bag_name:=" + bag_names[bag_index_sel])

def main():
    if not args.skip_bag_gen:
        os.system("roslaunch gait_training_robot sunny_play_local.launch " + ' '.join(launch_options))
        time.sleep(3)
        bag_ops.rectify_bag_names(os.path.join(gta_path, "bags", "sunny", "ga"))

    bag_name = os.path.splitext(bag_names[bag_index_sel])[0]
    out_bag_path = os.path.join(bag_path, "sunny", "ga", bag_name[:7] + '.bag')

    if os.path.exists(out_bag_path):

        # Read bag_info.yaml
        bag_info = {}
        with open(os.path.join(gta_path, 'bags', 'sunny', 'bag_info.yaml'), 'r') as stream:
            try:
                bag_info = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        mat_file = os.path.join('/home/ral2020/projects/gta_data2/sunny/simple', bag_names[bag_index_sel][:7] + '.mat')
        with rosbag.Bag(out_bag_path, 'r') as inbag:
            # Compare MoS
            KINECT = ga.get_mos_vec_dict(inbag, '/gait_analyzer/estimate/mos_vec')
            KINECT = ga.merge(KINECT, ga.get_gait_state_dict(inbag, '/gait_analyzer/gait_state'))

            t_min = KINECT['t'][0]
            t_max = KINECT['t'][-1]
            if bag_name in bag_info.keys() and 'time_range' in bag_info[bag_name].keys():
                t_min = bag_info[bag_name]['time_range'][0]
                t_max = bag_info[bag_name]['time_range'][1]
                trim_time(KINECT, t_min, t_max)
            
            stance_intervals = ga.get_stance_intervals(inbag, '/gait_analyzer/gait_state', t_min, t_max)
            STEP_KINECT    = ga.StepData(inbag, '/foot_pose_estimator/fused_pose_', stance_intervals)

            strideL = np.array(np.concatenate(STEP_KINECT.get_stride_lengths())) * 100
            stepL   = np.array(np.concatenate(STEP_KINECT.get_step_lengths())) * 100
            stepW   = np.array(np.concatenate(STEP_KINECT.get_step_widths())) * 100
            str_output = 'StrideL: {:3.1f}({:3.1f}) cm, StepL: {:3.1f}({:3.1f}) cm, StepW: {:3.1f}({:3.1f}) cm'.format(
                np.mean(strideL), np.std(strideL), 
                np.mean(stepL), np.std(stepL), 
                np.mean(stepW), np.std(stepW))
            print(str_output)

            if args.export_mat:
                assert(os.path.exists(os.path.dirname(mat_file)))
                sio.savemat(mat_file, {'KINECT': KINECT})
                print('Saved to ' + mat_file)


def trim_time(A, t_min, t_max):
    idx = np.logical_and(A['t'] >= t_min, A['t'] <= t_max)
    A.update({k: v[idx] for k, v in A.items()})

if __name__ == '__main__':
    main()