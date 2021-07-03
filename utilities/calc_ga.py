#!/usr/bin/python

# roscd gait_training_robot/bags/new/
# rosbag play

import datetime
import math
import rosbag
import rospy
import rospkg
import scipy.io as sio
from scipy.interpolate import interp1d
import numpy as np
import time
import yaml

import ga

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
parser.add_argument('-m', '--message', type=str, default='', help='Remarks to be logged.')
parser.add_argument('-zs', '--comkf_measurement_scheme', type=int, default=2)
parser.add_argument('-rawfpose', '--use_raw_foot_pose',  action='store_true')
parser.add_argument('-o', '--transform_to_optitrack',    action='store_true')
parser.add_argument('-rawodom', '--express_in_raw_odom', action='store_true')
parser.add_argument('-g', '--enable_rviz_ga',            action='store_true')
parser.add_argument('-e', '--enable_rviz_3d',            action='store_true')
parser.add_argument('-s', '--skip_bag_gen',              action='store_true')
parser.add_argument('-p', '--export_mat',                action='store_true')
args = parser.parse_args()



def main():
    rp = rospkg.RosPack()
    gta_path = rp.get_path('gait_training_robot')
    bag_path = os.path.join(gta_path, 'bags/optitrack/ga/')

    bag_info = {}
    with open(os.path.join(gta_path, 'bags', 'optitrack', 'bag_info.yaml'), 'r') as stream:
        try:
            bag_info = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(args.trial_id, args.trial_id + args.len)]]
    bag_names_skipped = [bag_name for bag_name in bag_names if bag_name not in bag_info.keys()]
    if bag_names_skipped: print("Skipping bags not specified in bag_info: " + ' '.join(bag_names_skipped))
    bag_names = [bag_name for bag_name in bag_names if bag_name in bag_info.keys()]
    if not args.skip_bag_gen:
        run_both(bag_names, bag_path)
    for bag_name in bag_names:
        bag_file = os.path.join(bag_path, bag_name + '.bag')
        if not os.path.exists(bag_file):
            print("Skipping nonexistent bag: " + bag_file)
            continue
        # mat_file = gta_path + '/matlab/mat/' + bag_name + '.mat'
        mat_file = os.path.join('/home/ral2020/projects/gta_data2/OptiTrack/simple', bag_name + '.mat')
        with rosbag.Bag(bag_file, 'r') as inbag:
            # Compare MoS
            KINECT_OV = ga.get_mos_vec_dict(inbag, '/gait_analyzer/estimate/mos_vec')
            OPTITRACK = ga.get_mos_vec_dict(inbag, '/gait_analyzer_optitrack/estimate/mos_vec')
            KINECT_OV = ga.merge(KINECT_OV, ga.get_gait_state_dict(inbag, '/gait_analyzer/gait_state'))
            OPTITRACK = ga.merge(OPTITRACK, ga.get_gait_state_dict(inbag, '/gait_analyzer_optitrack/gait_state'))
            t_min = OPTITRACK['t'][0]
            t_max = OPTITRACK['t'][-1]
            if bag_name in bag_info.keys() and 'time_range' in bag_info[bag_name].keys():
                t_min = max(t_min, bag_info[bag_name]['time_range'][0])
                t_max = min(t_max, bag_info[bag_name]['time_range'][1])
            
            # Loop {0: OV(overall), 1: ST(straight)}
            KINECT_ST = KINECT_OV.copy()
            for c in [0, 1]:
                if c == 0:
                    KINECT = KINECT_OV
                    trim_time(KINECT, [(t_min, t_max)])
                    c_str = 'OV'
                else:
                    KINECT = KINECT_ST
                    time_ranges_straight = ga.get_straight_segment_time_ranges(inbag, (t_min, t_max))
                    trim_time(KINECT, time_ranges_straight)
                    c_str = 'ST'
                    print("time_ranges_straight", time_ranges_straight)
                
                KINECT['MOS_OPTITRACK'] = interp1d(OPTITRACK['t'], OPTITRACK['MOS'], axis=0, copy=True, assume_sorted=True)(KINECT['t'])
                KINECT['MOS_RMSE'] = np.sqrt(np.mean((KINECT['MOS_OPTITRACK'] - KINECT['MOS'])**2, axis=0))
                mos_rmse = KINECT['MOS_RMSE'] * 100

                # Compare foot pose
                stance_intervals = ga.get_stance_intervals(inbag, '/gait_analyzer/gait_state', t_min, t_max)
                STEP_KINECT    = ga.StepData(inbag, '/foot_pose_estimator/fused_pose_', stance_intervals)
                STEP_OPTITRACK = ga.StepData(inbag, '/optitrack/foot_pose_', stance_intervals)
                STEP_ERROR = STEP_KINECT - STEP_OPTITRACK
                
                step_rmse = np.array([ga.calc_rmse(STEP_ERROR.get_stride_lengths()),
                                      ga.calc_rmse(STEP_ERROR.get_step_lengths()),
                                      ga.calc_rmse(STEP_ERROR.get_step_widths()),
                                      ga.calc_rmse(STEP_ERROR.get_stride_velocities())
                                     ]) * 100

                str_output = '{0:s} ({8:s}): RMSE [cm] ' \
                    '{{MOS: {1:3.2f}, MOSAP: {2:3.2f}, MOSML: {3:3.2f}, StrideL: {4:3.2f}, StepL: {5:3.2f}, StepW: {6:3.2f}, StrideV:{7:3.2f}}}'.format(
                    bag_name, mos_rmse[0], mos_rmse[1], mos_rmse[2], step_rmse[0], step_rmse[1], step_rmse[2], step_rmse[3], c_str)
                
                step_mae = np.array([ga.calc_mae(STEP_ERROR.get_stride_lengths()),
                                     ga.calc_mae(STEP_ERROR.get_step_lengths()),
                                     ga.calc_mae(STEP_ERROR.get_step_widths()),
                                     ga.calc_mae(STEP_ERROR.get_stride_velocities())
                                    ]) * 100
                str_output += ', MAE [cm] {{StrideL: {0:3.2f}, StepL: {1:3.2f}, StepW: {2:3.2f}, StrideV: {3:3.2f}}}'.format(
                    step_mae[0], step_mae[1], step_mae[2], step_mae[3])

                # step_mae_percentage = np.array([ga.calc_mae_percentage(STEP_ERROR.get_stride_lengths(), STEP_OPTITRACK.get_stride_lengths(), 0.3),
                #                                 ga.calc_mae_percentage(STEP_ERROR.get_step_lengths(), STEP_OPTITRACK.get_step_lengths(), 0.3),
                #                                 ga.calc_mae_percentage(STEP_ERROR.get_step_widths(), STEP_OPTITRACK.get_step_widths()),
                #                                 ga.calc_mae_percentage(STEP_ERROR.get_stride_velocities(), STEP_OPTITRACK.get_stride_velocities(), 0.3)
                #                                ]) * 100
                # str_output += ', MAE [%] {{StrideL: {0:3.2f}, StepL: {1:3.2f}, StepW: {2:4.2f}, StrideV: {3:4.2f}}}'.format(
                #     step_mae_percentage[0], step_mae_percentage[1], step_mae_percentage[2], step_mae_percentage[3])

                logging.info(str_output)
                print(str_output)

            if args.export_mat:
                assert(os.path.exists(os.path.dirname(mat_file)))
                sio.savemat(mat_file, {'KINECT': KINECT_OV, 'OPTITRACK': OPTITRACK})
                print('Saved to ' + mat_file)

def trim_time(A, t_ranges):
    idx = np.any(tuple(np.logical_and(A['t'] >= t_min, A['t'] <= t_max) for t_min, t_max in t_ranges), axis=0)
    A.update({k: v[idx] for k, v in A.items()})
    
def run_both(bag_names, bag_path):
    bag_ops.rectify_bag_names(bag_path)

    launch_options = ['record_gait_analytics:=true', 'play_back_rate:=' + str(args.rate)]

    if args.use_raw_foot_pose: launch_options.append('foot_pose_topic_prefix:=raw')
    else: launch_options.append('foot_pose_topic_prefix:=fused')

    if args.transform_to_optitrack: launch_options.append('global_frame:=optitrack')
    else: launch_options.append('global_frame:=fused_odom')

    if args.express_in_raw_odom: launch_options.append('global_frame_fpe:=odom')
    elif args.transform_to_optitrack: launch_options.append('global_frame_fpe:=optitrack')
    else: launch_options.append('global_frame_fpe:=fused_odom')

    launch_options.append('comkf_measurement_scheme:=' + str(args.comkf_measurement_scheme))

    if args.enable_rviz_ga: launch_options.append('enable_rviz:=true rviz_config_file:=slam_rtabmap_ekf_ga.rviz')
    elif args.enable_rviz_3d: launch_options.append('enable_rviz:=true rviz_config_file:=slam_rtabmap_ekf.rviz')
    else: launch_options.append('enable_rviz:=false')

    str_options_joined = ' '.join(launch_options)
    logging.info('[' + str(datetime.datetime.now()) + ']: ' + args.message)
    logging.info('roslaunch options: ' + str_options_joined)
    for bag_name in bag_names:        
        os.system('roslaunch gait_training_robot play_both.launch bag_name:=' + bag_name + ' ' + str_options_joined)
        time.sleep(3)
    bag_ops.rectify_bag_names(bag_path)
    # print(os.listdir(bag_path))


if __name__ == '__main__':
    main()