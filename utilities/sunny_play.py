#!/usr/bin/python

import rospkg
import rosbag

import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt

import os
import glob
import argparse
import time
import yaml
import csv

import bag_ops
import ga
from bag_ops import trim_time

import pickle


def process(filename, launch_options, png_filename):
    in_bag_path = os.path.join(os.getcwd(), filename)
    bag_name = os.path.basename(filename)
    # print(in_bag_path)


    launch_options.append("bag_path1:=" + in_bag_path)
    launch_options.append("bag_name:=" + bag_name)

    if not args.skip_bag_gen:
        os.system("roslaunch gait_training_robot sunny_play_local.launch " + ' '.join(launch_options))
        time.sleep(3)
        bag_ops.rectify_bag_names(os.path.join(gta_path, "bags", "sunny", "ga"))

    bag_name = os.path.splitext(bag_name)[0]
    out_bag_path = os.path.join(bag_path, "sunny", "ga", bag_name[:7] + '.bag')

    if not os.path.exists(out_bag_path):
        print(" ")
    else:
        # Read bag_info.yaml
        bag_info = {}
        with open(os.path.join(gta_path, 'bags', 'sunny', 'bag_info.yaml'), 'r') as stream:
            try:
                bag_info = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        with rosbag.Bag(out_bag_path, 'r') as inbag:
            # Compare MoS
            KINECT = ga.get_mos_vec_dict(inbag, '/gait_analyzer/estimate/mos_vec')
            KINECT = ga.merge(KINECT, ga.get_gait_state_dict(inbag, '/gait_analyzer/gait_state'))

            if not len(KINECT['t']):
                print(" ")
                return

            t_min = KINECT['t'][0]
            t_max = KINECT['t'][-1]
            if bag_name in bag_info.keys() and 'time_range' in bag_info[bag_name].keys():
                t_min = bag_info[bag_name]['time_range'][0]
                t_max = bag_info[bag_name]['time_range'][1]
                trim_time(KINECT, [(t_min, t_max)])

            t_min = max(t_max - 90, t_min)
            
            stance_intervals = ga.get_stance_intervals(inbag, '/gait_analyzer/gait_state', t_min, t_max)
            STEP_KINECT    = ga.StepData(inbag, '/foot_pose_estimator/fused_pose_', stance_intervals)

            for mode in ["mean_sd", "cv"]: #["mean_sd", "cv"]: ["cv", ]:
                str_output = ''.join((
                    ga.print_stats("strideL", STEP_KINECT.get_stride_lengths(), "cm", mode=mode),
                    ga.print_stats("stepL", STEP_KINECT.get_step_lengths(), "cm", mode=mode),
                    ga.print_stats("stepW", STEP_KINECT.get_step_widths(), "cm", mode=mode),
                    ga.print_stats("strideV", STEP_KINECT.get_stride_velocities(), "cm/s", mode=mode),
                    ga.print_stats("strideT", STEP_KINECT.get_stride_times(), "ms", mode=mode),
                ))
                print((bag_name + ":").ljust(10) + str_output)

            # Check sport_sole packets
            if args.show_frequency:
                msgs_insole = [msg for _, msg, _ in inbag.read_messages("/sport_sole_publisher/sport_sole")]
                ts_insole = np.array([msg.header.stamp.to_sec() for msg in msgs_insole])
                ts_insole -= STEP_KINECT.t0_zeno
                plt.figure(figsize=(20, 6))
                plt.plot(ts_insole, np.diff(ts_insole, prepend=ts_insole[0]))
                plt.plot([ts_insole[0], ts_insole[-1]], np.ones(2) * 0.1, 'g--')
                plt.xlabel('t [s]')
                plt.ylabel('Gap [s]')
                plt.savefig(png_filename)
                print("Saved to " + png_filename)
                # plt.show()
                return

            if args.export_mat:
                mat_file = os.path.join('/home/ral2020/projects/gta_data2/sunny/simple', bag_name[:7] + '.mat')
                assert(os.path.exists(os.path.dirname(mat_file)))
                sio.savemat(mat_file, {'KINECT': KINECT})
                print('Saved to ' + mat_file)
            
            if args.export_csv:
                csv_file_path = os.path.join(os.getenv("HOME"), 'Documents', 'gta_data')
                assert(os.path.exists(csv_file_path))
                for csv_type in ['stride', 'step']:
                    csv_filename = os.path.join(csv_file_path, bag_name[:7] + "_" + csv_type + ".csv")
                    rows = getattr(STEP_KINECT, csv_type + "_table_sorted")
                    with open(csv_filename, 'w') as csv_file:
                        writer = csv.writer(csv_file)
                        writer.writerow(rows[0]._fields)
                        writer.writerows(rows)
                        print('Saved to ' + csv_filename)
                    





if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Play a bag file:")
    parser.add_argument("filenames", nargs='*', help="Bag file path(s). Wildcard is supported.")
    parser.add_argument('-u', '--user-select',               action='store_true')
    parser.add_argument('-r', '--record-all',                action='store_true')
    parser.add_argument('-p', '--export-mat',                action='store_true')
    parser.add_argument('-c', '--export-csv',                action='store_true')
    parser.add_argument('-i', '--export-pickle',             action='store_true')
    parser.add_argument('-s', '--skip-bag-gen',              action='store_true')
    parser.add_argument('-e', '--enable-debug-log',          action='store_true')
    parser.add_argument('-f', '--show-frequency',            action='store_true')
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

    if args.enable_debug_log:
        launch_options.append('enable_debug_log:=true')
    else:
        launch_options.append('enable_debug_log:=false')

    # bag_names = [os.path.basename(f) for f in sorted(glob.glob(bag_path + '/*.bag'))]
    # bag_indices = range(0, len(bag_names))
    # print('\n'.join(['[{0: <5}] '.format(i) + n for i, n in zip(bag_indices, bag_names)]))
    # while True:
    #     bag_index_sel = input('Please enter the index of bag to be selected:')
    #     if bag_index_sel >= 0 and bag_index_sel < len(bag_names):
    #         break
    #     print('Input must be within [0, ' + str(len(bag_names)) + ')')

    # bag_names = ['data' + s + '.bag' for s in [str(i).rjust(3, '0') for i in range(424, 464)]]
    bag_names = ['data' + s + '.bag' for s in [str(i).rjust(3, '0') for i in range(424, 425)]]

    for filename in args.filenames:
        png_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "SportSoleGaps", 
            os.path.splitext(os.path.basename(filename))[0] + ".png")
        try:
            process(filename, launch_options, png_filename)
        except Exception as e:
            print(e)
            print("Failed to process " + filename)