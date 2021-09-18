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

from collections import namedtuple
dataInfoNT = namedtuple("dataInfoNT", ["sbj", "session"])
def constructDataInfo(sbj, session):
    sbj = "SBJ" + sbj
    return (sbj, session.upper())
data_info_dict = {
    424: dataInfoNT(*constructDataInfo("010", "d")),
    425: dataInfoNT(*constructDataInfo("010", "e")),
    426: dataInfoNT(*constructDataInfo("010", "f")),
    427: dataInfoNT(*constructDataInfo("010", "g")),
    428: dataInfoNT(*constructDataInfo("011", "d")),
    429: dataInfoNT(*constructDataInfo("011", "e")),
    430: dataInfoNT(*constructDataInfo("011", "f")),
    431: dataInfoNT(*constructDataInfo("011", "g")),
    432: dataInfoNT(*constructDataInfo("012", "d")),
    433: dataInfoNT(*constructDataInfo("012", "e")),
    434: dataInfoNT(*constructDataInfo("012", "f")),
    435: dataInfoNT(*constructDataInfo("012", "g")),
    436: dataInfoNT(*constructDataInfo("013", "d")),
    437: dataInfoNT(*constructDataInfo("013", "e")),
    438: dataInfoNT(*constructDataInfo("013", "f")),
    439: dataInfoNT(*constructDataInfo("013", "g")),
    444: dataInfoNT(*constructDataInfo("015", "d")),
    445: dataInfoNT(*constructDataInfo("015", "e")),
    446: dataInfoNT(*constructDataInfo("015", "f")),
    447: dataInfoNT(*constructDataInfo("015", "g")),
    448: dataInfoNT(*constructDataInfo("016", "d")),
    449: dataInfoNT(*constructDataInfo("016", "e")),
    450: dataInfoNT(*constructDataInfo("016", "f")),
    451: dataInfoNT(*constructDataInfo("016", "g")),
    456: dataInfoNT(*constructDataInfo("018", "d")),
    457: dataInfoNT(*constructDataInfo("018", "e")),
    458: dataInfoNT(*constructDataInfo("018", "f")),
    459: dataInfoNT(*constructDataInfo("018", "g")),
    460: dataInfoNT(*constructDataInfo("019", "d")),
    461: dataInfoNT(*constructDataInfo("019", "e")),
    462: dataInfoNT(*constructDataInfo("019", "f")),
    463: dataInfoNT(*constructDataInfo("019", "g")),
    500: dataInfoNT(*constructDataInfo("020", "F")),
    501: dataInfoNT(*constructDataInfo("020", "G")),
    502: dataInfoNT(*constructDataInfo("020", "D")),
    503: dataInfoNT(*constructDataInfo("020", "E")),
    504: dataInfoNT(*constructDataInfo("022", "F")),
    505: dataInfoNT(*constructDataInfo("022", "G")),
    506: dataInfoNT(*constructDataInfo("022", "D")),
    507: dataInfoNT(*constructDataInfo("022", "E")),
    508: dataInfoNT(*constructDataInfo("021", "F")),
    509: dataInfoNT(*constructDataInfo("021", "G")),
    510: dataInfoNT(*constructDataInfo("021", "D")),
    511: dataInfoNT(*constructDataInfo("021", "E")),
    512: dataInfoNT(*constructDataInfo("023", "F")),
    513: dataInfoNT(*constructDataInfo("023", "G")),
    514: dataInfoNT(*constructDataInfo("023", "D")),
    515: dataInfoNT(*constructDataInfo("023", "E")),
    516: dataInfoNT(*constructDataInfo("024", "F")),
    517: dataInfoNT(*constructDataInfo("024", "G")),
    518: dataInfoNT(*constructDataInfo("024", "D")), # No zeno data available
    519: dataInfoNT(*constructDataInfo("024", "E")),
    520: dataInfoNT(*constructDataInfo("025", "F")),
    521: dataInfoNT(*constructDataInfo("025", "G")),
    522: dataInfoNT(*constructDataInfo("025", "D")),
    523: dataInfoNT(*constructDataInfo("025", "E")),
    524: dataInfoNT(*constructDataInfo("026", "F")),
    525: dataInfoNT(*constructDataInfo("026", "G")),
    526: dataInfoNT(*constructDataInfo("026", "D")),
    527: dataInfoNT(*constructDataInfo("026", "E")),
    528: dataInfoNT(*constructDataInfo("027", "F")),
    529: dataInfoNT(*constructDataInfo("027", "G")),
    530: dataInfoNT(*constructDataInfo("027", "D")),
    531: dataInfoNT(*constructDataInfo("027", "E")),
    532: dataInfoNT(*constructDataInfo("028", "F")),
    533: dataInfoNT(*constructDataInfo("028", "G")),
    534: dataInfoNT(*constructDataInfo("028", "D")),
    535: dataInfoNT(*constructDataInfo("028", "E")),
    536: dataInfoNT(*constructDataInfo("029", "F")),
    537: dataInfoNT(*constructDataInfo("029", "G")),
    538: dataInfoNT(*constructDataInfo("029", "D")),
    539: dataInfoNT(*constructDataInfo("029", "E")),
    540: dataInfoNT(*constructDataInfo("030", "D")),
    541: dataInfoNT(*constructDataInfo("030", "E")),
    542: dataInfoNT(*constructDataInfo("030", "F")),
    543: dataInfoNT(*constructDataInfo("030", "G")),
    544: dataInfoNT(*constructDataInfo("031", "D")),
    545: dataInfoNT(*constructDataInfo("031", "E")),
    546: dataInfoNT(*constructDataInfo("031", "F")),
    547: dataInfoNT(*constructDataInfo("031", "G")),
    548: dataInfoNT(*constructDataInfo("032", "D")),
    549: dataInfoNT(*constructDataInfo("032", "E")),
    550: dataInfoNT(*constructDataInfo("032", "F")),
    551: dataInfoNT(*constructDataInfo("032", "G")),
    552: dataInfoNT(*constructDataInfo("033", "F")),
    553: dataInfoNT(*constructDataInfo("033", "G")),
    554: dataInfoNT(*constructDataInfo("033", "D")),
    555: dataInfoNT(*constructDataInfo("033", "E")),
}

def process(filename, launch_options, png_filename, trial_id):
    in_bag_path = os.path.join(os.getcwd(), filename)
    bag_name = os.path.basename(filename)
    # print(in_bag_path)
    sbj_session_name = data_info_dict[trial_id].sbj + data_info_dict[trial_id].session


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

            percentage_discarded = 0.0

            # Data minipulation
            if 1:
                if bag_name in bag_info.keys() and 'time_range' in bag_info[bag_name].keys():
                    t_min = bag_info[bag_name]['time_range'][0]
                    t_max = bag_info[bag_name]['time_range'][1]
                    percentage_discarded = 1.0 - trim_time(KINECT, [(t_min, t_max)])

                t_min = max(t_max - 90, t_min)
            
            print("{:s} percentage discarded: {:.2f}%".format(bag_name, percentage_discarded * 100))
            
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

            if args.show_time_range:
                png_filename_time_range = png_filename.replace("SportSoleGaps", "TimeRange")
                msgs_mos_vec = [msg for _, msg, _ in inbag.read_messages("/gait_analyzer/estimate/mos_vec")]
                ts  = np.array([msg.header.stamp.to_sec() for msg in msgs_mos_vec])
                mos = np.array([msg.vector.x for msg in msgs_mos_vec])
                ts -= STEP_KINECT.t0_zeno
                plt.figure(figsize=(50, 1.6))
                plt.plot(ts, mos)
                y_range = [-1.2, 0.2]
                plt.fill_between(
                    [t_min - STEP_KINECT.t0_zeno, t_max - STEP_KINECT.t0_zeno], 
                    y_range[0], y_range[1], color="green", alpha=0.5)
                plt.ylim(y_range)
                plt.xlim([ts[0], ts[0] + 300])
                plt.xlabel('Zeno Time [s]')
                plt.ylabel('MoS [m]')
                plt.title(sbj_session_name)
                plt.savefig(png_filename_time_range)
                print("Saved to " + png_filename_time_range)
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

            if args.export_pickle:
                pickle_filename = os.path.join("/home/ral2020/Documents/sunny/robot_full", bag_name[:7] + ".pkl")
                with open(pickle_filename, 'wb') as pickle_file:
                    pickle.dump(STEP_KINECT, pickle_file)
                    





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
    parser.add_argument('-t', '--show-time-range',           action='store_true')
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
        trial_name = os.path.splitext(os.path.basename(filename))[0]
        trial_id = int(trial_name[4:])
        png_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "SportSoleGaps", 
            trial_name + ".png")
        try:
            process(filename, launch_options, png_filename, trial_id)
        except Exception as e:
            print(e)
            print("Failed to process " + filename)