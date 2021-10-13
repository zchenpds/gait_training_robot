#! /usr/bin/env python

"""
Plot the following topics in a bag file

Topics: 
    "/foot_pose_estimator/fused_pose_l",
    "/foot_pose_estimator/fused_pose_r",
    "/foot_pose_estimator/raw_pose_l",
    "/foot_pose_estimator/raw_pose_r",
    "/optitrack/foot_pose_l",
    "/optitrack/foot_pose_r"
of type [geometry_msgs/PoseWithCovarianceStamped]
"""

import math
import argparse
import os
import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.optimize import fmin
from matplotlib import rc
#rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
rc('font',**{'family':'serif','serif':['Times New Roman'], 'size': 11})

import rospkg
import rosbag
from tf.transformations import quaternion_from_matrix, rotation_matrix

from bag_ops import trim_time, MessageExtractor, Aligner


def process_inbag(inbag, args):
    me = MessageExtractor(inbag)

    fsd = [me.extractPoseWithCovarianceStamped("/foot_pose_estimator/fused_pose_l", "Fused"),
           me.extractPoseWithCovarianceStamped("/foot_pose_estimator/fused_pose_r", "Fused")]

    raw = [me.extractPoseWithCovarianceStamped("/foot_pose_estimator/raw_pose_l", "Raw"),
           me.extractPoseWithCovarianceStamped("/foot_pose_estimator/raw_pose_r", "Raw")]

    ref = [me.extractPoseWithCovarianceStamped("/optitrack/foot_pose_l", "Refererence"),
           me.extractPoseWithCovarianceStamped("/optitrack/foot_pose_r", "Refererence")]

    pd_list = [fsd, raw, ref]
    plot_style_list = [{"ls": "-", "color": "blue"}, {"ls": "--", "color": "r"}, {"ls": "-", "color": "black"}]

    # Find the min and max of all the poses
    t_min = max([pd[lr]["t"][0] for lr in [0, 1] for pd in pd_list])
    t_max = min([pd[lr]["t"][-1] for lr in [0, 1] for pd in pd_list])

    # Merge time range provided by the user
    t_max = min([t_max, t_min + args.time_range[1]])
    t_min += args.time_range[0]

    # Trim the poses to the time range
    for pd in pd_list:
        for lr in [0, 1]:
            trim_time(pd[lr], [(t_min, t_max)])

    # Align the poses
    tf = Aligner(fsd[0], ref[0])
    for lr in [0, 1]:
        ref[lr] = tf.transform(ref[lr])

    # Determine whether to plot left or right or both
    if (args.left_only): lr_list = [0]
    elif (args.right_only): lr_list = [1]
    else: lr_list = [0, 1]

    # Plot
    xy_list = [0, 1]
    _, axs = plt.subplots(len(xy_list), len(lr_list), sharex=True, figsize=(6, 6))
    lr_str = ["Left Foot Position", "Right Foot Position"]
    for lr in lr_list:
        xy_str = ["x", "y"]
        for xy in xy_list:
            ax = axs[xy, lr] if len(lr_list) > 1 else axs[xy]
            for pd, plot_style in zip(pd_list, plot_style_list):
                ax.plot(pd[lr]["t"] - t_min + args.time_range[0], 
                    pd[lr]["xyz"][:, xy], 
                    label=pd[lr]["legend"], linewidth=args.linewidth, **plot_style)
            ax.legend()
            ax.set_xlabel("t [s]")
            ax.set_ylabel("{0:s} [m]".format(xy_str[xy]))
            if xy == 0: ax.set_title(lr_str[lr])
    plt.tight_layout()

    # Save
    if args.save:
        # eps_path = os.path.join(os.path.expanduser("~"), "Pictures",
        #     "Foot pose data" + str(args.trial_id).rjust(3, '0') +
        #     " [{0:3.1f}:{1:3.1f}]".format(args.time_range[0], args.time_range[1]) +
        #     datetime.datetime.now().strftime(" %Y-%m-%d %H-%M-%S.eps"))
        eps_path = os.path.join(os.path.expanduser("~"), "TMRB2021/results/foot_position_time_course.eps")
        plt.savefig(eps_path, format='eps')

    plt.show()
    


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Plot foot pose estimates.")
    parser.add_argument('trial_id', type=int)
    parser.add_argument('-tr', '--time-range', type=float, nargs=2, default=[0.0, 100.0])
    parser.add_argument('-lw', '--linewidth', type=float, default=0.5)
    parser.add_argument('-l', '--left-only', action='store_true')
    parser.add_argument('-r', '--right-only', action='store_true')
    parser.add_argument('-s', '--save', action='store_true')
    args = parser.parse_args()

    print("time_range: " + "{0:3.2f}, {1:3.2f}".format(args.time_range[0], args.time_range[1]))
    
    # Get input bag path
    rp = rospkg.RosPack()
    gta_path = rp.get_path('gait_training_robot')
    bag_path = os.path.join(gta_path, 'bags/optitrack/ga/')
    bag_name = 'data' + str(args.trial_id).rjust(3, '0')
    bag_filename = os.path.join(bag_path, bag_name + '.bag')
    if not os.path.exists(bag_filename):
        sys.exit("Nonexistent bag: " + bag_filename)

    # Open and process the bag
    with rosbag.Bag(bag_filename, 'r') as inbag:
        process_inbag(inbag, args)