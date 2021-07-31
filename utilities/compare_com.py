#! /usr/bin/env python

"""
Plot the following topics in a bag file

Topics:
    "/gait_analyzer_optitrack/measurement/com",
    "/gait_analyzer_optitrack/estimate/com",
    "/gait_analyzer_optitrack/cop",
    "/gait_analyzer/measurement/com",
    "/gait_analyzer/estimate/com",
    "/gait_analyzer/cop".
of type [geometry_msgs/PointStamped]
"""

import math
import argparse
import os
import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

import rospkg
import rosbag
from tf.transformations import quaternion_from_matrix, rotation_matrix

from bag_ops import trim_time, MessageExtractor, Aligner


def process_inbag(inbag, args):
    me = MessageExtractor(inbag)

    # CoM
    fsd = me.extractPointStamped("/gait_analyzer/estimate/com", "Fused CoM")
    raw = me.extractPointStamped("/gait_analyzer/measurement/com", "Raw CoM")
    ref = me.extractPointStamped("/gait_analyzer_optitrack/measurement/com", "Ref. CoM")

    # CoMv
    comv_fsd = me.extractVector3Stamped("/gait_analyzer/estimate/comv", "Fused CoMv")
    comv_raw = me.extractVector3Stamped("/gait_analyzer/measurement/comv", "Raw CoMv")
    comv_ref = me.extractVector3Stamped("/gait_analyzer_optitrack/estimate/comv", "Ref. CoMv")
    comv_list = [comv_fsd, comv_raw, comv_ref]
    
    # CoP
    cop = me.extractPointStamped("/gait_analyzer/cop", "CoP")

    # Footprint
    footprint = [me.extractFootprint("/gait_analyzer/footprint_l", "Left Footprint"),
                 me.extractFootprint("/gait_analyzer/footprint_r", "Right Footprint")]


    pd_list = [fsd, raw, ref, cop]

    # Find the min and max of all the time series
    t_min = max([pd["t"][0] for pd in pd_list])
    t_max = min([pd["t"][-1] for pd in pd_list])

    # Merge time range provided by the user
    t_max = min([t_max, t_min + args.time_range[1]])
    t_min += args.time_range[0]

    # Trim the positions to the time range
    for pd in pd_list + [footprint[0], footprint[1]] + comv_list:
        trim_time(pd, [(t_min, t_max)])

    # Align the trajectories
    Aligner.rotate(ref, -math.pi/2)
    tf = Aligner(ref, fsd)
    for pd in [fsd, raw, cop, footprint[0], footprint[1]]:
        pd = tf.transform(pd)

    Aligner.rotate(comv_ref, -math.pi/2)
    for pd in [comv_fsd, comv_raw]:
        pd = tf.rotate(pd, tf.theta)

    plot_com(pd_list, footprint, args)
    plot_comv(comv_list, t_min, args)

def plot_com(pd_list, footprint, args):
    # Plot CoM and CoP
    plt.figure()
    for pd in pd_list:
        plt.plot(pd["xyz"][:, 0], pd["xyz"][:, 1], 
            label=pd["legend"], linewidth=args.linewidth)
    
    # Plot footprints
    for lr in [0, 1]:
        t_prev = None
        for k, t in enumerate(footprint[lr]["t"]):
            if t_prev: 
                if t - t_prev < 0.3: 
                    continue
            plt.gca().add_patch(Polygon(footprint[lr]["pts"][k, :, 0:2]))
            t_prev = t

    plt.fill()
    plt.legend()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.tight_layout()

    # Save
    if args.save:
        eps_path = os.path.join(os.path.expanduser("~"), "Pictures",
            "CoM data" + str(args.trial_id).rjust(3, '0') + 
            " [{0:3.1f}:{1:3.1f}]".format(args.time_range[0], args.time_range[1]) +
            datetime.datetime.now().strftime(" %Y-%m-%d %H-%M-%S.eps"))
        plt.savefig(eps_path, format='eps')

    plt.show()

def plot_comv(comv_list, t_min, args):
    # Plot CoMv
    xy_list = [0, 1]
    xy_str = ["x", "y"]
    _, axs = plt.subplots(len(xy_list), 1)
    for xy in xy_list:
        ax = axs[xy]
        for pd in comv_list:
            ax.plot(pd["t"][:] - t_min + args.time_range[0], 
                pd["xyz"][:, xy], 
                label=pd["legend"], linewidth=args.linewidth)

        ax.legend()
        ax.set_xlabel("t [s]")
        ax.set_ylabel("{0:s} [m/s]".format(xy_str[xy]))
    plt.tight_layout()

    # Save
    if args.save:
        eps_path = os.path.join(os.path.expanduser("~"), "Pictures",
            "CoMv data" + str(args.trial_id).rjust(3, '0') + 
            " [{0:3.1f}:{1:3.1f}]".format(args.time_range[0], args.time_range[1]) +
            datetime.datetime.now().strftime(" %Y-%m-%d %H-%M-%S.eps"))
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