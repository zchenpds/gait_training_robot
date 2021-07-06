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

import rospkg
import rosbag
from tf.transformations import quaternion_from_matrix, rotation_matrix

from bag_ops import trim_time, extractPointStamped, Aligner


def process_inbag(inbag, args):
    fsd = extractPointStamped(inbag, "/gait_analyzer/estimate/com", "Fused CoM")

    raw = extractPointStamped(inbag, "/gait_analyzer/measurement/com", "Raw CoM")

    ref = extractPointStamped(inbag, "/gait_analyzer_optitrack/measurement/com", "Ref. CoM")

    cop = extractPointStamped(inbag, "/gait_analyzer/cop", "CoP")


    pd_list = [fsd, raw, ref, cop]

    # Find the min and max of all the time series
    t_min = max([pd["t"][0] for pd in pd_list])
    t_max = min([pd["t"][-1] for pd in pd_list])

    # Merge time range provided by the user
    t_max = min([t_max, t_min + args.time_range[1]])
    t_min += args.time_range[0]

    # Trim the positions to the time range
    for pd in pd_list:
        trim_time(pd, [(t_min, t_max)])

    # Align the trajectories
    Aligner.rotate(ref, -math.pi/2)
    tf = Aligner(ref, fsd)
    for pd in [fsd, raw, cop]:
        pd = tf.transform(pd)

    # Plot
    plt.figure()
    for pd in pd_list:
        plt.plot(pd["xyz"][:, 0], pd["xyz"][:, 1], 
            label=pd["legend"], linewidth=args.linewidth)
    plt.legend()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.gca().set_aspect('equal', adjustable='box')

    # Save
    if args.save:
        eps_path = os.path.join(os.path.expanduser("~"), "Pictures",
            "Foot pose data" + str(args.trial_id).rjust(3, '0') + 
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