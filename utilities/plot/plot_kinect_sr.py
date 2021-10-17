#!/usr/bin/python

import os
import glob
import argparse
import numpy as np
import pandas
from scipy.signal import argrelextrema, butter, filtfilt
from collections import namedtuple
import pickle

import rospy
import rosbag
from tf import Transformer
import tf

from visualization_msgs.msg import MarkerArray
import math
import pandas as pd

import matplotlib.pyplot as plt

DESIRED_DIST = 1.5



def extractFromBag(inbag):
    skeleton_msg_list = [msg for _, msg, _ in inbag.read_messages(topics="/body_tracking_data")]

    # Find sbj_id (subject of interest)
    sbj_id, dist_min = 1, 10000
    skeleton_msg1 = skeleton_msg_list[0]
    for i in range(0, len(skeleton_msg1.markers), 32):
        dist = math.hypot(skeleton_msg1.markers[i].pose.position.x, 
                          skeleton_msg1.markers[i].pose.position.z)
        if dist < dist_min:
            dist_min = dist
            sbj_id = skeleton_msg1.markers[i].id // 100

    # Remove markers that do not contain the subject of interest
    def filterMarkers(msg, id):
        i_sbj = -1
        for i in range(0, len(msg.markers), 32):
            if msg.markers[i].id // 100 == sbj_id:
                i_sbj = i
                break
        if i_sbj >= 0:
            msg.markers = msg.markers[i_sbj:i_sbj+32]
        else:
            msg.markers = []
        return msg

    skeleton_msg_list = [filterMarkers(msg, sbj_id) for msg in skeleton_msg_list if msg.markers]
    skeleton_dict = {
        "t":       [msg.markers[0 ].header.stamp.to_sec() for msg in skeleton_msg_list],
        "pelvisA": [msg.markers[0 ].color.a for msg in skeleton_msg_list],
        "pelvisX": [msg.markers[0 ].pose.position.x for msg in skeleton_msg_list],
        "pelvisY": [msg.markers[0 ].pose.position.y for msg in skeleton_msg_list],
        "pelvisZ": [msg.markers[0 ].pose.position.z for msg in skeleton_msg_list],
        "ankleLA": [msg.markers[20].color.a for msg in skeleton_msg_list],
        "ankleLX": [msg.markers[20].pose.position.x for msg in skeleton_msg_list],
        "ankleLY": [msg.markers[20].pose.position.y for msg in skeleton_msg_list],
        "ankleLZ": [msg.markers[20].pose.position.z for msg in skeleton_msg_list],
        "ankleRA": [msg.markers[24].color.a for msg in skeleton_msg_list],
        "ankleRX": [msg.markers[24].pose.position.x for msg in skeleton_msg_list],
        "ankleRY": [msg.markers[24].pose.position.y for msg in skeleton_msg_list],
        "ankleRZ": [msg.markers[24].pose.position.z for msg in skeleton_msg_list],
    }

    return pd.DataFrame(skeleton_dict)


def process(inbag, bag_name):
    pkl_filename = os.path.join(os.path.expanduser("~"), "Documents", "sunny", "kinect_pkl",
        bag_name + ".pkl")
    if os.path.exists(pkl_filename) and not args.force_update:
        with open(pkl_filename, "rb") as pkl_file:
            df_skeleton = pickle.load(pkl_file)
    else:
        df_skeleton = extractFromBag(inbag) # Very time-consuming
        with open(pkl_filename, 'wb') as pkl_file:
            pickle.dump(df_skeleton, pkl_file)

    ts = df_skeleton.t
    df_skeleton = df_skeleton[(ts > ts.iat[0] + 15) & (ts < ts.iat[-1] - 5)]
    ts = df_skeleton.t

    # sr: the ratio of # actual frames to the # expected frames (30 FPS)
    n = round((ts.iat[-1] - ts.iat[0]) * 30)
    m = len(ts)
    sr = (n - m) / n
    print("Kinect frame drop rate: {:.3f}%".format(sr * 100))

    # # sr for each joint
    # plt.close("all")
    # df_kinect_sr = df_skeleton.loc[:, ["pelvisA", "ankleLA", "ankleRA"]].mean() * 2 - 1
    # df_kinect_fr = 1 - df_kinect_sr
    # df_kinect_fr.plot(kind="bar")
    # plt.show()

    if args.plot_scatter:
        min_range = 0.25
        def conic_fun(x, y0): 
            return np.maximum(np.sqrt((x**2 + y0**2) / 3), np.sqrt(min_range**2 - min([y0, min_range])**2))
        _, axs = plt.subplots(1, 3, figsize=(20, 5))
        plotScatter(axs[0], df_skeleton.loc[:, "pelvisX"], df_skeleton.loc[:, "pelvisZ"],
                    lambda x: conic_fun(x, 0.0))
        plotScatter(axs[1], df_skeleton.loc[:, "ankleLX"], df_skeleton.loc[:, "ankleLZ"],
                    lambda x: conic_fun(x, 0.7))
        plotScatter(axs[2], df_skeleton.loc[:, "ankleRX"], df_skeleton.loc[:, "ankleRZ"],
                    lambda x: conic_fun(x, 0.7))

        axs[0].set_title("Pelvis")
        axs[1].set_title("Left Ankle")
        axs[2].set_title("Right Ankle")
        # plt.scatter(df_skeleton.loc[:, "pelvisX"], df_skeleton.loc[:, "pelvisZ"])
        if args.preview: plt.show()
        eps_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "kinect",
            bag_name + "_scatter" + ".eps")
        plt.savefig(eps_filename)

def plotScatter(ax, x, y, fov_fun=None):
    npts = 41
    xlims, ylims = [-1, 1], [1, 3]
    xedges, yedges = np.linspace(xlims[0], xlims[1], npts), np.linspace(ylims[0], ylims[1], npts)
    if fov_fun:
        z_fov = fov_fun(xedges)
        ax.plot(xedges, z_fov, color="lightgray", alpha=0.3)
        ax.fill_between(xedges, z_fov, 2.5, color="lightgray", alpha=0.3)
    
    hist, xedges, yedges = np.histogram2d(x, y, (xedges, yedges))
    xidx = np.clip(np.digitize(x, xedges), 0, hist.shape[0]-1)
    yidx = np.clip(np.digitize(y, yedges), 0, hist.shape[1]-1)
    cmap = hist[xidx, yidx]
    sc = ax.scatter(x, y, c=cmap)
    plt.colorbar(sc, ax=ax)
    ax.set_xlim(xlims)
    ax.set_ylim([0, 2.5])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filenames", nargs='*', help="Bag file path(s). Wildcard is supported.")
    parser.add_argument("-d", "--debug", action="store_true")
    parser.add_argument("-p", "--preview", action="store_true")
    parser.add_argument("-s", "--plot-scatter", action="store_true")
    parser.add_argument("-f", "--force-update", action="store_true", help="Update the pickle files if they existed.")
    args = parser.parse_args()
    if not args.filenames:
        print("Empty list of files.")
    for filename in args.filenames:
        if not os.path.exists(filename):
            print("Cannot find " + filename)
        print(filename)
        with rosbag.Bag(filename, 'r') as inbag:
            process(inbag, os.path.splitext(os.path.basename(filename))[0])