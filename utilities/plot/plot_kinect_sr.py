#!/usr/bin/python

import os
from collections import namedtuple
import argparse
import numpy as np
from functools import reduce
from collections import namedtuple
import pickle

import rosbag

from visualization_msgs.msg import MarkerArray
import math
import pandas as pd

import matplotlib.pyplot as plt

DESIRED_DIST = 1.5


WS_PATH = "/home/ral2020/Documents/sunny"
class BoxPlotter:
    dataInfoNT = namedtuple("dataInfoNT", ["sbj", "session"])
    session_list = ["D", "E", "F", "G"]
    vibration_conditions = ["VOFF", "VON"] # rows (index)
    vibration_dict = {"D": "VOFF", "E": "VOFF", "F": "VON", "G": "VON"}
    cognitive_conditions = ["COFF", "CON"] # columns
    cognitive_dict = {"D": "COFF", "E": "CON", "F": "COFF", "G": "CON"}
    sbj_list = []
    param_list = ["FrameDropRate", "NoObservationRate"]
    meanstd_list = ["mean", "std", "se"]

    @classmethod
    def constructDataInfo(cls, sbj, session):
        sbj = "SBJ" + sbj
        return (sbj, session.upper())

    def __init__(self):
        self.data_info_dict = {
            424: self.dataInfoNT(*BoxPlotter.constructDataInfo("010", "d")),
            425: self.dataInfoNT(*BoxPlotter.constructDataInfo("010", "e")),
            426: self.dataInfoNT(*BoxPlotter.constructDataInfo("010", "f")),
            427: self.dataInfoNT(*BoxPlotter.constructDataInfo("010", "g")),
            428: self.dataInfoNT(*BoxPlotter.constructDataInfo("011", "d")),
            429: self.dataInfoNT(*BoxPlotter.constructDataInfo("011", "e")),
            430: self.dataInfoNT(*BoxPlotter.constructDataInfo("011", "f")),
            431: self.dataInfoNT(*BoxPlotter.constructDataInfo("011", "g")),
            432: self.dataInfoNT(*BoxPlotter.constructDataInfo("012", "d")),
            433: self.dataInfoNT(*BoxPlotter.constructDataInfo("012", "e")),
            434: self.dataInfoNT(*BoxPlotter.constructDataInfo("012", "f")),
            435: self.dataInfoNT(*BoxPlotter.constructDataInfo("012", "g")),
            436: self.dataInfoNT(*BoxPlotter.constructDataInfo("013", "d")),
            437: self.dataInfoNT(*BoxPlotter.constructDataInfo("013", "e")),
            438: self.dataInfoNT(*BoxPlotter.constructDataInfo("013", "f")),
            439: self.dataInfoNT(*BoxPlotter.constructDataInfo("013", "g")),
            440: self.dataInfoNT(*BoxPlotter.constructDataInfo("014", "d")),
            441: self.dataInfoNT(*BoxPlotter.constructDataInfo("014", "e")),
            442: self.dataInfoNT(*BoxPlotter.constructDataInfo("014", "f")),
            443: self.dataInfoNT(*BoxPlotter.constructDataInfo("014", "g")),
            444: self.dataInfoNT(*BoxPlotter.constructDataInfo("015", "d")),
            445: self.dataInfoNT(*BoxPlotter.constructDataInfo("015", "e")),
            446: self.dataInfoNT(*BoxPlotter.constructDataInfo("015", "f")),
            447: self.dataInfoNT(*BoxPlotter.constructDataInfo("015", "g")),
            448: self.dataInfoNT(*BoxPlotter.constructDataInfo("016", "d")),
            449: self.dataInfoNT(*BoxPlotter.constructDataInfo("016", "e")),
            450: self.dataInfoNT(*BoxPlotter.constructDataInfo("016", "f")),
            451: self.dataInfoNT(*BoxPlotter.constructDataInfo("016", "g")),
            452: self.dataInfoNT(*BoxPlotter.constructDataInfo("017", "d")),
            453: self.dataInfoNT(*BoxPlotter.constructDataInfo("017", "e")),
            454: self.dataInfoNT(*BoxPlotter.constructDataInfo("017", "f")),
            455: self.dataInfoNT(*BoxPlotter.constructDataInfo("017", "g")),
            456: self.dataInfoNT(*BoxPlotter.constructDataInfo("018", "d")),
            457: self.dataInfoNT(*BoxPlotter.constructDataInfo("018", "e")),
            458: self.dataInfoNT(*BoxPlotter.constructDataInfo("018", "f")),
            459: self.dataInfoNT(*BoxPlotter.constructDataInfo("018", "g")),
            460: self.dataInfoNT(*BoxPlotter.constructDataInfo("019", "d")),
            461: self.dataInfoNT(*BoxPlotter.constructDataInfo("019", "e")),
            462: self.dataInfoNT(*BoxPlotter.constructDataInfo("019", "f")),
            463: self.dataInfoNT(*BoxPlotter.constructDataInfo("019", "g")),
            500: self.dataInfoNT(*BoxPlotter.constructDataInfo("020", "F")),
            501: self.dataInfoNT(*BoxPlotter.constructDataInfo("020", "G")),
            502: self.dataInfoNT(*BoxPlotter.constructDataInfo("020", "D")),
            503: self.dataInfoNT(*BoxPlotter.constructDataInfo("020", "E")),
            504: self.dataInfoNT(*BoxPlotter.constructDataInfo("022", "F")),
            505: self.dataInfoNT(*BoxPlotter.constructDataInfo("022", "G")),
            506: self.dataInfoNT(*BoxPlotter.constructDataInfo("022", "D")),
            507: self.dataInfoNT(*BoxPlotter.constructDataInfo("022", "E")),
            508: self.dataInfoNT(*BoxPlotter.constructDataInfo("021", "F")),
            509: self.dataInfoNT(*BoxPlotter.constructDataInfo("021", "G")),
            510: self.dataInfoNT(*BoxPlotter.constructDataInfo("021", "D")),
            511: self.dataInfoNT(*BoxPlotter.constructDataInfo("021", "E")),
            512: self.dataInfoNT(*BoxPlotter.constructDataInfo("023", "F")),
            513: self.dataInfoNT(*BoxPlotter.constructDataInfo("023", "G")),
            514: self.dataInfoNT(*BoxPlotter.constructDataInfo("023", "D")),
            515: self.dataInfoNT(*BoxPlotter.constructDataInfo("023", "E")),
            516: self.dataInfoNT(*BoxPlotter.constructDataInfo("024", "F")),
            517: self.dataInfoNT(*BoxPlotter.constructDataInfo("024", "G")),
            518: self.dataInfoNT(*BoxPlotter.constructDataInfo("024", "D")), # No zeno data available
            519: self.dataInfoNT(*BoxPlotter.constructDataInfo("024", "E")),
            520: self.dataInfoNT(*BoxPlotter.constructDataInfo("025", "F")),
            521: self.dataInfoNT(*BoxPlotter.constructDataInfo("025", "G")),
            522: self.dataInfoNT(*BoxPlotter.constructDataInfo("025", "D")),
            523: self.dataInfoNT(*BoxPlotter.constructDataInfo("025", "E")),
            524: self.dataInfoNT(*BoxPlotter.constructDataInfo("026", "F")),
            525: self.dataInfoNT(*BoxPlotter.constructDataInfo("026", "G")),
            526: self.dataInfoNT(*BoxPlotter.constructDataInfo("026", "D")),
            527: self.dataInfoNT(*BoxPlotter.constructDataInfo("026", "E")),
            528: self.dataInfoNT(*BoxPlotter.constructDataInfo("027", "F")),
            529: self.dataInfoNT(*BoxPlotter.constructDataInfo("027", "G")),
            530: self.dataInfoNT(*BoxPlotter.constructDataInfo("027", "D")),
            531: self.dataInfoNT(*BoxPlotter.constructDataInfo("027", "E")),
            532: self.dataInfoNT(*BoxPlotter.constructDataInfo("028", "F")),
            533: self.dataInfoNT(*BoxPlotter.constructDataInfo("028", "G")),
            534: self.dataInfoNT(*BoxPlotter.constructDataInfo("028", "D")),
            535: self.dataInfoNT(*BoxPlotter.constructDataInfo("028", "E")),
            536: self.dataInfoNT(*BoxPlotter.constructDataInfo("029", "F")),
            537: self.dataInfoNT(*BoxPlotter.constructDataInfo("029", "G")),
            538: self.dataInfoNT(*BoxPlotter.constructDataInfo("029", "D")),
            539: self.dataInfoNT(*BoxPlotter.constructDataInfo("029", "E")),
            540: self.dataInfoNT(*BoxPlotter.constructDataInfo("030", "D")),
            541: self.dataInfoNT(*BoxPlotter.constructDataInfo("030", "E")),
            542: self.dataInfoNT(*BoxPlotter.constructDataInfo("030", "F")),
            543: self.dataInfoNT(*BoxPlotter.constructDataInfo("030", "G")),
            544: self.dataInfoNT(*BoxPlotter.constructDataInfo("031", "D")),
            545: self.dataInfoNT(*BoxPlotter.constructDataInfo("031", "E")),
            546: self.dataInfoNT(*BoxPlotter.constructDataInfo("031", "F")),
            547: self.dataInfoNT(*BoxPlotter.constructDataInfo("031", "G")),
            548: self.dataInfoNT(*BoxPlotter.constructDataInfo("032", "D")),
            549: self.dataInfoNT(*BoxPlotter.constructDataInfo("032", "E")),
            550: self.dataInfoNT(*BoxPlotter.constructDataInfo("032", "F")),
            551: self.dataInfoNT(*BoxPlotter.constructDataInfo("032", "G")),
            552: self.dataInfoNT(*BoxPlotter.constructDataInfo("033", "F")),
            553: self.dataInfoNT(*BoxPlotter.constructDataInfo("033", "G")),
            554: self.dataInfoNT(*BoxPlotter.constructDataInfo("033", "D")),
            555: self.dataInfoNT(*BoxPlotter.constructDataInfo("033", "E")),
        }
        self.df_skeletons = []
        self.df_agg = pd.DataFrame("", index=sorted(self.data_info_dict.keys()),columns=["sbj", "session"])
        # Insert subject info columns
        for trial_id in self.data_info_dict.keys():
            self.df_agg.at[trial_id, "sbj"]     = self.data_info_dict[trial_id].sbj
            self.df_agg.at[trial_id, "session"] = self.data_info_dict[trial_id].session


    def process_trial(self, trial_id):
        pkl_filename = os.path.join(WS_PATH, "kinect_pkl", "data" + str(trial_id).rjust(3, '0') + ".pkl")
        with open(pkl_filename, "rb") as pkl_file:
            df_skeleton = pickle.load(pkl_file)
            # sr: the ratio of # actual frames to the # expected frames (30 FPS)
            ts = df_skeleton.t
            n = round((ts.iat[-1] - ts.iat[0]) * 30)
            m = len(ts)
            frame_drop_rate = (n - m) / n
            # print("Kinect frame drop rate: {:.3f}%".format(frame_drop_rate * 100))

            # Populate self.df_agg by trial_id
            self.df_agg.at[trial_id, self.param_list[0]] = frame_drop_rate
            for joint_alpha_field in ["pelvisA", "ankleLA", "ankleRA"]:
                joint_alpha = df_skeleton.loc[:, joint_alpha_field].mean() # takes either 0.5 or 1.0
                no_observation_rate = 1 - (joint_alpha * 2 - 1)
                self.df_agg.at[trial_id, self.param_list[1] + "_" + joint_alpha_field[:-1]] = no_observation_rate

            if args.plot_scatter:
                self.df_skeletons.append(df_skeleton)

            
            # # sr for each joint
            # plt.close("all")
            # df_kinect_sr = df_skeleton.loc[:, ["pelvisA", "ankleLA", "ankleRA"]].mean() * 2 - 1
            # df_kinect_fr = 1 - df_kinect_sr
            # df_kinect_fr.plot(kind="bar")
            # plt.show()

    def update_and_save_csv(self):
        # Calculate Mean
        df_mean = self.df_agg.mean()
        df_mean.name = "mean"
        # Create aggregate table with mean
        self.df_with_mean = self.df_agg.append(df_mean)
        # Write to csv
        csv_filename_out = os.path.join(WS_PATH, "kinect_sr.csv")
        self.df_with_mean.to_csv(csv_filename_out, index=True, float_format='%.4f')
        print("Aggregate table saved to: " + csv_filename_out)

        # Calculate by-sbj table
        self.df_by_sbj = self.df_agg.groupby("sbj").mean()
        csv_filename_out = os.path.join(WS_PATH, "kinect_sr_by_sbj.csv")
        self.df_by_sbj.to_csv(csv_filename_out, index=True, float_format='%.4f')
        print("By-sbj table saved to: " + csv_filename_out)


    def plot_chart(self):
        if args.plot_scatter:
            df_merged = pd.concat(self.df_skeletons)
            out_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "kinect_scatter" + ".jpg")
            plotScatters(df_merged, out_filename)


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
    def filterMarkers(msg):
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

    skeleton_msg_list = [filterMarkers(msg) for msg in skeleton_msg_list]
    skeleton_msg_list = [msg for msg in skeleton_msg_list if msg.markers]
    skeleton_dict = {
        "t":       [msg.markers[0 ].header.stamp.to_sec() for msg in skeleton_msg_list],
        "pelvisA": [msg.markers[0 ].color.a               for msg in skeleton_msg_list],
        "pelvisX": [msg.markers[0 ].pose.position.x       for msg in skeleton_msg_list],
        "pelvisY": [msg.markers[0 ].pose.position.y       for msg in skeleton_msg_list],
        "pelvisZ": [msg.markers[0 ].pose.position.z       for msg in skeleton_msg_list],
        "ankleLA": [msg.markers[20].color.a               for msg in skeleton_msg_list],
        "ankleLX": [msg.markers[20].pose.position.x       for msg in skeleton_msg_list],
        "ankleLY": [msg.markers[20].pose.position.y       for msg in skeleton_msg_list],
        "ankleLZ": [msg.markers[20].pose.position.z       for msg in skeleton_msg_list],
        "ankleRA": [msg.markers[24].color.a               for msg in skeleton_msg_list],
        "ankleRX": [msg.markers[24].pose.position.x       for msg in skeleton_msg_list],
        "ankleRY": [msg.markers[24].pose.position.y       for msg in skeleton_msg_list],
        "ankleRZ": [msg.markers[24].pose.position.z       for msg in skeleton_msg_list],
    }

    df_skeleton =  pd.DataFrame(skeleton_dict)

    # Remove begining and end of the time series
    ts = df_skeleton.t
    df_skeleton = df_skeleton[(ts > ts.iat[0] + 15) & (ts < ts.iat[-1] - 5)]

    return df_skeleton


def process(inbag, bag_name):
    pkl_filename = os.path.join(WS_PATH, "kinect_pkl", bag_name + ".pkl")
    if os.path.exists(pkl_filename) or not args.force_update:
        with open(pkl_filename, "rb") as pkl_file:
            df_skeleton = pickle.load(pkl_file)
    else:
        print("Updating from " + bag_name)
        df_skeleton = extractFromBag(inbag) # Very time-consuming
        with open(pkl_filename, 'wb') as pkl_file:
            pickle.dump(df_skeleton, pkl_file)

    if args.plot_scatter:
        out_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "kinect",
            bag_name + "_scatter" + ".jpg")
        plotScatters(df_skeleton, out_filename)


def plotScatters(df_skeleton, out_filename):
    fig, axs = plt.subplots(1, 3, figsize=(20, 5))
    plotScatter(axs[0], df_skeleton.loc[:, "pelvisX"], df_skeleton.loc[:, "pelvisZ"], 0.0)
    plotScatter(axs[1], df_skeleton.loc[:, "ankleLX"], df_skeleton.loc[:, "ankleLZ"], 0.7)
    plotScatter(axs[2], df_skeleton.loc[:, "ankleRX"], df_skeleton.loc[:, "ankleRZ"], 0.7)

    axs[0].set_title("Pelvis")
    axs[1].set_title("Left Ankle")
    axs[2].set_title("Right Ankle")
    # plt.scatter(df_skeleton.loc[:, "pelvisX"], df_skeleton.loc[:, "pelvisZ"])
    if args.preview: plt.show()
    fig.savefig(out_filename)
    print("Saved to " + out_filename)
    plt.close(fig)

def plotScatter(ax, x, y, y0):
    npts = 41
    xlims, ylims = [-1.5, 1.5], [0.0, 3]
    xedges, yedges = np.linspace(xlims[0], xlims[1], npts), np.linspace(ylims[0], ylims[1], npts)
    
    def min_fov_fun(x): 
        min_range = 0.25
        return np.maximum(np.sqrt((x**2 + y0**2) / 3), np.sqrt(min_range**2 - min([y0, min_range])**2))
    def max_fov_fun(x): 
        max_range = 2.88
        return np.sqrt(max_range**2 - x**2 - y0**2)
    ax.fill_between(xedges, min_fov_fun(xedges), max_fov_fun(xedges), color="lightgray", alpha=0.3)
    
    hist, xedges, yedges = np.histogram2d(x, y, (xedges, yedges))
    xidx = np.clip(np.digitize(x, xedges), 0, hist.shape[0]-1)
    yidx = np.clip(np.digitize(y, yedges), 0, hist.shape[1]-1)
    cmap = hist[xidx, yidx]
    sc = ax.scatter(x, y, c=cmap, s=0.1)
    plt.colorbar(sc, ax=ax)
    ax.set_xlim(xlims)
    ax.set_ylim(ylims)
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
        plotter = BoxPlotter()
        for trial_id in sorted(plotter.data_info_dict.keys()):
        # for trial_id in [424, 425]:
            plotter.process_trial(trial_id)
        plotter.update_and_save_csv()
        plotter.plot_chart()
    else:
        for filename in args.filenames:
            if not os.path.exists(filename):
                print("Cannot find " + filename)
                continue
            print(filename)
            with rosbag.Bag(filename, 'r') as inbag:
                process(inbag, os.path.splitext(os.path.basename(filename))[0])