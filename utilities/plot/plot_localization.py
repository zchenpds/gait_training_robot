#!/usr/bin/python

import matplotlib.pyplot as plt
import os
import glob
import argparse
import numpy as np
from scipy.signal import argrelextrema, butter, filtfilt
from collections import namedtuple
import pickle

import rospy
import rosbag
from tf import Transformer
import tf

DESIRED_DIST = 1.4

PositionSeries = namedtuple("PositionSeries", ["t", "xy", "i_minima"])
CurveData = namedtuple("CurveData", ["x", "y", "opt"])
PlotData = namedtuple('PlotData', ["curve_list", "title", "xlabel", "ylabel"])

def process(inbag, filename, bag_name):
    odom_list = [msg for _, msg, _ in inbag.read_messages(topics="/odom")]
    pos_robot_odom = PositionSeries(
        t=[msg.header.stamp.to_sec() for msg in odom_list],
        xy=[[msg.pose.pose.position.x, msg.pose.pose.position.y] for msg in odom_list],
        i_minima=[]
    )

    transformer = Transformer()
    tf_static_list = [msg for _, tfs, _ in inbag.read_messages(topics=["/tf_static"]) for msg in tfs.transforms]
    for tf_static in tf_static_list:
        transformer._buffer.set_transform_static(tf_static, "default_authority")

    pos_human_odom = PositionSeries([], [], [])
    pos_robot_map = PositionSeries([], [], [])
    pos_human_map = PositionSeries([], [], [])
    pos_human_robot = PositionSeries([], [], [])

    tf_list = [msg for _, tfs, _ in inbag.read_messages(topics=["/tf"]) for msg in tfs.transforms]
    for tf1 in tf_list:
        transformer.setTransform(tf1)
        if tf1.header.frame_id == "map" or tf1.header.frame_id == "odom":
            try:
                tf2 = transformer.lookupTransform('map', 'base_link', rospy.Time(0))
                pos_robot_map.xy.append([tf2[0][0], tf2[0][1]])
                pos_robot_map.t.append(tf1.header.stamp.to_sec())
            except tf.Exception:
                pass
        elif tf1.child_frame_id == "joint_pelvis":
            try:
                tf2 = transformer.lookupTransform('odom', 'joint_pelvis', rospy.Time(0))
                pos_human_odom.xy.append([tf2[0][0], tf2[0][1]])
                pos_human_odom.t.append(tf1.header.stamp.to_sec())
            except tf.Exception:
                pass
            try:
                tf2 = transformer.lookupTransform('map', 'joint_pelvis', rospy.Time(0))
                pos_human_map.xy.append([tf2[0][0], tf2[0][1]])
                pos_human_map.t.append(tf1.header.stamp.to_sec())
            except tf.Exception:
                pass
            try:
                tf2 = transformer.lookupTransform('base_link', 'joint_pelvis', rospy.Time(0))
                pos_human_robot.xy.append([tf2[0][0], tf2[0][1]])
                pos_human_robot.t.append(tf1.header.stamp.to_sec())
            except tf.Exception:
                pass
    
    if not pos_human_odom.t:
        raise Exception("Cannot find tf from odom to joint_pelvis. Did you specify the processed bag file in folder ga?")
    
    t0 = pos_robot_odom.t[0]
    pos_robot_odom = pos_robot_odom._replace(t=np.array(pos_robot_odom.t) - t0, xy=np.array(pos_robot_odom.xy))
    pos_human_odom = pos_human_odom._replace(t=np.array(pos_human_odom.t) - t0, xy=np.array(pos_human_odom.xy))
    pos_robot_map = pos_robot_map._replace(t=np.array(pos_robot_map.t) - t0, xy=np.array(pos_robot_map.xy))
    pos_human_map = pos_human_map._replace(t=np.array(pos_human_map.t) - t0, xy=np.array(pos_human_map.xy))
    pos_human_robot = pos_human_robot._replace(t=np.array(pos_human_robot.t) - t0, xy=np.array(pos_human_robot.xy))

    dist_error = (pos_human_robot.xy[:, 0]**2 + pos_human_robot.xy[:, 1]**2)**0.5 - DESIRED_DIST
    idx = np.nonzero(np.logical_and(pos_human_robot.t > min(pos_human_robot.t) + 15, pos_human_robot.t < max(pos_human_robot.t) - 5))
    dist_rmse = np.sqrt(np.mean(dist_error[idx]**2)) * 100
    dist_error_sd = np.std(dist_error[idx]) * 100
    if args.print_distance_rmse:
        print("{:.3} ({:.3})".format(dist_rmse, dist_error_sd))
        if not args.export_pickle:
            return

    if args.export_pickle:
        pickle_filename = os.path.join("/home/ral2020/Documents/sunny/localization", bag_name[:7] + ".pkl")
        with open(pickle_filename, 'wb') as pickle_file:
            pickle.dump([dist_error, dist_rmse, dist_error_sd], pickle_file)
            print("Saved to " + pickle_filename)
        return


    # Separate time series into laps
    dist_to_origin = (pos_robot_odom.xy[:, 0]**2 + pos_robot_odom.xy[:, 1]**2)**0.5
    i_minima_odom = argrelextrema(dist_to_origin, np.less)[0]
    i_minima_odom = np.r_[[0], i_minima_odom[np.nonzero(dist_to_origin[i_minima_odom] < 3.0)], len(dist_to_origin) - 1]
    t_minima = pos_robot_odom.t[i_minima_odom]

    for pos_series in [pos_robot_odom, pos_human_odom, pos_robot_map, pos_human_map, pos_human_robot]:
        pos_series.i_minima.extend([next((i for i, t in enumerate(pos_series.t) if t >= tmin), -1) for tmin in t_minima])
    if args.debug:
        plt.plot(pos_robot_odom.t, dist_to_origin)
        plt.plot(pos_robot_odom.t[i_minima_odom], dist_to_origin[i_minima_odom], 'x')
        plt.show()

    def get_lap_curve_data(position_series, k, mode, opts):
        a, b = position_series.i_minima[k-1], position_series.i_minima[k]
        if mode == 0:
            return CurveData(position_series.xy[a:b, 0], position_series.xy[a:b, 1], opts)
        elif mode == 1: # Start point
            return CurveData(position_series.xy[a, 0], position_series.xy[a, 1], opts)
        elif mode == 2: # 2-norm
            return CurveData(position_series.t[a:b], np.sqrt(position_series.xy[a:b, 0] ** 2 + position_series.xy[a:b, 1] ** 2), opts)
        elif mode == 3: # Horizontal line y = DESIRED_DIST
            return CurveData([position_series.t[a], position_series.t[b]], [DESIRED_DIST, DESIRED_DIST], opts)
        elif mode == 4: # Filtered 2-norm
            v = np.sqrt(position_series.xy[a:b, 0] ** 2 + position_series.xy[a:b, 1] ** 2)
            for i in range(1, len(v)):
                if v[i] > 1.8: v[i] = v[i - 1]
            b1, a1 = butter(4, 0.06)
            if len(v) > 0: v = filtfilt(b1, a1, v, padlen=0)
            return CurveData(position_series.t[a:b], v, opts)
        elif mode == 5: # Horizontal line y = 1.2
            return CurveData([position_series.t[a], position_series.t[b]], [1.2, 1.2], opts)

    # Calculate velocity
    vel_robot_odom = PositionSeries(pos_robot_odom.t, 
        np.diff(pos_robot_odom.xy, axis=0, prepend=[[0.0, 0.0]]) / 
            np.tile(np.expand_dims(np.diff(pos_robot_odom.t, axis=0, prepend=1.0), axis=1), (1, 2)), 
        pos_robot_odom.i_minima)
    vel_human_odom = PositionSeries(pos_human_odom.t,
        np.diff(pos_human_odom.xy, axis=0, prepend=[[0.0, 0.0]]) /
            np.tile(np.expand_dims(np.diff(pos_human_odom.t, axis=0, prepend=1.0), axis=1), (1, 2)), 
        pos_human_odom.i_minima)

    plot_list = []
    robot_color = 'b'
    human_color = 'g'
    marker_size = [1, 10]
    for k in range(1, min([5, len(i_minima_odom)])):
        plot_list.append([
            PlotData(
                [
                    get_lap_curve_data(pos_robot_odom, k, 0, {"label": "Robot", "lw": 1.0, "ls": "-", "color": robot_color}),
                    get_lap_curve_data(pos_human_odom, k, 0, {"label": "Human", "lw": 1.0, "ls": "--", "color": human_color}),
                    get_lap_curve_data(pos_robot_odom, k, 1, {"marker":'o', "markersize": marker_size[1], "color": robot_color}),
                    get_lap_curve_data(pos_human_odom, k, 1, {"marker":'o', "markersize": marker_size[1], "color": human_color}),
                ], 
                'Odom (Lap {:d})'.format(k), 'x [m]', 'y [m]'),
            # PlotData(
            #     [
            #         get_lap_curve_data(pos_robot_map, k, 0, {"label": "Robot", "lw": 1.0, "ls": "-", "color": robot_color}),
            #         get_lap_curve_data(pos_human_map, k, 0, {"label": "Human", "lw": 1.0, "ls": "--", "color": human_color}),
            #         get_lap_curve_data(pos_robot_map, k, 1, {"marker":'o', "markersize": marker_size[1], "color": robot_color}),
            #         get_lap_curve_data(pos_human_map, k, 1, {"marker":'o', "markersize": marker_size[1], "color": human_color}),
            #     ],
            #     'SLAM (Lap {:d})'.format(k), 'x [m]', 'y [m]'),
            PlotData(
                [
                    get_lap_curve_data(pos_human_robot, k, 2, {"label": "Actual", "color": 'm'}),
                    get_lap_curve_data(pos_human_robot, k, 3, {"label": "Desired", "color": 'y', "ls": "--"}),
                ],
                'Human-Robot Distance (Lap {:d})'.format(k), 't [s]', 'Distance [m]'),
            PlotData(
                [
                    get_lap_curve_data(vel_robot_odom, k, 2, {"label": "Robot", "color": 'b'}),
                    get_lap_curve_data(vel_human_odom, k, 4, {"label": "Human", "color": 'g'}),
                    get_lap_curve_data(vel_robot_odom, k, 5, {"label": "Robot Max", "color": 'r', "ls": "--"}),
                ],
                'Velocity (Lap {:d})'.format(k), 't [s]', 'Velocity [m/s]'),
        ])
    
    fig, axs = plt.subplots(len(plot_list), len(plot_list[0]), squeeze=False)
    fig.set_size_inches(25, 18)
    for i in range(len(plot_list)):
        for j in range(len(plot_list[0])):
            ax = axs[i][j]
            curve_list, title, xlabel, ylabel = plot_list[i][j]
            for curve in curve_list:
                ax.plot(curve.x, curve.y, **curve.opt)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            if "[m]" in ax.xaxis.get_label().get_text():
                ax.set_aspect('equal')
            if "Distance" in ax.yaxis.get_label().get_text():
                ax.set_ylim([0.5, 2.5])
            if "Velocity" in ax.yaxis.get_label().get_text():
                ax.set_ylim([0.0, 1.5])
            ax.legend()
            ax.set_title(title)
    # plt.tight_layout()
    plt.savefig(filename)
    if args.preview: plt.show()
        


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filenames", nargs='*', help="Bag file path(s). Wildcard is supported.")
    parser.add_argument("-d", "--debug", action="store_true")
    parser.add_argument("-p", "--preview", action="store_true")
    parser.add_argument("-i", "--export-pickle", action="store_true")
    parser.add_argument("-r", "--print-distance-rmse", action="store_true")
    args = parser.parse_args()
    if not args.filenames:
        print("Empty list of files.")
    for filename in args.filenames:
        if not os.path.exists(filename):
            print("Cannot find " + filename)
        print(filename)
        with rosbag.Bag(filename, 'r') as inbag:
            png_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "Localization", 
                os.path.splitext(os.path.basename(filename))[0] + ".eps")
            process(inbag, png_filename, os.path.basename(filename))