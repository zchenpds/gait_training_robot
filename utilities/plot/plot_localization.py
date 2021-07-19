#!/usr/bin/python

import matplotlib.pyplot as plt
import os
import glob
import argparse
import numpy as np
from scipy.signal import argrelextrema

import rospy
import rosbag
from tf import Transformer
import tf



def process(inbag, filename):
    odom_list = [msg for _, msg, _ in inbag.read_messages(topics="/odom")]
    xy_odom = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y] for msg in odom_list], dtype=float)
    t_odom = np.array([msg.header.stamp.to_sec() for msg in odom_list], dtype=float)

    tf_list = [msg for _, tfs, _ in inbag.read_messages(topics="/tf") for msg in tfs.transforms if msg.header.frame_id == "map" or msg.header.frame_id == "odom"]
    transformer = Transformer()
    xy_map = []
    t_map = []
    for tf1 in tf_list:
        transformer.setTransform(tf1)
        try:
            tf2 = transformer.lookupTransform('map', 'base_link', rospy.Time(0))
            xy_map.append([tf2[0][0], tf2[0][1]])
            t_map.append(tf1.header.stamp.to_sec())
        except tf.Exception:
            pass
    xy_map = np.array(xy_map)
    t_map = np.array(t_map)

    dist_to_origin = (xy_odom[:, 0]**2 + xy_odom[:, 1]**2)**0.5
    i_minima_odom = argrelextrema(dist_to_origin, np.less)[0]
    i_minima_odom = np.r_[[0], i_minima_odom[np.nonzero(dist_to_origin[i_minima_odom] < 3.0)], len(dist_to_origin) - 1]
    t_minima = t_odom[i_minima_odom]
    i_minima_map = [next((i for i, t in enumerate(t_map) if t >= tmin), -1) for tmin in t_minima]
    if args.debug:
        plt.plot(t_odom, dist_to_origin)
        plt.plot(t_odom[i_minima_odom], dist_to_origin[i_minima_odom], 'x')
        plt.show()

    if args.separate_lap:
        plot_list = []
        for k in range(1, min([5, len(i_minima_odom)])):
            a, b = i_minima_odom[k-1], i_minima_odom[k]
            c, d = i_minima_map[k-1], i_minima_map[k]
            plot_list.append([(xy_odom[a:b, 0], xy_odom[a:b, 1], 'Odom (Lap {:d})'.format(k), 'x [m]', 'y [m]'),
                              (xy_map[c:d, 0], xy_map[c:d, 1], 'SLAM (Lap {:d})'.format(k), 'x [m]', 'y [m]')])
    else:
        plot_list = [[
            (xy_odom[:, 0], xy_odom[:, 1], 'Odom', 'x [m]', 'y [m]'),
            (xy_map[:, 0], xy_map[:, 1], 'SLAM', 'x [m]', 'y [m]')]]
    
    fig, axs = plt.subplots(len(plot_list), len(plot_list[0]), squeeze=False)
    fig.set_size_inches(13, 19)
    for i in range(len(plot_list)):
        for j in range(len(plot_list[0])):
            ax = axs[i][j]
            x, y, legend, xlabel, ylabel = plot_list[i][j]
            ax.plot(x, y, label=legend)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_aspect('equal')
            # if legend: ax.legend()
            ax.set_title(legend)
    # plt.tight_layout()
    plt.savefig(filename)
    if args.preview: plt.show()
        


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filenames", nargs='*', help="Bag file path(s). Wildcard is supported.")
    parser.add_argument("-d", "--debug", action="store_true")
    parser.add_argument("-s", "--separate-lap", action="store_true")
    parser.add_argument("-p", "--preview", action="store_true")
    args = parser.parse_args()
    if not args.filenames:
        print("Empty list of files.")
    for filename in args.filenames:
        if not os.path.exists(filename):
            print("Cannot find " + filename)
        print(filename)
        with rosbag.Bag(filename, 'r') as inbag:
            png_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "Localization", 
                os.path.splitext(os.path.basename(filename))[0] + ".png")
            process(inbag, png_filename)