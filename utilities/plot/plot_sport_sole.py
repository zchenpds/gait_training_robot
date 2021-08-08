#!/usr/bin/python

import matplotlib.pyplot as plt
import os
import glob
import argparse
import numpy as np
from scipy.signal import argrelextrema

import rospy
import rosbag


def process(inbag, png_raw_filename, png_gap_filename, t0=None):
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics="/sport_sole_publisher/sport_sole")]
    ts = np.array([msg.header.stamp.to_sec() for msg in msg_list])
    if len(ts) == 0:
        print("No sport sole data")
        return

    if t0 is None:
        t0 = ts[0]
    ts -= t0

    def pres_to_list(pres, i0): return [pres[i] for i in range(i0, i0 + 8)]
    def xyz_to_list(xyz): return [xyz.x, xyz.y, xyz.z]

    plot_list = [
        [(ts, np.array([pres_to_list(msg.pressures, lr * 8) for msg in msg_list]), 
          ('Left' if lr == 0 else 'Right') + ' Pressure', 't [s]', 'p') for lr in [0, 1]], 
        [(ts, np.array([xyz_to_list(msg.raw_acceleration[lr].linear) for msg in msg_list]), 
          ('Left' if lr == 0 else 'Right') + ' Raw Acceleration', 't [s]', 'a_raw [m/s^2]') for lr in [0, 1]],
        [(ts, np.array([xyz_to_list(msg.acceleration[lr].linear) for msg in msg_list]), 
          ('Left' if lr == 0 else 'Right') + ' Acceleration due to motion', 't [s]', 'a [m/s^2]') for lr in [0, 1]],
        [(ts, np.array([xyz_to_list(msg.angular_velocity[lr]) for msg in msg_list]), 
          ('Left' if lr == 0 else 'Right') + ' Angular Velocity', 't [s]', 'w [rad/s]') for lr in [0, 1]]
    ]

    # Stack plots vertically
    plot_list = [[tup] for row in plot_list for tup in row]
    
    fig, axs = plt.subplots(len(plot_list), len(plot_list[0]), squeeze=False)
    fig.set_size_inches(100, 40)
    for i in range(len(plot_list)):
        for j in range(len(plot_list[0])):
            ax = axs[i][j]
            x, y, legend, xlabel, ylabel = plot_list[i][j]
            ax.plot(x, y, label=legend)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            # if legend: ax.legend()
            ax.set_title(legend)
    plt.tight_layout()
    plt.savefig(png_raw_filename)
    print("Saved to " + png_raw_filename)
    if args.preview: plt.show()

    plt.figure()
    msg_list = [msg for _, msg, _ in inbag.read_messages("/sport_sole_publisher/sport_sole")]
    plt.figure(figsize=(20, 6))
    plt.plot(ts, np.diff(ts, prepend=ts[0]))
    plt.plot([ts[0], ts[-1]], np.ones(2) * 0.1, 'g--')
    plt.xlabel('t [s]')
    plt.ylabel('Gap [s]')
    plt.savefig(png_gap_filename)
    print("Saved to " + png_gap_filename)
    if args.preview: plt.show()

        


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filenames", nargs='*', help="Bag file path(s). Wildcard is supported.")
    parser.add_argument("-d", "--debug", action="store_true")
    parser.add_argument("-p", "--preview", action="store_true")
    args = parser.parse_args()
    if not args.filenames:
        print("Empty list of files.")
    for filename in args.filenames:
        if not os.path.exists(filename):
            print("Cannot find " + filename)
        print(filename)
        with rosbag.Bag(filename, 'r') as inbag:
            png_raw_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "SportSoleRaw", 
                os.path.splitext(os.path.basename(filename))[0] + ".png")
            png_gap_filename = os.path.join(os.path.expanduser("~"), "Pictures", "sunnyside", "SportSoleGaps", 
                os.path.splitext(os.path.basename(filename))[0] + ".png")
            process(inbag, png_raw_filename, png_gap_filename)