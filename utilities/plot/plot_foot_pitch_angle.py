#!/usr/bin/python

import matplotlib.pyplot as plt
import os
import argparse
import numpy as np

import rosbag

def movmean(x, window_size):
    return np.convolve(x, np.ones(window_size)/float(window_size), mode='same')

def process(inbag):
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics="/sport_sole_publisher/sport_sole")]
    if not msg_list: raise Exception("Cannot find sport_sole message.")
    w = np.array([[msg.angular_velocity[0].x, msg.angular_velocity[0].y, msg.angular_velocity[0].z] for msg in msg_list], dtype=float)
    t = np.array([msg.header.stamp.to_sec() for msg in msg_list])
    t -= t[0]
    gs = np.array([msg.gait_state for msg in msg_list])
    lstate = [(s>>2) & 3 for s in gs]

    # for i, l in enumerate(["x", "y", "z"]):
    #     plt.subplot(3, 1, i + 1)
    #     plt.plot(t, w[:, i])
    #     plt.title(l)

    dt = np.diff(t, append=[t[-1]])
    wy = w[:, 1]
    wy = movmean(wy, 50)
    wy_int = np.zeros(len(dt))
    for i in range(0, len(dt) - 1):
        wy_int[i+1] = wy_int[i] + dt[i] * wy[i] if not (lstate[i] == 0 and lstate[i+1] == 2) else 0
    wy_int = movmean(wy_int, 50)

    plot_list = [
        (t, lstate, '0: Swing, 1: LC, 2:IC, 3:FF', 'Gait state'), 
        (t, wy, '', 'Pitch rate [rad/s]'), 
        (t, wy_int, '', 'Pitch angle [rad]')]
    _, axs = plt.subplots(len(plot_list), 1, sharex=True)
    for ax, (x, y, legend, ylabel) in zip(axs, plot_list):
        ax.plot(x, y, label=legend)
        ax.set_ylabel(ylabel)
        if legend: ax.legend()
    plt.xlabel('t [s]')
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filenames", nargs='*', help="Bag file path(s). Wildcard is supported.")
    args = parser.parse_args()
    if not args.filenames:
        print("Empty list of files.")
    for filename in args.filenames:
        if not os.path.exists(filename):
            print("Cannot find " + filename)
        print(filename)
        with rosbag.Bag(filename, 'r') as inbag:
            process(inbag)