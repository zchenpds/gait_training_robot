#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3Stamped
from sport_sole.msg import GaitState

import numpy as np
import matplotlib.pyplot as plt

MOS_YLIMITS = [[-80.0, 30.0], [-80.0, 30.0], [-20.0, 20.0]]
MOS_LABELS  = ("MoS [cm]", "MoS AP [cm]", "MoS ML [cm]")

# class MosPlotter:
#     def __init__(self):
#         pass

if __name__ == "__main__":
    plt.rcParams.update({'font.size': 16})
    fig, axs = plt.subplots(3, 1, figsize=(11, 7.5))
    ts, mos_vec = [], [[], [], []]
    lns_curr = [None] * len(MOS_LABELS)
    lns_prev = [None] * len(MOS_LABELS)
    for i in range(len(MOS_LABELS)):
        lns_curr[i] = axs[i].plot([], [], color='black', label="Current cycle")
        lns_prev[i] = axs[i].plot([], [], color='gray', label="Previous cycle")

    t0 = [0]
    def mosCallback(msg):
        ts.append(msg.header.stamp.to_sec() - t0[0])
        mos_vec[0].append(msg.vector.x * 100)
        mos_vec[1].append(msg.vector.y * 100)
        mos_vec[2].append(msg.vector.z * 100)

    prev_state = ['\x00']
    reset = [False]
    def gaitStateCallback(msg):
        curr_state = msg.gait_phase[0]
        # print(curr_state, prev_state[0])
        if curr_state != '\x00' and prev_state[0] == '\x00':
            reset[0] = True
            # print("Heel Strike.")
            t0[0] = msg.header.stamp.to_sec()
            ts[:] = []
            for i in range(len(MOS_LABELS)):
                mos_vec[i][:] = []
        prev_state[0] = curr_state

        
    rospy.init_node('plot_mos')
    rospy.Subscriber("/gait_analyzer/estimate/mos_vec", Vector3Stamped, mosCallback)
    rospy.Subscriber("/gait_analyzer/gait_state", GaitState, gaitStateCallback)
    # rospy.wait_for_message("/gait_analyzer/gait_state", Vector3Stamped)
    
    # Initialize figure
    for i in range(len(axs)):
        axs[i].set_xlim(0.0, 1.5)
        axs[i].set_ylim(MOS_YLIMITS[i])
        axs[i].set_xlabel("Time since last left heel-strike [sec]")
        axs[i].set_ylabel(MOS_LABELS[i])
        axs[i].legend(loc="upper right")
    plt.tight_layout()

    plt.ion()
    plt.show()
    plt.pause(0.05)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if reset[0]:
            for i in range(len(MOS_LABELS)):
                # print(type(lns_curr[i][0].get_xdata()))
                lns_prev[i][0].set_data(lns_curr[i][0].get_xdata(), lns_curr[i][0].get_ydata()[:])
            reset[0] = False
        for i in range(len(MOS_LABELS)):
            lns_curr[i][0].set_data(ts[:], mos_vec[i][:])
        # print(len(ts))
        plt.pause(0.02)
        r.sleep()