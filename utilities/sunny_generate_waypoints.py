#!/usr/bin/python3

import os
import yaml
import math
import sys
from collections import namedtuple

import numpy as np
import scipy

import argparse

# Pose in SE(2)
Pose = namedtuple("Pose", ["x", "y", "th"])

def getWaypoint(pose, seq):
    # print("The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3]))
    return {
        "header": {
            "seq": seq,
            "frame_id": "map", # map, odom
        },
        "pose": {
            "position": {
                "x": float(pose.x),
                "y": float(pose.y),
            },
            "orientation": {
                "z": float(math.sin(pose.th / 2)),
                "w": float(math.cos(pose.th / 2)),
            }
        }

    }


class Path:
    def __init__(self, x, y, th):
        """ Initialize the path with the first waypoint specified. """
        self._path = []
        self._path.append(Pose(x, y, th))
        self.s = 0.0

    @classmethod
    def setArgs(cls, args):
        cls.ds = args.ds

    def appendCircular(self, radius, angle):
        radius = math.fabs(radius)
        if radius < 1.0:
            print("Radius too small.")
            return
        n_segments = int(radius * math.fabs(angle) // Path.ds)
        dth = angle / n_segments
        if dth == 0:
            return

        th0 = self._path[-1].th - math.copysign(math.pi / 2.0, angle)
        xc = self._path[-1].x - radius * math.cos(th0)
        yc = self._path[-1].y - radius * math.sin(th0)
        for th in np.linspace(th0, th0 + angle, n_segments, endpoint=False) + dth:
            x = xc + radius * math.cos(th)
            y = yc + radius * math.sin(th)
            tangent = th + math.copysign(math.pi / 2.0, dth)
            self._path.append(Pose(x, y, tangent))
        self.s += math.fabs(radius * angle)

    def appendStraight(self, x1, y1):
        x0 = self._path[-1].x
        y0 = self._path[-1].y
        th = math.atan2(y1, x1)
        s1 = math.hypot(y1, x1)
        n_segments = int(s1 // Path.ds)
        for s in np.linspace(0, s1, n_segments, endpoint=False) + s1 / n_segments:
            x = x0 + s * math.cos(th)
            y = y0 + s * math.sin(th)
            self._path.append(Pose(x, y, th))
        self.s += s1

    def close_path(self):
        if len(self._path) < 2:
            print("Cannot close path. Too few waypoints.")
        x1 = self._path[0].x - self._path[-1].x
        y1 = self._path[0].y - self._path[-1].y
        self.appendStraight(x1, y1)


    def writeYaml(self, suffix):
        # Helper function replacing rospkg.RosPack().get_path()
        def get_ros_package_path(package_name):
            for ros_path in os.getenv("ROS_PACKAGE_PATH").split(':'):
                package_path = os.path.join(ros_path, package_name)
                if os.path.exists(package_path):
                    return package_path
            raise Exception("Cannot find ROS package: " + package_name)

        with open(get_ros_package_path('gait_training_robot') + '/data/waypoints' + suffix + '.yaml', 'w') as outfile:
            for i, pose in enumerate(self._path):
                yaml.dump(getWaypoint(pose, i), outfile, default_flow_style=False, explicit_start=True)



if __name__ == '__main__':
    elongation_term = 0.0
    r = 2.0 - elongation_term / 2
    l = 9.0 + elongation_term
    b = 1.5 + elongation_term / 2

    parser = argparse.ArgumentParser(description="Prescribe the path for the robot to follow.")
    parser.add_argument('-ds', type=float, default=0.5, help='Separation between consecutive waypoints.')

    args = parser.parse_args()
    Path.setArgs(args)


    for dir in ["cw", "ccw"]:
        if dir == "cw":
            path_human = Path(0, 0, math.pi)
            path_human.appendStraight(b - l, 0)
            path_human.appendCircular(r, -math.pi)
            path_human.appendStraight(l, 0)
            path_human.appendCircular(r, -math.pi)
        else:
            path_human = Path(0, 0, 0)
            path_human.appendStraight(b, 0)
            path_human.appendCircular(r, math.pi)
            path_human.appendStraight(-l, 0)
            path_human.appendCircular(r, math.pi)
        path_human.close_path()
        path_human.writeYaml("_sunny_" + dir)
        print("Human path circumference {:3s}: {:.2f} m.".format(dir, path_human.s))