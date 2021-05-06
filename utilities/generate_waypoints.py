#!/usr/bin/python

import yaml
import rospy
import rospkg
import math
import sys

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

import argparse
parser = argparse.ArgumentParser(description="Prescribe the path for the robot to follow.")
parser.add_argument('room_id', type=str)
parser.add_argument('-ds', type=float, default=0.5, help='Separation between consecutive waypoints.')
parser.add_argument('-yaw_degrees', type=float, default=0.0, help='Rotation of the whole trajectory')

args = parser.parse_args()
ds = args.ds
yaw_degrees = 0
frame_id = "map" # map, odom

def getWaypoint(x, y, th, seq):
    q = quaternion_from_euler(0.0, 0.0, th)
    # print("The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3]))
    return {
        "header": {
            "seq": seq,
            "frame_id": frame_id,
        },
        "pose": {
            "position": {
                "x": float(x),
                "y": float(y),
            },
            "orientation": {
                "z": float(q[2]),
                "w": float(q[3]),
            }
        }

    }

def getYawFromWaypoint(waypoint):
    quat = [0.0, 0.0, waypoint['pose']['orientation']['z'], waypoint['pose']['orientation']['w']]
    euler = euler_from_quaternion(quat)
    return euler[2]

def appendCircularPath(path, xc, yc, radius, th0, th1):
    radius = math.fabs(radius)
    if radius < 1.0:
        print("Radius too small.")
        return path
    dth = ds / radius * math.copysign(1.0, th1 - th0)
    if dth == 0:
        return path

    th = th0
    seq = 0
    if path:
        appendStraightPath(path, xc + radius * math.cos(th), yc + radius * math.sin(th))
        seq = path[-1]["header"]["seq"] + 1
    while dth * (th - th1) < 0:
        x = xc + radius * math.cos(th)
        y = yc + radius * math.sin(th)
        tangent = th + math.copysign(math.pi / 2.0, dth)
        path.append(getWaypoint(x, y, tangent, seq))
        seq += 1
        th += dth
    return path
    
    


def appendStraightPath(path, x1, y1):
    if not path:
        print("Cannot append when path is empty.")
        return path
    x0 = path[-1]["pose"]["position"]["x"]
    y0 = path[-1]["pose"]["position"]["y"]
    seq = path[-1]["header"]["seq"] + 1
    th = math.atan2(y1 - y0, x1 - x0)
    dx = ds * math.cos(th)
    dy = ds * math.sin(th)
    x, y, s = x0, y0, 0.0
    s1 = math.hypot(y1 - y0, x1 - x0)
    while s + ds < s1:
        x += dx
        y += dy
        s += ds
        path.append(getWaypoint(x, y, th, seq))
        seq += 1
    return path

def closePath(path):
    if len(path) < 2:
        print("Cannot close path. Too few waypoints.")
        return path
    x1 = path[0]["pose"]["position"]["x"]
    y1 = path[0]["pose"]["position"]["y"]
    appendStraightPath(path, x1, y1)
    return path

def rotateAround(path, xc, yc, yaw):
    s = math.sin(yaw)
    c = math.cos(yaw)
    for waypoint in path:
        x = waypoint["pose"]["position"]["x"]
        y = waypoint["pose"]["position"]["y"]
        x -= xc
        y -= yc
        x = x * c - y * s
        y = x * s + y * c
        waypoint["pose"]["position"]["x"] = x + xc
        waypoint["pose"]["position"]["y"] = y + yc
        th = getYawFromWaypoint(waypoint) + yaw
        q = quaternion_from_euler(0.0, 0.0, th)
        waypoint['pose']['orientation']['z'] = float(q[2])
        waypoint['pose']['orientation']['w'] = float(q[3])
    
        

def writeYaml(path, suffix):
    rp = rospkg.RosPack()
    with open(rp.get_path('gait_training_robot') + '/data/waypoints' + suffix + '.yaml', 'w') as outfile:
        for wp in path:
            yaml.dump(wp, outfile, default_flow_style=False, explicit_start=True)


PI = math.pi


def main():
    if args.room_id == 'eas102':
        r = 1.4
        R = 3.9
        alpha = math.pi * 0.2
        dx = (R - r) * math.cos(alpha)
        dy = (R - r) * math.sin(alpha)

        # cw
        xc1, yc1 = 4.0, 1.2
        xc2, yc2 = 4.0, -2.7
        path = []
        # Upper semicircle
        appendCircularPath(path, xc1 + dx, yc1 - dy, R, PI, PI - alpha)
        appendCircularPath(path, xc1, yc1, r, PI - alpha, alpha)
        appendCircularPath(path, xc1 - dx, yc1 - dy, R, alpha, 0.0)
        # Lower semicircle
        appendCircularPath(path, xc2 - dx, yc2 + dy, R, 0.0, -alpha)
        appendCircularPath(path, xc2, yc2, r, -alpha, -PI + alpha)
        appendCircularPath(path, xc2 + dx, yc2 + dy, R, -PI + alpha, -PI)
        closePath(path)
        rotateAround(path, (xc1 + xc2) / 2.0, (yc1 + yc2) / 2.0, args.yaw_degrees * PI / 180.0)
        suffix = '_cw'
        writeYaml(path, suffix)

        # ccw
        xc1, yc1 = 4.2, 1.2
        xc2, yc2 = 4.2, -2.7
        path = []
        # Lower semicircle
        appendCircularPath(path, xc2 + dx, yc2 + dy, R, -PI, -PI + alpha)
        appendCircularPath(path, xc2, yc2, r, -PI + alpha, -alpha)
        appendCircularPath(path, xc2 - dx, yc2 + dy, R, -alpha, 0.0)
        # Upper semicircle
        appendCircularPath(path, xc1 - dx, yc1 - dy, R, 0.0, alpha)
        appendCircularPath(path, xc1, yc1, r, alpha, PI - alpha)
        appendCircularPath(path, xc1 + dx, yc1 - dy, R, PI - alpha, PI)
        closePath(path)
        rotateAround(path, (xc1 + xc2) / 2.0, (yc1 + yc2) / 2.0, yaw_degrees * PI / 180.0)
        suffix = '_ccw'
        writeYaml(path, suffix)
    elif args.room_id == 'eas101':
        r = 1.2
        R = 1.7
        alpha = math.pi * 0.3
        dx = (R - r) * math.cos(alpha)
        dy = (R - r) * math.sin(alpha)
        
        # cw
        xc1, yc1 = 0.0, 1.5
        xc2, yc2 = -2.0, 1.5
        path = []
        # Upper semicircle
        appendCircularPath(path, xc2 + dx, yc2 + dy, R, 3*PI/2,         3*PI/2 - alpha)
        appendCircularPath(path, xc2,      yc2,      r, 3*PI/2 - alpha, PI/2 + alpha)
        appendCircularPath(path, xc2 + dx, yc2 - dy, R, PI/2 + alpha,   PI/2)
        # Lower semicircle
        appendCircularPath(path, xc1 - dx, yc1 - dy, R, PI/2,           PI/2 - alpha)
        appendCircularPath(path, xc1,      yc1,      r, PI/2 - alpha,  -PI/2 + alpha)
        appendCircularPath(path, xc1 - dx, yc1 + dy, R, -PI/2 + alpha, -PI/2)
        closePath(path)
        rotateAround(path, (xc1 + xc2) / 2.0, (yc1 + yc2) / 2.0, args.yaw_degrees * PI / 180.0)
        suffix = '_cw'
        writeYaml(path, suffix)

        # ccw
        path = []
        # Lower semicircle
        appendCircularPath(path, xc1 - dx, yc1 + dy, R, -PI/2,        -PI/2 + alpha)
        appendCircularPath(path, xc1,      yc1,      r, -PI/2 + alpha, PI/2 - alpha)
        appendCircularPath(path, xc1 - dx, yc1 - dy, R,  PI/2 - alpha, PI/2)
        # Upper semicircle
        appendCircularPath(path, xc2 + dx, yc2 - dy, R, PI/2,           PI/2 + alpha)
        appendCircularPath(path, xc2,      yc2,      r, PI/2 + alpha,   3*PI/2 - alpha)
        appendCircularPath(path, xc2 + dx, yc2 + dy, R, 3*PI/2 - alpha, 3*PI/2)
        closePath(path)
        rotateAround(path, (xc1 + xc2) / 2.0, (yc1 + yc2) / 2.0, args.yaw_degrees * PI / 180.0)
        suffix = '_ccw'
        writeYaml(path, suffix)
    else:
        raise Exception('Unknown room_id.')


if __name__ == '__main__':
    main()