#! /usr/bin/python3

import os
import yaml
import math
import sys
from collections import namedtuple

import numpy as np
import scipy.integrate

import argparse

import matplotlib.pyplot as plt
from matplotlib.legend_handler import HandlerPatch
import matplotlib.patches as mpatches


# Pose in SE(2)
Pose = namedtuple("Pose", ["x", "y", "th"])

# Control
# v:  Linear velocity
# w:  Angular velocity
# dt: Duration of the control
Control  = namedtuple("Control", ["v", "w", "dt"])

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
    def __init__(self, x=None, y=None, th=None, agent_name="Robot", color='b'):
        """ Initialize the path with the first waypoint specified. """
        self._path = [] # Pose sequence
        if x is not None and y is not None and th is not None:
            self._path.append(Pose(x, y, th))
        self.s = 0.0
        self._u_path = [] # Control sequence
        self.agent_name = agent_name
        self.color = color

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

        # Update Control sequence
        v = 1.0
        w = v / math.copysign(radius, angle)
        duration = angle / w
        self._u_path.append(Control(v, w, duration))

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

        # Update Control sequence
        v = 1.0
        w = 0.0
        duration = s1 / v
        self._u_path.append(Control(v, w, duration))

    def closePath(self):
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
        
        yaml_filename = get_ros_package_path('gait_training_robot') + '/data/waypoints' + suffix + '.yaml'
        with open(yaml_filename, 'w') as outfile:
            for i, pose in enumerate(self._path):
                yaml.dump(getWaypoint(pose, i), outfile, default_flow_style=False, explicit_start=True)
        print("Waypoints saved to {:%s}" % yaml_filename)

    def preview(self, ax=None):
        arrow_len = 0.7 * args.ds
        if ax is None: ax = plt
        for i, pose in enumerate(self._path):
            if i == 0: kwargs = {"label": self.agent_name}
            else: kwargs = {}
            ax.arrow(pose.x, pose.y, 
                     arrow_len * math.cos(pose.th), arrow_len * math.sin(pose.th),
                     length_includes_head=True,
                     width=arrow_len/12, linewidth=1e-4, color=self.color, **kwargs)
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')

class HumanPath(Path):
    # Distance between human and robot
    RHO = 1.5
    def __init__(self, *args):
        super(HumanPath, self).__init__(*args, agent_name="Human", color='g')

    def getRobotPath(self):
        if not self._u_path: return None

        # Timespans for a sequence of piece-wise constant control commands
        ut = []
        t = 0.0
        for u in self._u_path:
            ut.append((t, t + u.dt))
            t += u.dt

        # Define differential equation dy/dt = f(t, y)
        def func(t, xi):
            thr, thh = xi[2], xi[3]
            ui = next((i for i, t_span in enumerate(ut) if t_span[0] <= t and t <= t_span[1]), -1)
            if ui < 0: v, w = 0, 0
            else: v, w = self._u_path[ui][0], self._u_path[ui][1]
            sinthrh = math.sin(thr - thh)
            xr_dot = v * (math.cos(thh) - sinthrh * math.sin(thr))
            yr_dot = v * (math.sin(thh) + sinthrh * math.cos(thr))
            thr_dot = v * sinthrh / self.RHO
            thh_dot = w
            return [xr_dot, yr_dot, thr_dot, thh_dot]
        
        # Initial condition
        xi0 =  [self._path[0].x + self.RHO * math.cos(self._path[0].th),
                self._path[0].y + self.RHO * math.sin(self._path[0].th),
                self._path[0].th,
                self._path[0].th]

        # Time span for which the ODE is solved
        t_start = ut[0][0]
        t_end = ut[-1][1]
        t_segments = int((t_end - t_start) // self.ds)
        sol = scipy.integrate.solve_ivp(func, (t_end, t_start), xi0, 
                t_eval=np.linspace(t_end, t_start, t_segments),
                max_step=0.01)

        res = Path()
        for row in np.transpose(sol.y)[1:, :]:
            res._path.append(Pose(row[0], row[1], row[2]))
        res._path.reverse()
        res.s = t_end - t_start
        return res

 
if __name__ == '__main__':
    elongation_term = 0.0
    r = 2.5 - elongation_term / 2
    l = 10.0 + elongation_term
    b = 5.0 + elongation_term / 2

    parser = argparse.ArgumentParser(description="Prescribe the path for the robot to follow.")
    parser.add_argument('-ds', type=float, default=0.5, help='Separation between consecutive waypoints.')
    parser.add_argument('-p', '--preview', action="store_true")

    args = parser.parse_args()
    Path.setArgs(args)

    if args.preview:
        _, axs = plt.subplots(2, 1)

    for i, dir in enumerate(["cw", "ccw"]):
        if dir == "cw":
            path_human = HumanPath(0, 0, math.pi)
            path_human.appendStraight(b - l, 0)
            path_human.appendCircular(r, -math.pi/2)
            path_human.appendStraight(0, 1)
            path_human.appendCircular(r, -math.pi/2)
            path_human.appendStraight(l, 0)
            path_human.appendCircular(r, -math.pi/2)
            path_human.appendStraight(0, -1)
            path_human.appendCircular(r, -math.pi/2)
        else:
            path_human = HumanPath(0, 0, 0)
            path_human.appendStraight(b, 0)
            path_human.appendCircular(r, math.pi/2)
            path_human.appendStraight(0, 1)
            path_human.appendCircular(r, math.pi/2)
            path_human.appendStraight(-l, 0)
            path_human.appendCircular(r, math.pi/2)
            path_human.appendStraight(0, -1)
            path_human.appendCircular(r, math.pi/2)
        path_human.closePath()
        path_robot = path_human.getRobotPath()
        for path in [path_human, path_robot]:
            print("{:s} path circumference {:3s}: {:.2f} m, waypoints: {:3d}.".format(
                path.agent_name, dir, path.s, len(path._path)))
        
        if args.preview:
            path_human.preview(axs[i])
            path_robot.preview(axs[i])
        else:
            path_human.writeYaml("_sunny_human_" + dir)
            path_robot.writeYaml("_sunny_" + dir)
    
    if args.preview:
        def make_legend_arrow(legend, orig_handle,
                              xdescent, ydescent,
                              width, height, fontsize):
            p = mpatches.FancyArrow(0, 0.5*height, width, 0, 
                length_includes_head=True, head_width=0.75*height, width=0.2*height)
            return p
        for ax in axs:
            ax.legend(handler_map={mpatches.FancyArrow : HandlerPatch(patch_func=make_legend_arrow)})
        plt.tight_layout()
        plt.savefig(os.path.join(os.getenv("HOME"), "Pictures", "figs", "human_robot_path.eps"))
        plt.show()