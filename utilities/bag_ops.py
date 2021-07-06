import os
import shutil
import numpy as np
import math
from scipy.interpolate import interp1d
from scipy.optimize import fmin

def rectify_bag_names(path, trial_id=None):
    files = os.listdir(path)
    if trial_id:
        files = [f for f in files if f[4:7]==trial_id]
        if not files:
            print('Cannot find data' + trial_id + ' in ' + path)
    for file in files:
        name_old = os.path.join(path, file)
        name_new = os.path.join(path, file[:7] + '.bag')
        shutil.move(name_old, name_new)


def trim_time(A, t_ranges):
    """
    Trim the time series data contained in dictionary A.
    @t_ranges: a list of 2-tuples (t_min, t_max)
    """
    idx = np.any(tuple(np.logical_and(A['t'] >= t_min, A['t'] <= t_max) for t_min, t_max in t_ranges), axis=0)
    A.update({k: v[idx] if type(v).__module__ == np.__name__ else v for k, v in A.items()})


class Aligner:
    """ Spatially align one timeseries with another by an SE(2) transform."""
    def __init__(self, A, B):
        """
        Calculate the SE(2) transform that moves B to A. 
        A and B must be dictionaries with the following items:
        {
            "t": <n-1 numpy.ndarray>
            "xyz" <n-3 numpy.ndarray>
        }
        """
        centroidA = np.mean(A["xyz"], axis=0)
        centroidB = np.mean(B["xyz"], axis=0)
        self.centroidA = centroidA
        self.displacement = centroidA - centroidB
        xyA = A["xyz"][:, 0:2]
        xyB = (B["xyz"] + np.tile(self.displacement, [len(B["xyz"]), 1]))[:, 0:2]
        xyAInterp = interp1d(A["t"], xyA, axis=0, copy=True, assume_sorted=True, fill_value="extrapolate")(B["t"])

        def rmse(theta):
            xyBRotated = xyB.copy()
            for i in range(len(xyB)):
                xyBRotated[i, :] = self.__rotate(centroidA, xyB[i, :], theta)
            error = xyAInterp - xyBRotated
            return np.sqrt(np.mean(error[:, 0]**2 + error[:, 1]**2))
        
        self.theta = fmin(rmse, 0)[0]

    @staticmethod
    def __rotate(origin, point, angle):
        ox, oy = origin[0], origin[1]
        px, py = point[0], point[1]
        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return np.array([qx, qy])
    
    def transform(self, B):
        xyB = B["xyz"][:, 0:2]
        for i in range(len(xyB)):
            xyB[i, :] = self.__rotate(self.centroidA, xyB[i, :] + self.displacement[0:2], self.theta)
        return B
    
    @staticmethod
    def rotate(B, theta):
        centroidB = np.mean(B["xyz"], axis=0)
        xyB = B["xyz"][:, 0:2]
        for i in range(len(xyB)):
            xyB[i, :] = Aligner.__rotate(centroidB, xyB[i, :], theta)
        return B



def extractPoseWithCovarianceStamped(inbag, topic, legend=""):
    """
    [geometry_msgs/PoseWithCovarianceStamped]:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/PoseWithCovariance pose
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      float64[36] covariance
    """
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics=topic)]
    return {"t": np.array([msg.header.stamp.to_sec() for msg in msg_list]),
            "xyz": np.array([[msg.pose.pose.position.x, 
                              msg.pose.pose.position.y, 
                              msg.pose.pose.position.z] for msg in msg_list]),
            "legend": legend}


def extractPointStamped(inbag, topic, legend=""):
    """ 
    [geometry_msgs/PointStamped]:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Point point
      float64 x
      float64 y
      float64 z
    """
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics=topic)]
    return {"t": np.array([msg.header.stamp.to_sec() for msg in msg_list]),
            "xyz": np.array([[msg.point.x, 
                              msg.point.y, 
                              msg.point.z] for msg in msg_list]),
            "legend": legend}