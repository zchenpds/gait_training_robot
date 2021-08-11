import numpy as np
import rospy
import math
from collections import namedtuple


def get_straight_segment_time_ranges(inbag, time_range, odom_topic='/odom', 
        moving_mean_window_size=50):
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics=odom_topic)]
    t = np.array([msg.header.stamp.to_sec() for msg in msg_list])
    wz = np.array([msg.twist.twist.angular.z for msg in msg_list])
    # Moving mean filter and absolute value
    wz_filtered_abs = np.fabs(np.convolve(wz,
        np.ones(moving_mean_window_size)/float(moving_mean_window_size), mode='same'))
    lower_threshold = 0.3 * np.max(wz_filtered_abs)
    upper_threshold = 2.0 * lower_threshold
    
    res = [] # Time ranges
    t_min, t_max = time_range
    t1 = t_min
    for i in range(len(t)):
        if t[i] < t_min: continue
        elif t[i] > t_max: break
        if not t1:
            if wz_filtered_abs[i] < lower_threshold:
                t1 = t[i]
        else:
            if wz_filtered_abs[i] > upper_threshold or i == len(t) - 1:
                t2 = t[i]
                res.append((t1, t2))
                t1 = None

    return res


def get_mos_vec_dict(inbag, topic):
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics=topic)]
    return {"t": np.array([msg.header.stamp.to_sec() for msg in msg_list]),
            "MOS": np.array([[msg.vector.x, msg.vector.y, msg.vector.z] for msg in msg_list])}

def get_gait_state_dict(inbag, topic):
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics=topic)]
    return {"t": np.array([msg.header.stamp.to_sec() for msg in msg_list]),
            "LState": np.array([[ord(msg.gait_phase[0]) & 1, ord(msg.gait_phase[0]) & 2] for msg in msg_list], dtype=bool),
            "RState": np.array([[ord(msg.gait_phase[1]) & 1, ord(msg.gait_phase[1]) & 2] for msg in msg_list], dtype=bool)}

def merge(data1, data2):
    res = {}
    res["t"], idx1, idx2 = np.intersect1d(data1["t"], data2["t"], return_indices=True)
    res.update({k: v[idx1] for k, v in data1.items()})
    res.update({k: v[idx2] for k, v in data2.items()})
    return res

class SpatialParams:
    """
    A class that represents the spatial gait parameters.

    ...

    Attributes
    ----------
    name : stride_lengths
    name : step_lengths
    name : step_widths
    Methods
    -------
    get_stride_lengths():
        Returns both the left and right stride lengths [l, r].
    get_step_lengths():
        Returns step lengths [lrl, rlr].
    get_step_widths():
        Returns step widths [lrl, rlr].
    """
    def __init__(self):
        self.stride_lengths = []
        self.step_lengths = []
        self.step_widths = []
        self.stride_velocities = []

    def get_stride_lengths(self):
        return self.stride_lengths

    def get_step_lengths(self):
        return self.step_lengths

    def get_step_widths(self):
        return self.step_widths

    def get_stride_velocities(self):
        return self.stride_velocities

    def get_stride_times(self):
        return self.stride_times


StrideRow = namedtuple("StrideRow", ["ts_FC", "LR", "StrideL", "StrideV", "StrideT"])
StepRow   = namedtuple("StepRow",   ["ts_FC", "LR", "StepL", "StepW"])

class StepData(SpatialParams):
    """
    A class that represents the data for each step of both left and right feet.

    ...

    Attributes
    ----------
    name : data
        [pos_l, pos_r]
        where pos_l and pos_r are each a list of (numpy) 3-vectors representing the positions of the left and right steps.
    name : stance_intervals
        [intervals_l, intervals_r]
        where intervals_l and intervals_r are each a list of intervals of the form [t_start, t_end].
    """
    def __init__(self, inbag, topic_pose, stance_intervals):
        self.stance_intervals = stance_intervals
        #! [pos_l, pos_r]
        self.ts_ff = [[], []]
        self.data = [[], []]
        self.stride_table = [[], []]
        self.step_table = [[], []]

        stamp_list = [msg.data.to_sec() for _, msg, _ in inbag.read_messages(topics='/sport_sole_publisher/t0_zeno')]
        if not stamp_list: stamp_list = [msg.header.stamp.to_sec() for _, msg, _ in inbag.read_messages(topics='/sport_sole_publisher/sport_sole')]
        self.t0_zeno = next(ts for ts in stamp_list if ts > 1.0 )
        
        def get_mean(pos_list):
            return np.mean(pos_list[-len(pos_list)/3-1:], axis=0)

        for lr in [0, 1]:
            if lr == 0: lr_str = 'l'
            else: lr_str = 'r'
            i = 0
            pos_list = []
            msg_list = [msg for _, msg, _ in inbag.read_messages(topics=topic_pose+lr_str)]
            for msg in msg_list:
                if i >= len(stance_intervals[lr]): break
                if msg.header.stamp < stance_intervals[lr][i][0]: continue
                elif msg.header.stamp >= stance_intervals[lr][i][0] and msg.header.stamp < stance_intervals[lr][i][1]:
                    pos_list.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
                else:
                    self.ts_ff[lr].append(stance_intervals[lr][i][0].to_sec())
                    self.data[lr].append(get_mean(pos_list))
                    pos_list = []
                    i += 1
            if pos_list:
                self.ts_ff[lr].append(stance_intervals[lr][i][0].to_sec())
                self.data[lr].append(get_mean(pos_list))
                pos_list = []

        #! [SL_l, SL_r]
        self.stride_lengths = [[], []]
        self.stride_velocities = [[], []]
        self.stride_times = [[], []]
        for lr in [0, 1]:
            for i in range(1, len(self.data[lr])):
                # Calculate stride length
                strideL = np.linalg.norm(self.data[lr][i] - self.data[lr][i - 1])
                if strideL > 2.0: continue
                self.stride_lengths[lr].append(strideL)
                # Calculate stride velocity
                strideT = self.ts_ff[lr][i] - self.ts_ff[lr][i - 1]
                strideV = strideL / strideT
                if strideT < 2.0: # Exclude strides that last too long
                    self.stride_velocities[lr].append(strideV)
                    self.stride_times[lr].append(strideT)
                # Add to table
                self.stride_table[lr].append(
                    StrideRow(self.ts_ff[lr][i-1] - self.t0_zeno,
                                   ("L" if lr == 0 else "R") + str(i),
                                   strideL * 100,
                                   strideV * 100, 
                                   strideT))
            self.stride_lengths[lr] = np.array(self.stride_lengths[lr])
            self.stride_velocities[lr] = np.array(self.stride_velocities[lr])
            self.stride_times[lr] = np.array(self.stride_times[lr])
            # print(lr, len(self.stride_lengths[lr]), len(self.stride_velocities[lr]))
        self.stride_table_sorted = sorted(self.stride_table[0] + self.stride_table[1], key=lambda stride_row: stride_row.ts_FC)
        
        #! [lrl, rlr]
        self.step_lengths = [[], []]
        self.step_widths = [[], []]
        for lr in [0, 1]:
            #! if lr == 0: a, b, c ~ l, r, l
            #! if lr == 1: a, b, c ~ r, l, r
            b = 0
            b_max = len(stance_intervals[1 - lr])
            for i in range(1, len(stance_intervals[lr])):
                a, c = i - 1, i
                while b < b_max and stance_intervals[1 - lr][b][0] <= stance_intervals[lr][a][0]: b += 1
                if b == b_max: break
                if stance_intervals[1 - lr][b][0] > stance_intervals[lr][c][0]: continue
                pa, pb, pc = self.data[lr][a], self.data[1 - lr][b], self.data[lr][c]
                step_length = np.dot(pc - pa, pb - pa) / np.linalg.norm(pc - pa)
                step_width = math.sqrt(np.linalg.norm(pb - pa) ** 2  - step_length ** 2)
                if step_length > 1.0 or step_length < 0.3: continue
                self.step_lengths[lr].append(step_length)
                self.step_widths[lr].append(step_width)
                # Add to table
                self.step_table[lr].append(
                    StepRow(self.ts_ff[lr][i-1] - self.t0_zeno,
                                 ("L" if lr == 0 else "R") + str(i),
                                 step_length * 100,
                                 step_width * 100))
            self.step_lengths[lr] = np.array(self.step_lengths[lr])
            self.step_widths[lr] = np.array(self.step_widths[lr])
        self.step_table_sorted = sorted(self.step_table[0] + self.step_table[1], key=lambda step_row: step_row.ts_FC)

    def __sub__(self, other):
        res = SpatialParams()
        for lr in [0, 1]:
            for attr in ['stride_lengths', 'step_lengths', 'step_widths', 'stride_velocities']:
                l1 = len(getattr(self, attr)[lr])
                l2 = len(getattr(other, attr)[lr])
                l = min(l1, l2)
                getattr(res, attr).append(getattr(self, attr)[lr][:l] - getattr(other, attr)[lr][:l])
        return res



def calc_rmse(error):
    if isinstance(error, list): error = np.concatenate(error)
    return np.sqrt(np.mean(error**2))

def calc_mae(error):
    if isinstance(error, list): error = np.concatenate(error)
    return np.mean(np.abs(error))

def calc_mae_percentage(error, data_true, data_min=0.0):
    if isinstance(error, list): error = np.concatenate(error)
    if isinstance(data_true, list): data_true = np.concatenate(data_true)
    idx = np.where(data_true > data_min) 
    return np.mean(np.abs(error[idx])/data_true[idx])


def get_stance_intervals(inbag, topic_gs, t_min, t_max):
    FF = '\x03'
    gait_phase_prev = [0, 0]
    res = [[], []]
    for _, msg, _ in inbag.read_messages(topics=topic_gs):
        for lr in [0, 1]:
            if msg.gait_phase[lr] == FF and gait_phase_prev[lr] != FF:
                res[lr].append([msg.header.stamp])
            elif msg.gait_phase[lr] != FF and gait_phase_prev[lr] == FF:
                res[lr][-1].append(msg.header.stamp)
                if res[lr][-1][0].to_sec() < t_min or res[lr][-1][1].to_sec() > t_max:
                    del res[lr][-1]
            gait_phase_prev[lr] = msg.gait_phase[lr]
    for lr in [0, 1]:
        if len(res[lr]) and len(res[lr][-1]) == 1:
            res[lr][-1].append(res[lr][-1][0] + rospy.Duration(1.0))
    return res


def print_stats(param_name, data, unit="cm", mode="mean_sd"):
    m1, m2 = np.mean(data[0]), np.mean(data[1])
    s1, s2 = np.std(data[0]),  np.std(data[1])
    n1, n2 = len(data[0]),     len(data[1])
    scale = 100.0 if "cm" in unit else 1000
    combined_std = math.sqrt(( (n1 - 1) * s1 * s1 + (n2 - 1) * s2 * s2 
        + n1 * n2 / (n1 + n2) * (m1**2 + m2**2 - 2 * m1 * m2))
        / (n1 + n2 - 1)) * scale
    combined_mean = (n1 * m1 + n2 * m2) / (n1 + n2) * scale

    if mode == "mean_sd":
        return ('{:3.1f}({:3.1f}) ').format(combined_mean, combined_std).ljust(15)
    else: # cv
        return ('{:3.2f} ').format(combined_std/combined_mean*100).ljust(10)