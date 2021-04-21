import numpy as np
import rospy
import math

def get_mos_vec_dict(inbag, topic):
    msg_list = [msg for _, msg, _ in inbag.read_messages(topics=topic)]
    return {"t": np.array([msg.header.stamp.to_sec() for msg in msg_list]),
            "MOS": np.array([[msg.vector.x, msg.vector.y, msg.vector.z] for msg in msg_list])}

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

    def get_stride_lengths(self):
        return self.stride_lengths

    def get_step_lengths(self):
        return self.step_lengths

    def get_step_widths(self):
        return self.step_widths

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
        self.data = [[], []]
        
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
                    self.data[lr].append(get_mean(pos_list))
                    pos_list = []
                    i += 1
            if pos_list:
                self.data[lr].append(get_mean(pos_list))
                pos_list = []

        #! [SL_l, SL_r]
        self.stride_lengths = [[], []]
        for lr in [0, 1]:
            for i in range(1, len(self.data[lr])):
                self.stride_lengths[lr].append(np.linalg.norm(self.data[lr][i] - self.data[lr][i - 1]))
            self.stride_lengths[lr] = np.array(self.stride_lengths[lr])
        
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
                self.step_lengths[lr].append(step_length)
                self.step_widths[lr].append(step_width)
            self.step_lengths[lr] = np.array(self.step_lengths[lr])
            self.step_widths[lr] = np.array(self.step_widths[lr])

    def __sub__(self, other):
        res = SpatialParams()
        for lr in [0, 1]:
            for attr in ['stride_lengths', 'step_lengths', 'step_widths']:
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