#!/usr/bin/python

import os
import scipy.io
import glob
from collections import namedtuple
import pickle
import ga
import numpy as np


# MAT file structure
# updatedValidation.Stride_Time_sec,
# updatedValidation.Stride_Length_cm,
# updatedValidation.Stride_Velocity_cm_sec,
# updatedValidation.Swing_Time_sec,
# updatedValidation.Swing_Percent,
# updatedValidation.Stance_Time_sec,
# updatedValidation.Stance_Percent,
# updatedValidation.Terminal_DS_Time_sec,
# updatedValidation.Terminal_DS_Percent


class ValidationTableUpdater:
    dataInfoNT = namedtuple("dataInfoNT", ["path_ts", "path_val", "sbj", "session"])
    ws_path = "/home/ral2020/Documents/sunny/"
    ts_prefix = "processed_"
    val_prefix = "validation2"
    @classmethod
    def getMatFileName(cls, sbj, session):
        filename_list1 = glob.glob(os.path.join(cls.ws_path, sbj, "_Processed", cls.ts_prefix + "*" + session + ".mat"))
        assert(len(filename_list1) == 1)
        filename_list2 = glob.glob(os.path.join(cls.ws_path, sbj, "_Processed", cls.val_prefix + "*" + session + ".mat"))
        assert(len(filename_list2) == 1)
        return (filename_list1[0], filename_list2[0], sbj, session)

    def __init__(self):
        self.data_info_dict = {
            424: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C010", "D")),
            425: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C010", "E")),
            426: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C010", "F")),
            427: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C010", "G")),
            428: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C011", "D")),
            429: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C011", "E")),
            430: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C011", "F")),
            431: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C011", "G")),
            432: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C012", "D")),
            433: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C012", "E")),
            434: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C012", "F")),
            435: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C012", "G")),
            436: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C013", "D")),
            437: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C013", "E")),
            438: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C013", "F")),
            439: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C013", "G")),
            444: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C015", "D")),
            445: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C015", "E")),
            446: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C015", "F")),
            447: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C015", "G")),
            448: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C016", "D")),
            449: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C016", "E")),
            450: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C016", "F")),
            451: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C016", "G")),
            456: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C018", "D")),
            457: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C018", "E")),
            458: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C018", "F")),
            459: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C018", "G")),
            460: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C019", "D")),
            461: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C019", "E")),
            462: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C019", "F")),
            463: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("C019", "G")),
        }

    def test(self, trial_id):
        mat_filename_in = self.data_info_dict[trial_id].path_val
        mat = scipy.io.loadmat(mat_filename_in)
        mat_filename_out = mat_filename_in.replace("validation2", "validation3")

        # Find tables
        ts = scipy.io.loadmat(self.data_info_dict[trial_id].path_ts)["L_t"]
        table_scale_list = [
            (mat["updatedValidation"][0,0]["Stride_Time_sec"][0,0]["validationTable"], 1.0),
            (mat["updatedValidation"][0,0]["Stride_Length_cm"][0,0]["validationTable"], 100.0),
            (mat["updatedValidation"][0,0]["Stride_Velocity_cm_sec"][0,0]["validationTable"], 100.0),
        ]
        # print(temp)
        pkl_filename = os.path.join(self.ws_path, "robot", "data" + str(trial_id).rjust(3, '0') + ".pkl")
        with open(pkl_filename, "rb") as pkl_file:
            STEP_KINECT = pickle.load(pkl_file)

            for i, (table, scale) in enumerate(table_scale_list):
                ts_col = np.empty((table.shape[0], 1))
                ts_col[:, 0] = ts[table[:,2].astype(int), 0]

                robot_col = np.empty((table.shape[0], 2))
                robot_col[:, :] = np.nan
                self.updateRobotCol(robot_col, ts_col, STEP_KINECT, scale)

                if i == 0: 
                    mat["updatedValidation"][0,0]["Stride_Time_sec"][0,0]["validationTable"] = \
                        np.hstack((table_scale_list[0][0], ts_col, robot_col))
                if i == 1: 
                    mat["updatedValidation"][0,0]["Stride_Length_cm"][0,0]["validationTable"] = \
                        np.hstack((table_scale_list[1][0], ts_col, robot_col))
                if i == 2: 
                    mat["updatedValidation"][0,0]["Stride_Velocity_cm_sec"][0,0]["validationTable"] = \
                        np.hstack((table_scale_list[2][0], ts_col, robot_col))
            scipy.io.savemat(mat_filename_out, mat)
            pass

    @staticmethod
    def updateRobotCol(robot_col, ts_col, STEP_KINECT, scale):
        for row in STEP_KINECT.stride_table_sorted:
            ts_sportsole = row.ts_FC + STEP_KINECT.t0_zeno - STEP_KINECT.t0_sportsole
            diff_col = np.abs(ts_col - ts_sportsole)
            if diff_col.min() < 0.3:
                i = np.argmin(diff_col)
                robot_col[i, 0] = diff_col[i]
                robot_col[i, 1] = getattr(row, "StrideT") * scale
            pass



if __name__ == "__main__":
    vtu = ValidationTableUpdater()
    for trial_id in vtu.data_info_dict.keys():
    # for trial_id in [424]:
        vtu.test(trial_id)