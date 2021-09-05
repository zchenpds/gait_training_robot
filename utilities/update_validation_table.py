#!/usr/bin/python

import os
import scipy.io
import glob
from collections import namedtuple
import pickle
import ga
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


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
    dataInfoNT = namedtuple("dataInfoNT", ["path_ts", "path_val", "path_raw", "sbj", "session"])
    ws_path = "/home/ral2020/Documents/sunny/"
    session_list = ["D", "E", "F", "G"]
    sbj_list = []
    param_list = ["StrideT", "StrideV", "StrideL"]
    unit_list  = ["(sec.)", "(cm./sec.)", "(cm.)"]
    etype_list = ["MAE", "ESD"]
    meanstd_list = ["mean", "std"]

    @classmethod
    def constructDataInfo(cls, sbj, session):
        sbj = "C" + sbj
        if sbj not in cls.sbj_list:
            cls.sbj_list.append(sbj)
        session = session.upper()
        filename_list1 = glob.glob(os.path.join(cls.ws_path, sbj, "_Processed", "processed_" + "*" + session + ".mat"))
        assert(len(filename_list1) == 1)
        filename_list2 = glob.glob(os.path.join(cls.ws_path, sbj, "_Processed", "validation2" + "*" + session + ".mat"))
        assert(len(filename_list2) == 1)
        filename_list3 = glob.glob(os.path.join(cls.ws_path, sbj, "matlab_" + "*" + session + ".mat"))
        assert(len(filename_list3) == 1)
        return (filename_list1[0], filename_list2[0], filename_list3[0], sbj, session)

    def __init__(self):
        self.data_info_dict = {
            424: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("010", "d")),
            425: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("010", "e")),
            426: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("010", "f")),
            427: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("010", "g")),
            428: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("011", "d")),
            429: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("011", "e")),
            430: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("011", "f")),
            431: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("011", "g")),
            432: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("012", "d")),
            433: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("012", "e")),
            434: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("012", "f")),
            435: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("012", "g")),
            436: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("013", "d")),
            437: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("013", "e")),
            438: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("013", "f")),
            439: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("013", "g")),
            444: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("015", "d")),
            445: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("015", "e")),
            446: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("015", "f")),
            447: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("015", "g")),
            448: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("016", "d")),
            449: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("016", "e")),
            450: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("016", "f")),
            451: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("016", "g")),
            456: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("018", "d")),
            457: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("018", "e")),
            458: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("018", "f")),
            459: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("018", "g")),
            460: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("019", "d")),
            461: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("019", "e")),
            462: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("019", "f")),
            463: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("019", "g")),
        }
        self.df_trial = None
        self.df_agg = pd.DataFrame("", index=sorted(self.data_info_dict.keys()),columns=["sbj", "session"])
        self.dfs_by_param = \
        {etype:
            {param: 
                {meanstd: pd.DataFrame(np.nan, index=self.session_list, columns=["robot", "sportsole"]) 
                    for meanstd in self.meanstd_list
                }
                for param in self.param_list}
            for etype in self.etype_list
        }

        # Insert subject info columns
        for trial_id in self.data_info_dict.keys():
            self.df_agg.at[trial_id, "sbj"]     = self.data_info_dict[trial_id].sbj
            self.df_agg.at[trial_id, "session"] = self.data_info_dict[trial_id].session

    def test(self, trial_id):
        data_info = self.data_info_dict[trial_id]
        mat_filename_in = data_info.path_val
        mat = scipy.io.loadmat(mat_filename_in)
        mat_filename_out = mat_filename_in.replace("validation2", "validation3")
        csv_filename_out = os.path.join(self.ws_path, "validation_csv", data_info.sbj + "_" + data_info.session + ".csv")

        # Find tables
        ts = scipy.io.loadmat(data_info.path_ts)["L_t"]
        ext_sync = scipy.io.loadmat(data_info.path_raw)["PDShoeL"][0,0]["ExtSync"]
        ts_zeno0 = ts[np.argmax(ext_sync > 0)]
        table_info_list = [
            (mat["updatedValidation"][0,0]["Stride_Time_sec"][0,0]["validationTable"], "StrideT", " (sec.)"),
            (mat["updatedValidation"][0,0]["Stride_Length_cm"][0,0]["validationTable"], "StrideL", " (cm.)"),
            (mat["updatedValidation"][0,0]["Stride_Velocity_cm_sec"][0,0]["validationTable"], "StrideV", " (cm./sec.)"),
        ]
        # print(temp)
        pkl_filename = os.path.join(self.ws_path, "robot", "data" + str(trial_id).rjust(3, '0') + ".pkl")
        with open(pkl_filename, "rb") as pkl_file:
            STEP_KINECT = pickle.load(pkl_file)

            for i, (table, field, unit) in enumerate(table_info_list):
                ts_col = np.empty((table.shape[0], 1))
                ts_col[:, 0] = ts[table[:,2].astype(int), 0] - ts_zeno0

                robot_col = np.empty((table.shape[0], 2))
                robot_col[:, :] = np.nan
                self.updateRobotCol(robot_col, ts_col, STEP_KINECT, field)

                new_table = np.hstack((table_info_list[i][0], ts_col, robot_col))
                self.updateCsv(new_table, field, unit)
                if i == 0: 
                    mat["updatedValidation"][0,0]["Stride_Time_sec"][0,0]["validationTable"] = new_table
                if i == 1: 
                    mat["updatedValidation"][0,0]["Stride_Length_cm"][0,0]["validationTable"] = new_table
                if i == 2: 
                    mat["updatedValidation"][0,0]["Stride_Velocity_cm_sec"][0,0]["validationTable"] = new_table
            scipy.io.savemat(mat_filename_out, mat)
            
            # Process csv by trial_id
            for col in list(self.df_trial):
                self.df_agg.at[trial_id, "MAE_" + col] = np.mean(np.abs(self.df_trial.loc[:, col]))
                self.df_agg.at[trial_id, "ESD_" + col] = np.std(self.df_trial.loc[:, col])
            
            # Process csv by session
            data_sources = ["robot", "sportsole"]
            for etype in self.etype_list:
                for param in self.param_list:
                    df_sessions = {session: pd.DataFrame(np.nan, index=self.sbj_list, columns=data_sources)
                                    for session in self.session_list}
                    for trial_id, data_info in self.data_info_dict.items():
                        sbj = data_info.sbj
                        session = data_info.session
                        for data_source in data_sources:
                            # Find col that contains field
                            for col in list(self.df_agg):
                                field = "_".join([etype, param, data_source, "zeno"])
                                if field in col:
                                    df_sessions[session].at[sbj, data_source] = self.df_agg.at[trial_id, col]
                    for session in df_sessions.keys():
                        MEAN = df_sessions[session].mean()
                        STD  = df_sessions[session].std()
                        for data_source in data_sources:
                            self.dfs_by_param[etype][param]["mean"].at[session, data_source] = MEAN[data_source]
                            self.dfs_by_param[etype][param]["std"].at[session, data_source]  = STD[data_source]


            self.df_trial.to_csv(csv_filename_out, index=False, float_format='%.4f')
            print("Saved to: " + csv_filename_out)
            self.df_trial = None
            

    def updateCsv(self, validation_table, field, unit):
        table_selected = np.hstack((validation_table[:, 0:2], validation_table[:, 26:27]))
        data_sources = ["_zeno", "_sportsole", "_robot"]
        df1 = pd.DataFrame(table_selected, columns=[field + suffix for suffix in data_sources])
        df2 = pd.DataFrame()
        for a, b in [(1, 0), (2, 0), (2, 1)]:
            df2.loc[:, field + data_sources[a] + data_sources[b] + unit] = \
                df1.loc[:, field + data_sources[a]] - df1.loc[:, field + data_sources[b]]
        if self.df_trial is None:
            self.df_trial = df2
        else:
            self.df_trial = self.df_trial.join(df2)


    @staticmethod
    def updateRobotCol(robot_col, ts_col, STEP_KINECT, field):
        for row in STEP_KINECT.stride_table_sorted:
            ts_sportsole = row.ts_FC
            diff_col = np.abs(ts_col - ts_sportsole)
            if diff_col.min() < 0.3:
                i = np.argmin(diff_col)
                robot_col[i, 0] = diff_col[i]
                robot_col[i, 1] = getattr(row, field)
            pass

    def update_and_save_csv(self):
        # Calculate Mean
        df_mean = self.df_agg.mean()
        df_mean.name = "mean"
        # Create aggregate table with mean
        self.df_with_mean = self.df_agg.append(df_mean)
        # Write to csv
        csv_filename_out = os.path.join(self.ws_path, "Compare_robot_zeno_sportsole.csv")
        self.df_with_mean.to_csv(csv_filename_out, index=True, float_format='%.4f')
        print("Aggregate table saved to: " + csv_filename_out)

    def plotChart(self):
        for etype in self.etype_list:
            for param, unit in zip(self.param_list, self.unit_list):
                for meanstd in self.meanstd_list:
                    csv_filename = os.path.join(self.ws_path, 
                        "by_param", param + "_" + etype + "_" + meanstd + ".csv")
                    self.dfs_by_param[etype][param][meanstd].to_csv(csv_filename)
                    print(param + " saved to: " + csv_filename)

                # Plot
                plt.figure()
                self.dfs_by_param[etype][param]["mean"].plot(kind="bar", capsize=4,
                    rot=0, title=param, yerr = self.dfs_by_param[etype][param]["std"])
                plt.ylabel(" ".join([etype, unit]))
                fig_filename = os.path.join(self.ws_path, "by_param", param + "_" + etype + ".jpg")
                plt.savefig(fig_filename)
                print(param + " saved to: " + fig_filename)


if __name__ == "__main__":
    vtu = ValidationTableUpdater()
    for trial_id in vtu.data_info_dict.keys():
    # for trial_id in [424, 425]:
        vtu.test(trial_id)
    vtu.update_and_save_csv()
    vtu.plotChart()