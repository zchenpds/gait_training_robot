#!/usr/bin/python

import os
import pandas as pd
import numpy as np
import glob
from collections import namedtuple
import pickle
import ga


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
    dataInfoNT = namedtuple("dataInfoNT", ["path", "sbj", "session"])
    ws_path = "/home/ral2020/Documents/sunny/"

    @classmethod
    def getMatFileName(cls, sbj, session, str_prefix):
        search_pattern = os.path.join(cls.ws_path, str_prefix, sbj, "*" + session + "_GaitParameters.csv")
        filename_list = glob.glob(search_pattern)
        assert(len(filename_list) == 1)
        return (filename_list[0], sbj, session)

    def __init__(self):
        self.data_info_dict = {
            424: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ010", "d", "zeno")),
            425: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ010", "e", "zeno")),
            426: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ010", "f", "zeno")),
            427: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ010", "g", "zeno")),
            428: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ011", "d", "zeno")),
            429: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ011", "e", "zeno")),
            430: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ011", "f", "zeno")),
            431: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ011", "g", "zeno")),
            432: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ012", "d", "zeno")),
            433: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ012", "e", "zeno")),
            434: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ012", "f", "zeno")),
            435: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ012", "g", "zeno")),
            436: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ013", "d", "zeno")),
            437: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ013", "e", "zeno")),
            438: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ013", "f", "zeno")),
            439: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ013", "g", "zeno")),
            444: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ015", "d", "zeno")),
            445: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ015", "e", "zeno")),
            446: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ015", "f", "zeno")),
            447: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ015", "g", "zeno")),
            448: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ016", "d", "zeno")),
            449: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ016", "e", "zeno")),
            450: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ016", "f", "zeno")),
            451: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ016", "g", "zeno")),
            456: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ018", "d", "zeno")),
            457: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ018", "e", "zeno")),
            458: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ018", "f", "zeno")),
            459: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ018", "g", "zeno")),
            460: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ019", "d", "zeno")),
            461: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ019", "e", "zeno")),
            462: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ019", "f", "zeno")),
            463: self.dataInfoNT(*ValidationTableUpdater.getMatFileName("SBJ019", "g", "zeno")),
        }
        self.param_types = ["StrideT", "StrideL", "StrideV",     "StepL",  "StepW"]
        self.param_units = [" (sec.)", " (cm.)",  " (cm./sec.)", " (cm.)", " (cm.)"]
        self.param_names = [a + b for a, b in zip(self.param_types, self.param_units)]
        self.df = pd.DataFrame(np.nan, index=sorted(self.data_info_dict.keys()),
            columns=[prefix + param_name for prefix in ["MAE_", "ESD_"] for param_name in self.param_names])

        # Insert subject info columns
        self.df.insert(0, "sbj",     "")
        self.df.insert(1, "session", "")
        for trial_id in self.data_info_dict.keys():
            self.df.at[trial_id, "sbj"]     = self.data_info_dict[trial_id].sbj
            self.df.at[trial_id, "session"] = self.data_info_dict[trial_id].session

    def process_trial(self, trial_id):
        csv_filename_in = self.data_info_dict[trial_id].path
        csv_filename_out = csv_filename_in.replace("GaitParameters", "Compare")
        df = pd.read_csv(csv_filename_in, skiprows=range(0,11)+range(12,27), 
            usecols=["First Contact (sec.)", "Step Length (cm.)",
                "Stride Length (cm.)", "Stride Width (cm.)", 
                "Stride Time (sec.)", "Stride Velocity (cm./sec.)"])

        for param_name in self.param_names:
            df["Err" + param_name] = np.nan

        pkl_filename = os.path.join(self.ws_path, "robot", "data" + str(trial_id).rjust(3, '0') + ".pkl")
        with open(pkl_filename, "rb") as pkl_file:
            STEP_KINECT = pickle.load(pkl_file)
            ts_zeno = df["First Contact (sec.)"]
            for row in STEP_KINECT.stride_table_sorted:
                abs_diff = abs(ts_zeno - row.ts_FC)
                i = abs_diff.idxmin()
                if abs_diff.at[i] > 0.2: continue
                for field in ["StrideT", "StrideV", "StrideL"]:
                    df.at[i, field] = getattr(row, field)
                df.at[i, "ErrStrideT"] = df.at[i, "StrideT"] - df.at[i, "Stride Time (sec.)"]
                df.at[i, "ErrStrideL"] = df.at[i, "StrideL"] - df.at[i, "Stride Length (cm.)"]
                df.at[i, "ErrStrideV"] = df.at[i, "StrideV"] - df.at[i, "Stride Velocity (cm./sec.)"]
            for row in STEP_KINECT.step_table_sorted:
                abs_diff = abs(ts_zeno - row.ts_FC)
                i = abs_diff.idxmin()
                if abs_diff.at[i] > 0.2: continue
                for field in ["StepL", "StepW"]:
                    df.at[i, field] = getattr(row, field)
                df.at[i, "ErrStepL"] = df.at[i, "StepL"] - df.at[i, "Step Length (cm.)"]
                df.at[i, "ErrStepW"] = df.at[i, "StepW"] - df.at[i, "Stride Width (cm.)"]

            df.to_csv(csv_filename_out, index=False, float_format='%.4f')
            print("Saved to: " + csv_filename_out)

            # Update aggregate table
            for param_type, param_name in zip(self.param_types, self.param_names):
                self.df.at[trial_id, "MAE_" + param_name] = np.mean(np.abs(df.loc[:, "Err" + param_type]))
                self.df.at[trial_id, "ESD_"   + param_name] = np.std(df.loc[:, "Err" + param_type])
    
    def update_and_save_csv(self):
        # Calculate Mean
        df_mean = self.df.mean()
        df_mean.name = "mean"
        # Create aggregate table with mean
        self.df_with_mean = self.df.append(df_mean)
        # Write to csv
        csv_filename_out = os.path.join(self.ws_path, "Compare_robot_zeno.csv")
        self.df_with_mean.to_csv(csv_filename_out, index=True, float_format='%.4f')
        print("Aggregate table saved to: " + csv_filename_out)




if __name__ == "__main__":
    vtu = ValidationTableUpdater()
    for trial_id in sorted(vtu.data_info_dict.keys()):
    # for trial_id in [424]:
        vtu.process_trial(trial_id)
    vtu.update_and_save_csv()