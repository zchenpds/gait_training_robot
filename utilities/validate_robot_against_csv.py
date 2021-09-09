#!/usr/bin/python

import os
import pandas as pd
import numpy as np
import glob
from collections import namedtuple
import pickle
import ga
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
    dataInfoNT = namedtuple("dataInfoNT", ["path", "sbj", "session"])
    ws_path = "/home/ral2020/Documents/sunny/"
    str_prefix = "zeno"
    session_list = ["D", "E", "F", "G"]
    stride_params = ["StrideT", "StrideV", "StrideL"]
    stride_units  = ["(sec.)", "(cm./sec.)", "(cm.)"]
    step_params = ["StepL", "StepW"]
    step_units  = ["(cm.)", "(cm.)"]
    etype_list = ["MAE", "ESD"]
    meanstd_list = ["mean", "std", "se"]

    @classmethod
    def constructDataInfo(cls, sbj, session):
        sbj = "SBJ" + sbj
        search_pattern = os.path.join(cls.ws_path, cls.str_prefix, sbj, "*" + session + "_GaitParameters.csv")
        filename_list = glob.glob(search_pattern)
        assert(len(filename_list) == 1)
        return (filename_list[0], sbj, session.upper())

    def __init__(self):
        self.param_list = self.stride_params + self.step_params
        self.unit_list = self.stride_units + self.step_units
        self.unit_dict = {param: unit for param, unit in zip(self.param_list, self.unit_list)}
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
            # 463: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("019", "g")),
            500: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("020", "F")),
            501: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("020", "G")),
            502: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("020", "D")),
            503: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("020", "E")),
            504: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("022", "F")),
            505: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("022", "G")),
            506: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("022", "D")),
            507: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("022", "E")),
            508: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("021", "F")),
            509: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("021", "G")),
            510: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("021", "D")),
            511: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("021", "E")),
            512: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("023", "F")),
            513: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("023", "G")),
            514: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("023", "D")),
            515: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("023", "E")),
            516: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("024", "F")),
            517: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("024", "G")),
            # 518: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("024", "D")), # No zeno data available
            519: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("024", "E")),
            520: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("025", "F")),
            521: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("025", "G")),
            522: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("025", "D")),
            523: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("025", "E")),
            524: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("026", "F")),
            525: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("026", "G")),
            526: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("026", "D")),
            527: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("026", "E")),
            528: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("027", "F")),
            529: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("027", "G")),
            530: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("027", "D")),
            531: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("027", "E")),
            532: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("028", "F")),
            533: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("028", "G")),
            534: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("028", "D")),
            535: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("028", "E")),
            536: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("029", "F")),
            537: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("029", "G")),
            538: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("029", "D")),
            539: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("029", "E")),
            540: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("030", "D")),
            541: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("030", "E")),
            542: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("030", "F")),
            543: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("030", "G")),
            544: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("031", "D")),
            545: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("031", "E")),
            546: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("031", "F")),
            547: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("031", "G")),
            548: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("032", "D")),
            549: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("032", "E")),
            550: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("032", "F")),
            551: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("032", "G")),
            552: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("033", "F")),
            553: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("033", "G")),
            554: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("033", "D")),
            555: self.dataInfoNT(*ValidationTableUpdater.constructDataInfo("033", "E")),
        }
        self.param_types = ["StrideT", "StrideL", "StrideV",     "StepL",  "StepW"]
        self.param_units = [" (sec.)", " (cm.)",  " (cm./sec.)", " (cm.)", " (cm.)"]
        self.param_names = [a + b for a, b in zip(self.param_types, self.param_units)]

        # Initialize DataFrame
        self.df_agg = pd.DataFrame("", index=sorted(self.data_info_dict.keys()),
            columns=["sbj", "session"] + [prefix + param_name for prefix in ["MAE_", "ESD_"] for param_name in self.param_names])
        self.dfs_by_param = \
        {etype:
            {param:
                {meanstd: pd.DataFrame(np.nan, index=self.session_list, columns=["robot"]) 
                    for meanstd in self.meanstd_list
                }
                for param in self.param_list}
            for etype in self.etype_list
        }

        # Insert subject info columns
        for trial_id in self.data_info_dict.keys():
            self.df_agg.at[trial_id, "sbj"]     = self.data_info_dict[trial_id].sbj
            self.df_agg.at[trial_id, "session"] = self.data_info_dict[trial_id].session

    def process_trial(self, trial_id):
        csv_filename_in = self.data_info_dict[trial_id].path
        csv_filename_out = csv_filename_in.replace("GaitParameters", "Compare")
        df = pd.read_csv(csv_filename_in, skiprows=range(0,11)+range(12,27), 
            usecols=["First Contact (sec.)", "Step Length (cm.)",
                "Stride Length (cm.)", "Stride Width (cm.)", 
                "Stride Time (sec.)", "Stride Velocity (cm./sec.)"])

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
                self.df_agg.at[trial_id, "MAE_" + param_name] = np.mean(np.abs(df.loc[:, "Err" + param_type]))
                self.df_agg.at[trial_id, "ESD_" + param_name] = np.std(df.loc[:, "Err" + param_type])
    

    def update_and_save_csv(self):
        # Calculate Mean
        series_mean = self.df_agg.mean()
        series_mean.name = "mean"
        # Create aggregate table with mean
        df_with_mean = self.df_agg.append(series_mean)
        # Write to csv
        csv_filename_out = os.path.join(self.ws_path, "Compare_robot_zeno.csv")
        df_with_mean.to_csv(csv_filename_out, index=True, float_format='%.4f')
        print("Aggregate table saved to: " + csv_filename_out)


    def plotChart(self):
        # Populate dataframe df_sessions by session
        data_sources = ["robot"]
        for etype in self.etype_list:
            for param in self.param_list:
                df_sessions = {session: pd.DataFrame(np.nan, index=[], columns=data_sources)
                                for session in self.session_list}
                field = "_".join([etype, param])
                # Find col that contains field
                for col in list(self.df_agg):
                    if field in col:
                        for trial_id, data_info in self.data_info_dict.items():
                            sbj = data_info.sbj
                            session = data_info.session
                            for data_source in data_sources:
                                df_sessions[session].at[sbj, data_source] = self.df_agg.at[trial_id, col]
                for session in df_sessions.keys():
                    MEAN = df_sessions[session].mean()
                    STD  = df_sessions[session].std()
                    SE   = STD / np.math.sqrt(len(df_sessions[session]))
                    for data_source in data_sources:
                        self.dfs_by_param[etype][param]["mean"].at[session, data_source] = MEAN[data_source]
                        self.dfs_by_param[etype][param]["std"].at[session, data_source]  = STD[data_source]
                        self.dfs_by_param[etype][param]["se"].at[session, data_source] = SE[data_source]

        for etype in self.etype_list:
            for param, unit in zip(self.param_list, self.unit_list):
                for meanstd in self.meanstd_list:
                    csv_filename = os.path.join(self.ws_path, 
                        "by_param", "robot_" + param + "_" + etype + "_" + meanstd + ".csv")
                    self.dfs_by_param[etype][param][meanstd].to_csv(csv_filename)
                    print(param + " saved to: " + csv_filename)

                # Plot
                plt.figure()
                self.dfs_by_param[etype][param]["mean"].plot(kind="bar", capsize=4, legend=False,
                    rot=0, title=param + " " + etype, yerr = self.dfs_by_param[etype][param]["se"])
                plt.ylabel(" ".join([etype, unit]))
                fig_filename = os.path.join(self.ws_path, "by_param", "robot_" + param + "_" + etype + ".jpg")
                plt.savefig(fig_filename)
                print(param + " saved to: " + fig_filename)


if __name__ == "__main__":
    vtu = ValidationTableUpdater()
    for trial_id in sorted(vtu.data_info_dict.keys()):
    # for trial_id in [424]:
        vtu.process_trial(trial_id)
    vtu.update_and_save_csv()
    vtu.plotChart()
