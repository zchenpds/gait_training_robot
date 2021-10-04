#! /usr/bin/python

import os
from collections import namedtuple
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pickle
import ga
import scipy.stats
import argparse

class BoxPlotter:
    dataInfoNT = namedtuple("dataInfoNT", ["sbj", "session"])
    ws_path = "/home/ral2020/Documents/sunny"
    session_list = ["D", "E", "F", "G"]
    ratio_list = []
    vibration_conditions = ["VOFF", "VON"] # rows (index)
    vibration_dict = {"D": "VOFF", "E": "VOFF", "F": "VON", "G": "VON"}
    cognitive_conditions = ["COFF", "CON"] # columns
    cognitive_dict = {"D": "COFF", "E": "CON", "F": "COFF", "G": "CON"}
    sbj_list = []
    stride_params = ["StrideT", "StrideV", "StrideL"]
    stride_units  = ["(sec.)", "(cm./sec.)", "(cm.)"]
    step_params = ["StepL", "StepW"]
    step_units  = ["(cm.)", "(cm.)"]
    etype_list = ["CV", "Value"]
    meanstd_list = ["mean", "std", "se"]
    pkl_folder = "robot"
    data_source_robot = "robot"

    @classmethod
    def constructDataInfo(cls, sbj, session):
        sbj = "SBJ" + sbj
        return (sbj, session.upper())

    def __init__(self, is_combined):
        if is_combined:
            self.pkl_folder = "robotv"
            self.data_source_robot = "combined"
        self.param_list = self.stride_params + self.step_params
        self.spss_param_list = []
        self.unit_list = self.stride_units + self.step_units
        self.ratio_xticks = [p + '\n(CON/COFF)' for p in self.param_list]
        self.unit_dict = {param: unit for param, unit in zip(self.param_list, self.unit_list)}
        self.data_info_dict = {
            424: self.dataInfoNT(*BoxPlotter.constructDataInfo("010", "d")),
            425: self.dataInfoNT(*BoxPlotter.constructDataInfo("010", "e")),
            426: self.dataInfoNT(*BoxPlotter.constructDataInfo("010", "f")),
            427: self.dataInfoNT(*BoxPlotter.constructDataInfo("010", "g")),
            428: self.dataInfoNT(*BoxPlotter.constructDataInfo("011", "d")),
            429: self.dataInfoNT(*BoxPlotter.constructDataInfo("011", "e")),
            430: self.dataInfoNT(*BoxPlotter.constructDataInfo("011", "f")),
            431: self.dataInfoNT(*BoxPlotter.constructDataInfo("011", "g")),
            432: self.dataInfoNT(*BoxPlotter.constructDataInfo("012", "d")),
            433: self.dataInfoNT(*BoxPlotter.constructDataInfo("012", "e")),
            434: self.dataInfoNT(*BoxPlotter.constructDataInfo("012", "f")),
            435: self.dataInfoNT(*BoxPlotter.constructDataInfo("012", "g")),
            436: self.dataInfoNT(*BoxPlotter.constructDataInfo("013", "d")),
            437: self.dataInfoNT(*BoxPlotter.constructDataInfo("013", "e")),
            438: self.dataInfoNT(*BoxPlotter.constructDataInfo("013", "f")),
            439: self.dataInfoNT(*BoxPlotter.constructDataInfo("013", "g")),
            444: self.dataInfoNT(*BoxPlotter.constructDataInfo("015", "d")),
            445: self.dataInfoNT(*BoxPlotter.constructDataInfo("015", "e")),
            446: self.dataInfoNT(*BoxPlotter.constructDataInfo("015", "f")),
            447: self.dataInfoNT(*BoxPlotter.constructDataInfo("015", "g")),
            448: self.dataInfoNT(*BoxPlotter.constructDataInfo("016", "d")),
            449: self.dataInfoNT(*BoxPlotter.constructDataInfo("016", "e")),
            450: self.dataInfoNT(*BoxPlotter.constructDataInfo("016", "f")),
            451: self.dataInfoNT(*BoxPlotter.constructDataInfo("016", "g")),
            456: self.dataInfoNT(*BoxPlotter.constructDataInfo("018", "d")),
            457: self.dataInfoNT(*BoxPlotter.constructDataInfo("018", "e")),
            458: self.dataInfoNT(*BoxPlotter.constructDataInfo("018", "f")),
            459: self.dataInfoNT(*BoxPlotter.constructDataInfo("018", "g")),
            460: self.dataInfoNT(*BoxPlotter.constructDataInfo("019", "d")),
            461: self.dataInfoNT(*BoxPlotter.constructDataInfo("019", "e")),
            462: self.dataInfoNT(*BoxPlotter.constructDataInfo("019", "f")),
            463: self.dataInfoNT(*BoxPlotter.constructDataInfo("019", "g")),
            500: self.dataInfoNT(*BoxPlotter.constructDataInfo("020", "F")),
            501: self.dataInfoNT(*BoxPlotter.constructDataInfo("020", "G")),
            502: self.dataInfoNT(*BoxPlotter.constructDataInfo("020", "D")),
            503: self.dataInfoNT(*BoxPlotter.constructDataInfo("020", "E")),
            504: self.dataInfoNT(*BoxPlotter.constructDataInfo("022", "F")),
            505: self.dataInfoNT(*BoxPlotter.constructDataInfo("022", "G")),
            506: self.dataInfoNT(*BoxPlotter.constructDataInfo("022", "D")),
            507: self.dataInfoNT(*BoxPlotter.constructDataInfo("022", "E")),
            508: self.dataInfoNT(*BoxPlotter.constructDataInfo("021", "F")),
            509: self.dataInfoNT(*BoxPlotter.constructDataInfo("021", "G")),
            510: self.dataInfoNT(*BoxPlotter.constructDataInfo("021", "D")),
            511: self.dataInfoNT(*BoxPlotter.constructDataInfo("021", "E")),
            512: self.dataInfoNT(*BoxPlotter.constructDataInfo("023", "F")),
            513: self.dataInfoNT(*BoxPlotter.constructDataInfo("023", "G")),
            514: self.dataInfoNT(*BoxPlotter.constructDataInfo("023", "D")),
            515: self.dataInfoNT(*BoxPlotter.constructDataInfo("023", "E")),
            516: self.dataInfoNT(*BoxPlotter.constructDataInfo("024", "F")),
            517: self.dataInfoNT(*BoxPlotter.constructDataInfo("024", "G")),
            518: self.dataInfoNT(*BoxPlotter.constructDataInfo("024", "D")), # No zeno data available
            519: self.dataInfoNT(*BoxPlotter.constructDataInfo("024", "E")),
            520: self.dataInfoNT(*BoxPlotter.constructDataInfo("025", "F")),
            521: self.dataInfoNT(*BoxPlotter.constructDataInfo("025", "G")),
            522: self.dataInfoNT(*BoxPlotter.constructDataInfo("025", "D")),
            523: self.dataInfoNT(*BoxPlotter.constructDataInfo("025", "E")),
            524: self.dataInfoNT(*BoxPlotter.constructDataInfo("026", "F")),
            525: self.dataInfoNT(*BoxPlotter.constructDataInfo("026", "G")),
            526: self.dataInfoNT(*BoxPlotter.constructDataInfo("026", "D")),
            527: self.dataInfoNT(*BoxPlotter.constructDataInfo("026", "E")),
            528: self.dataInfoNT(*BoxPlotter.constructDataInfo("027", "F")),
            529: self.dataInfoNT(*BoxPlotter.constructDataInfo("027", "G")),
            530: self.dataInfoNT(*BoxPlotter.constructDataInfo("027", "D")),
            531: self.dataInfoNT(*BoxPlotter.constructDataInfo("027", "E")),
            532: self.dataInfoNT(*BoxPlotter.constructDataInfo("028", "F")),
            533: self.dataInfoNT(*BoxPlotter.constructDataInfo("028", "G")),
            534: self.dataInfoNT(*BoxPlotter.constructDataInfo("028", "D")),
            535: self.dataInfoNT(*BoxPlotter.constructDataInfo("028", "E")),
            536: self.dataInfoNT(*BoxPlotter.constructDataInfo("029", "F")),
            537: self.dataInfoNT(*BoxPlotter.constructDataInfo("029", "G")),
            538: self.dataInfoNT(*BoxPlotter.constructDataInfo("029", "D")),
            539: self.dataInfoNT(*BoxPlotter.constructDataInfo("029", "E")),
            540: self.dataInfoNT(*BoxPlotter.constructDataInfo("030", "D")),
            541: self.dataInfoNT(*BoxPlotter.constructDataInfo("030", "E")),
            542: self.dataInfoNT(*BoxPlotter.constructDataInfo("030", "F")),
            543: self.dataInfoNT(*BoxPlotter.constructDataInfo("030", "G")),
            544: self.dataInfoNT(*BoxPlotter.constructDataInfo("031", "D")),
            545: self.dataInfoNT(*BoxPlotter.constructDataInfo("031", "E")),
            546: self.dataInfoNT(*BoxPlotter.constructDataInfo("031", "F")),
            547: self.dataInfoNT(*BoxPlotter.constructDataInfo("031", "G")),
            548: self.dataInfoNT(*BoxPlotter.constructDataInfo("032", "D")),
            549: self.dataInfoNT(*BoxPlotter.constructDataInfo("032", "E")),
            550: self.dataInfoNT(*BoxPlotter.constructDataInfo("032", "F")),
            551: self.dataInfoNT(*BoxPlotter.constructDataInfo("032", "G")),
            552: self.dataInfoNT(*BoxPlotter.constructDataInfo("033", "F")),
            553: self.dataInfoNT(*BoxPlotter.constructDataInfo("033", "G")),
            554: self.dataInfoNT(*BoxPlotter.constructDataInfo("033", "D")),
            555: self.dataInfoNT(*BoxPlotter.constructDataInfo("033", "E")),
        }
        self.df_agg = pd.DataFrame("", index=sorted(self.data_info_dict.keys()),columns=["sbj", "session"])
        self.dfs_by_param = \
        {etype:
            {param:
                {meanstd: pd.DataFrame(np.nan, index=self.vibration_conditions, columns=self.cognitive_conditions) 
                    for meanstd in self.meanstd_list
                }
                for param in self.param_list}
            for etype in self.etype_list
        }
        self.dfs_ratio = \
        {etype:
            {meanstd: pd.DataFrame(np.nan, index=self.param_list, columns=self.ratio_list) 
                for meanstd in self.meanstd_list
            }
            for etype in self.etype_list
        }
        # Insert subject info columns
        for trial_id in self.data_info_dict.keys():
            self.df_agg.at[trial_id, "sbj"]     = self.data_info_dict[trial_id].sbj
            self.df_agg.at[trial_id, "session"] = self.data_info_dict[trial_id].session

        # Initialize the DataFrame for spss anaylsis
        self.dfs_spss = \
        {etype:
            {param:
                pd.DataFrame("", index=np.unique(self.df_agg.loc[:, "sbj"]),
                    columns=["Gait_Parameter", "SbjID"] + self.session_list)
                for param in self.param_list
            }
            for etype in self.etype_list
        }
        

    def process_trial(self, trial_id):
        pkl_filename = os.path.join(self.ws_path, self.pkl_folder, "data" + str(trial_id).rjust(3, '0') + ".pkl")
        with open(pkl_filename, "rb") as pkl_file:
            STEP_KINECT = pickle.load(pkl_file)

            def get_csv_filename(type):
                sbj_session_name = self.data_info_dict[trial_id].sbj + self.data_info_dict[trial_id].session
                return os.path.join(self.ws_path, "by_trial", type + "_" + sbj_session_name + ".csv")

            # Calculate the Coefficient of Variation as the ratio of the standard deviation to 
            # the mean times 100. Save the raw data by trial as well to csv files
            df_stride = pd.DataFrame(np.nan, index=range(0, len(STEP_KINECT.stride_table_sorted)), columns=[])
            for i, row in enumerate(STEP_KINECT.stride_table_sorted):
                df_stride.at[i, 'ts_FC'] = getattr(row, 'ts_FC')
                df_stride.at[i, 'LR'] = getattr(row, 'LR')
            for field in self.stride_params:
                for i, row in enumerate(STEP_KINECT.stride_table_sorted):
                    df_stride.at[i, field] = getattr(row, field)
                field_values = df_stride.loc[:, field]
                etype_field = "_".join([self.etype_list[0], field]) # CV
                self.df_agg.at[trial_id, etype_field] = np.std(field_values) / np.mean(field_values) * 100
                etype_field = "_".join([self.etype_list[1], field]) # value
                self.df_agg.at[trial_id, etype_field] = np.mean(field_values)
            csv_filename_out = get_csv_filename("stride")
            if not args.skip_csv:
                df_stride.to_csv(csv_filename_out, index=False, float_format='%.4f')
                print("Saved to: " + csv_filename_out)

            df_step = pd.DataFrame(np.nan, index=range(0, len(STEP_KINECT.step_table_sorted)), columns=[])
            for i, row in enumerate(STEP_KINECT.step_table_sorted):
                df_step.at[i, 'ts_FC'] = getattr(row, 'ts_FC')
                df_step.at[i, 'LR'] = getattr(row, 'LR')
            for field in self.step_params:
                for i, row in enumerate(STEP_KINECT.step_table_sorted):
                    df_step.at[i, field] = getattr(row, field)
                field_values = df_step.loc[:, field]
                etype_field = "_".join([self.etype_list[0], field])
                self.df_agg.at[trial_id, etype_field] = np.std(field_values) / np.mean(field_values) * 100
                etype_field = "_".join([self.etype_list[1], field]) # value
                self.df_agg.at[trial_id, etype_field] = np.mean(field_values)
            csv_filename_out = get_csv_filename("step")
            if not args.skip_csv:
                df_step.to_csv(csv_filename_out, index=False, float_format='%.4f')
                print("Saved to: " + csv_filename_out)


    def update_and_save_csv(self):
        # Calculate Mean
        df_mean = self.df_agg.mean()
        df_mean.name = "mean"
        # Create aggregate table with mean
        self.df_with_mean = self.df_agg.append(df_mean)
        # Write to csv
        csv_filename_out = os.path.join(self.ws_path, "robot_cv.csv")
        if not args.skip_csv:
            self.df_with_mean.to_csv(csv_filename_out, index=True, float_format='%.4f')
            print("Aggregate table saved to: " + csv_filename_out)

    def plotChart(self):
        # Process csv by session
        for etype in self.etype_list:
            for param in self.param_list:
                df_sessions = pd.DataFrame(np.nan, index=self.sbj_list, columns=self.session_list)
                df_ratios   = pd.DataFrame(np.nan, index=self.sbj_list, columns=self.ratio_list)
                field = "_".join([etype, param])
                # Find col that contains field
                for col in list(self.df_agg):
                    if field in col:
                        for trial_id, data_info in self.data_info_dict.items():
                            sbj = data_info.sbj
                            session = data_info.session
                            df_sessions.at[sbj, session] = self.df_agg.at[trial_id, col]
                            self.dfs_spss[etype][param].at[sbj, session] = "{:.4f}".format(self.df_agg.at[trial_id, col])
                            self.dfs_spss[etype][param].at[sbj, "SbjID"] = sbj.title()
                for session in list(df_sessions):
                    vib_cond = self.vibration_dict[session]
                    cog_cond = self.cognitive_dict[session]
                    self.dfs_by_param[etype][param]["mean"].at[vib_cond, cog_cond] = df_sessions.loc[:, session].mean()
                    self.dfs_by_param[etype][param]["std"].at[vib_cond, cog_cond]  = df_sessions.loc[:, session].std()
                    self.dfs_by_param[etype][param]["se"].at[vib_cond, cog_cond] = \
                        df_sessions.loc[:, session].std() / np.math.sqrt(len(df_sessions.loc[:, session]))

                # Calculate ratio
                df_ratios.loc[:, "E/D"] = df_sessions.loc[:, "E"] / df_sessions.loc[:, "D"]
                df_ratios.loc[:, "G/F"] = df_sessions.loc[:, "G"] / df_sessions.loc[:, "F"]
                for ratio_type in list(df_ratios):
                    self.dfs_ratio[etype]["mean"].at[param, ratio_type] = df_ratios.loc[:, ratio_type].mean()
                    self.dfs_ratio[etype]["std"].at[param, ratio_type]  = df_ratios.loc[:, ratio_type].std()
                    self.dfs_ratio[etype]["se"].at[param, ratio_type] = \
                        df_ratios.loc[:, ratio_type].std() / np.math.sqrt(len(df_ratios.loc[:, ratio_type]))
                
                # paired t-test
                print("{:8s} {:7s}, p-2samp(D;E)={:.4f}, p-2samp(F;G)={:.4f}, p-2samp(D/E, F/G)={:.4f}, "\
                    "p-1samp(D/E)={:.4f}, p-1samp(F/G)={:.4f}".format(param, etype,
                    scipy.stats.ttest_ind(df_sessions.loc[:, "D"], df_sessions.loc[:, "E"])[1],
                    scipy.stats.ttest_ind(df_sessions.loc[:, "F"], df_sessions.loc[:, "G"])[1],
                    scipy.stats.ttest_ind(df_ratios.loc[:, "E/D"], df_ratios.loc[:, "G/F"])[1],
                    scipy.stats.ttest_1samp(df_ratios.loc[:, "E/D"], 1.0)[1],
                    scipy.stats.ttest_1samp(df_ratios.loc[:, "G/F"], 1.0)[1]))


        for etype in self.etype_list:
            df_spss = pd.DataFrame()
            for param, param_spss in [
                    ("StepL", "_".join(["Step_Length", etype])), 
                    ("StrideL", "_".join(["Stride_Length", etype])),
                    ("StepW", "_".join(["Stride_Width", etype])),
                    ("StrideT", "_".join(["Stride_Time", etype])),
                    ("StrideV", "_".join(["Stride_Velocity", etype]))
                ]:
                self.dfs_spss[etype][param].loc[:, "Gait_Parameter"] = param_spss
                df_spss = df_spss.append(self.dfs_spss[etype][param], ignore_index=True)
            cols = df_spss.columns.tolist()
            cols = cols[0:3] + [cols[4], cols[3], cols[5]]
            df_spss = df_spss[cols]
            column_name_map = {session: self.cognitive_dict[session] + "_" + self.vibration_dict[session]
                                for session in self.session_list}
            df_spss.rename(columns=column_name_map, inplace=True)
            spss_filename = os.path.join(self.ws_path, "robot_spss_" + etype + ".csv")
            if not args.skip_csv:
                df_spss.to_csv(spss_filename, index=False, float_format='%.3f')
                print("SPSS table saved to: " + spss_filename)

        # Save csv and jpg for self.dfs_by_param
        for etype in self.etype_list:
            for param in self.param_list:
                for meanstd in self.meanstd_list:
                    csv_filename = os.path.join(self.ws_path, 
                        "by_param", param + "_" + etype + "_" + meanstd + ".csv")
                    if not args.skip_csv:
                        self.dfs_by_param[etype][param][meanstd].to_csv(csv_filename)
                        print(param + " saved to: " + csv_filename)

                # Plot
                plt.figure()
                self.dfs_by_param[etype][param]["mean"].plot(kind="bar", capsize=4,
                    rot=0, title=param + ' ' + etype + ' ({:s})'.format(self.data_source_robot), 
                    yerr = self.dfs_by_param[etype][param]["se"])
                if etype == "CV":
                    plt.ylabel(" ".join([etype, "(%)"]))
                else:
                    plt.ylabel(" ".join([param, self.unit_dict[param]]))
                fig_filename = os.path.join(self.ws_path, "by_param", param + "_" + etype + ".jpg")
                plt.savefig(fig_filename)
                print(param + " saved to: " + fig_filename)

        # Save csv and jpg for self.dfs_ratio
        for etype in self.etype_list:
            for meanstd in self.meanstd_list:
                csv_filename = os.path.join(self.ws_path, 
                    "cv", "robot_" + etype + "_ratio_" + meanstd + ".csv")
                if not args.skip_csv:
                    self.dfs_ratio[etype][meanstd].to_csv(csv_filename)
                    print("Robot cv_ratio saved to: " + csv_filename)

            # Plot
            plt.figure()
            ax = self.dfs_ratio[etype]["mean"].plot(kind="bar", capsize=4,
                rot=0, title=etype + ' Ratio', 
                yerr = self.dfs_ratio[etype]["se"])
            plt.xticks(range(len(self.ratio_xticks)), self.ratio_xticks)
            ax.legend(['VOFF', 'VON'])
            plt.ylabel("Ratio")
            for fig_ext in [".jpg", ".eps"]:
                fig_filename = os.path.join(self.ws_path, "cv", etype + " ratio" + fig_ext)
                plt.savefig(fig_filename)
                print("Robot cv_ratio saved to: " + fig_filename)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--skip-csv", action='store_true')
    parser.add_argument("-c", "--combined", action='store_true')
    args = parser.parse_args()

    plotter = BoxPlotter(args.combined)
    for trial_id in sorted(plotter.data_info_dict.keys()):
    # for trial_id in [424, 425]:
        plotter.process_trial(trial_id)
    plotter.update_and_save_csv()
    plotter.plotChart()