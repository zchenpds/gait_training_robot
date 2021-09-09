#! /usr/bin/python

import os
from collections import namedtuple
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pickle

class BoxPlotter:
    dataInfoNT = namedtuple("dataInfoNT", ["sbj", "session"])
    ws_path = "/home/ral2020/Documents/sunny"
    session_list = ["D", "E", "F", "G"]
    sbj_list = []
    param_list = ["Dist"]
    unit_list  = ["(cm.)", "(cm.)", "(%)", "(%)"]
    etype_list = ["MAE", "ESD", "MAE Percentage", "ESD Percentage"]
    meanstd_list = ["mean", "std", "se"]

    @classmethod
    def constructDataInfo(cls, sbj, session):
        sbj = "SBJ" + sbj
        return (sbj, session.upper())

    def __init__(self):
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
        pkl_filename = os.path.join(self.ws_path, "localization", "data" + str(trial_id).rjust(3, '0') + ".pkl")
        with open(pkl_filename, "rb") as pkl_file:
            [_, dist_rmse, dist_error_sd] = pickle.load(pkl_file)
            
            # Populate dataframe df_agg by trial_id
            self.df_agg.at[trial_id, "MAE_"  + self.param_list[0]] = dist_rmse
            self.df_agg.at[trial_id, "ESD_"  + self.param_list[0]] = dist_error_sd
            desired_dist = 1.5
            self.df_agg.at[trial_id, "MAE Percentage_" + self.param_list[0]] = dist_rmse / desired_dist
            self.df_agg.at[trial_id, "ESD Percentage_" + self.param_list[0]] = dist_error_sd / desired_dist


    def update_and_save_csv(self):
        # Calculate Mean
        df_mean = self.df_agg.mean()
        df_mean.name = "mean"
        # Create aggregate table with mean
        self.df_with_mean = self.df_agg.append(df_mean)
        # Write to csv
        csv_filename_out = os.path.join(self.ws_path, "Human_robot_distance.csv")
        self.df_with_mean.to_csv(csv_filename_out, index=True, float_format='%.4f')
        print("Aggregate table saved to: " + csv_filename_out)

    def plotChart(self):
        # Populate dataframe df_sessions by session
        data_sources = ["robot"]
        for etype in self.etype_list:
            for param in self.param_list:
                df_sessions = {session: pd.DataFrame(np.nan, index=self.sbj_list, columns=data_sources)
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
                            

        for etype, unit in zip(self.etype_list, self.unit_list):
            for param in self.param_list:
                for meanstd in self.meanstd_list:
                    csv_filename = os.path.join(self.ws_path, 
                        "by_param", param + "_" + etype + "_" + meanstd + ".csv")
                    self.dfs_by_param[etype][param][meanstd].to_csv(csv_filename)
                    print(param + " saved to: " + csv_filename)

                # Plot
                plt.figure()
                self.dfs_by_param[etype][param]["mean"].plot(kind="bar", capsize=4, legend=False,
                    rot=0, title="Human-Robot Distance " + etype, yerr = self.dfs_by_param[etype][param]["se"])
                plt.ylabel(" ".join([etype, unit]))
                fig_filename = os.path.join(self.ws_path, "by_param", param + "_" + etype + ".jpg")
                plt.savefig(fig_filename)
                print(param + " saved to: " + fig_filename)

if __name__ == "__main__":
    plotter = BoxPlotter()
    for trial_id in sorted(plotter.data_info_dict.keys()):
    # for trial_id in [424, 425]:
        plotter.process_trial(trial_id)
    plotter.update_and_save_csv()
    plotter.plotChart()