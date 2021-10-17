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
    vibration_conditions = ["VOFF", "VON"] # rows (index)
    vibration_dict = {"D": "VOFF", "E": "VOFF", "F": "VON", "G": "VON"}
    cognitive_conditions = ["COFF", "CON"] # columns
    cognitive_dict = {"D": "COFF", "E": "CON", "F": "COFF", "G": "CON"}
    sbj_list = []
    param_list = ["Vel"]
    unit_list  = ["(cm/s)", "(cm/s)"]
    etype_list = ["MAE", "ESD"]
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
                {meanstd: pd.DataFrame(np.nan, index=self.vibration_conditions, columns=self.cognitive_conditions) 
                    for meanstd in self.meanstd_list
                }
                for param in self.param_list}
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
                pd.concat([
                    pd.DataFrame("", index=np.unique(self.df_agg.loc[:, "sbj"]),
                        columns=["SbjID"]),
                    pd.DataFrame(np.nan, index=np.unique(self.df_agg.loc[:, "sbj"]),
                        columns=self.session_list),
                ], axis=1)                
                for param in self.param_list
            }
            for etype in self.etype_list
        }
        
    def process_trial(self, trial_id):
        pkl_filename = os.path.join(self.ws_path, "localization", "data" + str(trial_id).rjust(3, '0') + ".pkl")
        with open(pkl_filename, "rb") as pkl_file:
            [_, _, _, vel_mae, vel_esd] = pickle.load(pkl_file)
            
            # Populate dataframe df_agg by trial_id
            self.df_agg.at[trial_id, "MAE_"  + self.param_list[0]] = vel_mae
            self.df_agg.at[trial_id, "ESD_"  + self.param_list[0]] = vel_esd


    def update_and_save_csv(self):
        # Calculate Mean
        df_mean = self.df_agg.mean()
        df_mean.name = "mean"
        # Create aggregate table with mean
        self.df_with_mean = self.df_agg.append(df_mean)
        # Write to csv
        csv_filename_out = os.path.join(self.ws_path, "Velocity_error.csv")
        self.df_with_mean.to_csv(csv_filename_out, index=True, float_format='%.4f')
        print("Aggregate table saved to: " + csv_filename_out)

    def plotChart(self):
        # Populate dataframe df_sessions by session
        for etype in self.etype_list:
            for param in self.param_list:
                df_sessions = pd.DataFrame(np.nan, index=self.sbj_list, columns=self.session_list)
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
                for session in df_sessions.keys():
                    vib_cond = self.vibration_dict[session]
                    cog_cond = self.cognitive_dict[session]
                    self.dfs_by_param[etype][param]["mean"].at[vib_cond, cog_cond] = df_sessions.loc[:, session].mean()
                    self.dfs_by_param[etype][param]["std"].at[vib_cond, cog_cond]  = df_sessions.loc[:, session].std()
                    self.dfs_by_param[etype][param]["se"].at[vib_cond, cog_cond] = \
                        df_sessions.loc[:, session].std() / np.math.sqrt(len(df_sessions.loc[:, session]))
                            
        df_spss = self.dfs_spss[etype][param].loc[:, "SbjID"]
        prefix = "Velocity_"
        for etype in self.etype_list:
            for param in self.dfs_spss[etype].keys():
                df_temp = self.dfs_spss[etype][param].loc[:, "D":].copy() / 100.0
                df_temp[prefix + etype] = df_temp.mean(axis=1)
                df_spss = pd.concat([df_spss, df_temp], 1)
            column_name_map = {session: "Velocity_" + session + "_" + etype
                                for session in self.session_list}
            df_spss.rename(columns=column_name_map, inplace=True)
        spss_filename = os.path.join(self.ws_path, "Velocity_error.csv")
        df_spss.to_csv(spss_filename, index=False, float_format='%.3f')
        print("SPSS table saved to: " + spss_filename)

        for etype, unit in zip(self.etype_list, self.unit_list):
            for param in self.param_list:
                for meanstd in self.meanstd_list:
                    csv_filename = os.path.join(self.ws_path, 
                        "by_param", param + "_" + etype + "_" + meanstd + ".csv")
                    self.dfs_by_param[etype][param][meanstd].to_csv(csv_filename)
                    print(param + " saved to: " + csv_filename)

                # Plot
                plt.figure()
                self.dfs_by_param[etype][param]["mean"].plot(kind="bar", capsize=4, legend=True,
                    rot=0, title=("Velocity " + etype).replace("_", " "),
                    yerr = self.dfs_by_param[etype][param]["se"])
                plt.ylabel(" ".join([etype, unit]))
                for fig_ext in [".jpg", ".eps"]:
                    fig_filename = os.path.join(self.ws_path, "by_param", param + "_" + etype + fig_ext)
                    plt.savefig(fig_filename)
                    print(param + " saved to: " + fig_filename)

if __name__ == "__main__":
    plotter = BoxPlotter()
    for trial_id in sorted(plotter.data_info_dict.keys()):
    # for trial_id in [424, 425]:
        plotter.process_trial(trial_id)
    plotter.update_and_save_csv()
    plotter.plotChart()