#!/bin/bash

# IFS=:
# for f in $ROS_PACKAGE_PATH; do
#     gta_path="$f/gait_training_robot"
#     bag_path="${gta_path}/bags"
#     if [ -e $bag_path ]; then
#         cd $bag_path
#         printf "Going over files at %s\n" $(pwd)
#         break
#     fi
# done

# pwd

# pattern=data[0-9]{3}_202*.bag
pattern=SBJ*/zeno_*.xlsx
exists=false

for xlsx_name in $pattern
do
    [ -e $xlsx_name ] || continue
    printf "%-32s ->  %s\n" $xlsx_name ${xlsx_name:0:7}Sbj${xlsx_name:13:3}_${xlsx_name:21:1}_GaitParameters.csv
    exists=true
done

$exists || { echo "No file found." && exit 1 ; }

read -p "Continue? (Y/N): " confirm && [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]] || exit 1

for xlsx_name in $pattern
do
    [ -e $xlsx_name ] || continue
    libreoffice --headless --convert-to csv $xlsx_name
    mv ${xlsx_name:7:-5}.csv ${xlsx_name:0:7}Sbj${xlsx_name:13:3}_${xlsx_name:21:1}_GaitParameters.csv
done