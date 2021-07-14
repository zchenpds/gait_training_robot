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
pattern=data[0-9]*_*.bag
exists=false

for bag_name in $pattern
do
    [ -e $bag_name ] || continue
    printf "%-32s ->  %s\n" $bag_name ${bag_name:0:7}.bag
    exists=true
done

$exists || { echo "No file found." && exit 1 ; }

read -p "Continue? (Y/N): " confirm && [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]] || exit 1

for bag_name in $pattern
do
    [ -e $bag_name ] || continue
    mv $bag_name ${bag_name:0:7}.bag
done