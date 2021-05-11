#!/bin/bash
IFS=:
for f in $ROS_PACKAGE_PATH; do
    gta_path="$f/gait_training_robot"
    bag_path="${gta_path}/bags/optitrack"
    if [ -e $bag_path ]; then
        cd $bag_path
        printf "Parsing bag files at %s\n" $(pwd)
        break
    fi
done

> baginfo
for bag_name in data*.bag
do
    echo $bag_name
    rosbag info --freq $bag_name >> baginfo
    printf "\n" >> baginfo
done

printf "Created %s\n" $bag_path/baginfo