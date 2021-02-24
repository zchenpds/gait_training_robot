#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscd gait_training_robot/matlab/mat/
#{222..229}

suffix=b
for i in {226..229}
do
   cp data${i}${suffix}.mat ~/projects/gta_data/ros_data/
   cp data${i}${suffix}_imu.mat ~/projects/gta_data/ros_data/
done