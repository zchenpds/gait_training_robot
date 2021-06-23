## Sync csv files from external drive
```bash
$ rsync -avu /media/ral2020/SP\ PHD\ U3/Session\ 2020-12-21\ processed/ ~/catkin_ws/src/gait_training_robot/optitrack/csv/
```

## Generate bag files from OptiTrack csv files
```bash
$ rosrun gait_training_robot convert_optitrack_to_bag.py <bag_number> [-l <number_of_bags>]
```