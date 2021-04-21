## Generate the waypoint files
```bash
rosrun gait_training_robot generate_waypoints.py
```

## Map the Environment
1. Start rtabmap in mapping mode along with the robot.
```bash
roslaunch gait_training_robot rtabmap_map.launch
```

2. Start goal_generator
```bash
rosrun gait_training_robot goal_generator _stop_upon_completion:=false _max_num_laps:=1 _suffix:=_cw
rosrun gait_training_robot goal_generator _stop_upon_completion:=false _max_num_laps:=1 _suffix:=_ccw
```

## Start data collection

1. Make sure the trial id is consistent between Optitrack and the robot system.
```bash
roslaunch gait_training_robot record_cw.launch bag_name:=data243
roslaunch gait_training_robot record_ccw.launch bag_name:=data243
```