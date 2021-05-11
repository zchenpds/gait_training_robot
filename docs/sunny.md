## Mapping
1. On the robot computer (192.168.1.100) (optionally ssh from the remote computer):
    1. Generate waypoints for the environment.
        ```bash
        $ rosrun gait_training_robot generate_waypoints.py sunny
        ```
    1. Start roscore
    1. Run the following
        ```bash
        $ roslaunch gait_training_robot rtabmap_map.launch disable_rviz:=true
        $ rosrun gait_training_robot goal_generator _suffix:=_ccw _max_num_laps:=1 _preview:=false _stop_upon_completion:=false
        $ rosrun gait_training_robot goal_generator _suffix:=_cw _max_num_laps:=1 _preview:=false _stop_upon_completion:=false
        ```

## Data Collection
1. On the robot computer (192.168.1.100):
    1. Start roscore
    1. Run the following (cannot be from ssh)
        ```bash
        $ roslaunch gait_training_robot sunny_record_ccw.launch
        ```
2. On the remote computer (192.168.1.102):
    1. Run the following and select the bag file to play back
        ```bash
        $ roslaunch gait_training_robot sunny_play_remote.launch
        ```
    1. Start plotjuggler and load configuration "mos_only.xml"
        ```bash
        $ rosrun plotjuggler plotjuggler
        ```

## Data Playback
1. On the robot computer (192.168.1.100) (optionally ssh from the remote computer):
    1. Start roscore
    1. Run the following and select the bag file to play back
        ```bash
        $ rosrun gait_training_robot sunny_play.py
        ```
2. On the remote computer (192.168.1.102):
    1. Run the following and select the bag file to play back
        ```bash
        $ roslaunch gait_training_robot sunny_play_remote.launch
        ```
    1. Start plotjuggler and load configuration "mos_only.xml"
        ```bash
        $ rosrun plotjuggler plotjuggler
        ```

## Network
### IP addresses
|IP            | hostname   |  Description
|--------------|------------|------------------
|192.168.1.254 |            |  Router, SSID: schgta
|192.168.1.100 | ral2020    |  Robot computer, HP-omen
|192.168.1.11  |            |  Left insole
|192.168.1.12  |            |  Right insole
|192.168.1.102 | ral        |  Remote computer 1, Thinkpad P51, wired connection


### Other
* Fix WAN routing issue
```bash
$ ip route list
$ ip route delete default
```

## Sampling period body tracking

```bash
$ rostopic echo -b data_2021-05-07-22-45-18.bag -p /body_tracking_data/markers[0]/header/stamp > ~/.ros/tmp
```