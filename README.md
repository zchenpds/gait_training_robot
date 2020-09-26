# gait_training_robot
This ROS package provides a suite of software that performs gait-training tasks such as engaging people in walkinging exercise, analyzing people's gait, and providing corrective feedback. Some launch files in this package are a good starting point to get [P3-DX robot](https://www.generationrobots.com/media/Pioneer3DX-P3DX-RevA.pdf) with a Kinect sensor to perform some generic SLAM/localization/planning tasks. The development of this package is ongoing.
```bash
git clone --recurse-submodules https://github.com/zchenpds/gait_training_robot.git
```

## Prerequisites
- [P3-DX robot](https://www.generationrobots.com/media/Pioneer3DX-P3DX-RevA.pdf)
- Ubuntu 18.04
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Aria](https://github.com/zchenpds/Aria.git) MobileRobots' Advanced Robot Interface for Applications (ARIA) is a C++ library/SDK for all MobileRobots/ActivMedia platforms.
- [rosaria](https://github.com/amor-ros-pkg/rosaria) (A ROS wrapper for ARIA)
- [sport_sole](https://github.com/zchenpds/sport_sole) A ROS node for communicating with the insole sensor.

## Checklist for Kinect v1
1. Install pointcloud_to_laserscan. This package converts pointcloud data to laserscan data so that gmapping can use it to build a map. Required only if gmapi
  ```bash
  sudo apt-get install ros-melodic-pointcloud-to-laserscan
  ```
2. Install openni_launch. This is the primary driver for Kinect v1 sensor.
  ```bash
  sudo apt-get install ros-melodic-openni-launch
  ```
3. Install gmapping. (optional, alternative to rtabmap)
  ```bash
  sudo apt-get install ros-melodic-gmapping
  ```
4. Install hector_trajectory_server (optional, this package is only available for ROS Kinetic)
  ```bash
  sudo apt-get install ros-kinetic-hector-trajectory-server
  ```
5. Install move_base. This package takes a map published by either rtabmap or gmapping and computes the viable path to a specified goal.
  ```bash
  sudo apt-get install ros-melodic-move-base
  ```
6. Install rtabmap_ros. This pacakge does 3D SLAM and builds the map to be used by the motion planner.
  ```bash
  sudo apt-get install ros-melodic-rtabmap-ros
  ```
7. Install gsl
  ```bash
  sudo apt install libgsl-dev
  ```

## Checklist for the robot
1. Install libaria from source. Package rosaria is dependent on it.
   ```bash
   cd ~/projects
   git clone https://github.com/zchenpds/Aria.git
   cd Aria
   make
   sudo make install
   echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Aria/lib' >> ~/.bashrc
   sudo ldconfig
   ```
2. Download and `catkin_make` a forked source of rosaria
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/zchenpds/rosaria.git
   ```
3. Are you a member of the dialout group? If not, run 
   ```bash
   sudo usermod -a -G dialout $USER 
   ```
4. Ensure you have read and write access to the COM port by running 
   ```bash
   sudo chmod a+rw /dev/ttyUSB0
   ```
5. Test it. 
   ```bash
   rosrun rosaria RosAria
   ```

## Checklist for the gait analyzer
1. Install [Azure Kinect ROS Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver) from source. [Azure Kinect DK](https://docs.microsoft.com/en-us/azure/Kinect-dk/) sensor faces backward and monitors the body movement of the human following the robot.
  - For Azure Kinect Body Tracking SDK 1.0.1, running the following commands in a terminal would install both the device driver and the body tracking sdk.
    ```bash
    curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
    sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
    sudo apt-get update
    sudo apt install libk4abt1.0-dev
    sudo apt install k4a-tools=1.3.0
    ```
  - Remember to update udev rules by copying [this file](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/release/1.4.x/scripts/99-k4a.rules) to `/etc/udev/rules.d/`, according to [this](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/release/1.4.x/docs/usage.md#linux-device-setup).
    ```bash
    cd /etc/udev/rules.d/
    sudo wget https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/release/1.4.x/scripts/99-k4a.rules
    ```

## Two-machine configuration
Two-machine configuration consists of a desktop computer, a robot computer and wifi router. Both machines are connected to the router to allow for faster communication. To remotely monitor and control the robot computer from the desktop computer, we need to set up ssh and some ROS parameters. The steps for setting up ssh are as follows:
1. Ensure both machines have the same Ubuntu version and ROS version.
1. If you do not have the SSH key generated yet, run `ssh-keygen` on the desktop PC. Press enter to use the default path to save the key to. Press enter again to use no passphrase.
1. Run `ssh-copy-id robotname@192.168.1.102` to copy the generated key to the remote machine, where "robotname" is username of the onboard laptop and "192.168.1.102" is the IP address of the robot computer. IP address of either machine can be found by running `hostname -I`.
1. The previous steps need to be done only once. In the future, to start an ssh session, run `ssh robotname@192.168.1.102`

Follow these steps to set up the relevant ROS parameters
1. Add `export ROS_IP=$(hostname -I)` to the  `~/.bashrc` file of each of the machines.
1. On the desktop computer, add `export ROS_MASTER_URI=http://MASTER_IP:11311/` to `~/.bashrc`.
1. Remember to restart the terminals in order for the changes to take effect.

# Examples

1. SLAM using gmapping
  - **One Machine**. run `roslaunch gait_training_robot test1_slam_headless.launch`.
  - **Two Machines**. 
    1. On the robot computer (master), run `roslaunch gait_training_robot test1_slam_headless.launch`.
    2. On the desktop computer, run `roslaunch gait_training_robot test1_slam_rviz.launch`.
    3. Note that if kinect2_bridge package is started via ssh, it is necessary to tell GLFW to be launched on a server screen by running `export DISPLAY=:0` after `ssh`ing to the server.
    ![alt text](images/screenshot1.png)

2. Planning using gmapping
  - **Two Machines**.
    1. On the robot computer (master), run `roslaunch gait_training_robot test2_plan_headless.launch`.
    2. On the desktop computer, run `roslaunch gait_training_robot test1_slam_rviz.launch`.

3. SLAM using rtabmap_ros
  - **Two Machines**.
    1. On the robot computer (master), run `roslaunch gait_training_robot test1_rtabmap_headless.launch localization:=false enable_distance_controller:=false`.
    2. On the desktop computer, start rviz by running `roslaunch gait_training_robot test1_rviz_rtabmap.launch`.
4. Localization using rtabmap
  - **Two Machines**. Requires a map built by rtabmap.
    1. On the robot computer (master), run `roslaunch gait_training_robot test1_rtabmap_headless.launch localization:=true enable_distance_controller:=false`.
    2. On the desktop computer, start rviz by running `roslaunch gait_training_robot test1_rviz_rtabmap.launch`.
    3. To record waypoints on the robot computer, start a new terminal and run `rostopic echo /move_base_simple/goal > ~/catkin_ws/src/gait_training_robot/data/waypoints.yaml` on the robot computer, and publish goals from rviz. 
    4. To play back waypoints that have been recorded on the robot computer, run `rosrun gait_training_robot goal_generator`.
    5. Sometimes, the robot maybe stuck somewhere because it mistakenly perceives an obstacle which does not exist. In this case, we want to clear the cost map by running `rosservice call /move_base/clear_costmaps "{}"`.
5. Human distance controller
  - **Three Machines**. Requires a map built by rtabmap. Requires waypoints to be recorded.
    1. On the robot computer (master), run `roslaunch gait_training_robot test1_rtabmap_headless.launch localization:=true enable_distance_controller:=true`.
    2. On the desktop computer, start rviz by running `roslaunch gait_training_robot test1_rviz_rtabmap.launch`.
    3. On the Windows computer, start `BodyTrackerAzure.exe`.
    4. To play back waypoints that have been recorded on the robot computer, run `rosrun gait_training_robot goal_generator`.
6. Validate urdf
  - **One Machine**. After changes are made to `urdf/*.urdf.xacro` files, use `catkin_make` to generate the urdf files.
    1. Run `roslaunch gait_training_robot rviz+urdf_state.launch`.
    
    ![alt text](images/screenshot2.png)

7. Test Kalman filter
  `roslaunch gait_training_robot test4_kalman_filter.launch record_bag:=true`
  `roslaunch gait_training_robot test4_kalman_filter.launch play_bag:=true bag_name:=data_2019-12-03-21-05-58.bag`

8. Teleoperate using wireless keyboard.
  `roslaunch gait_training_robot robot.launch enable_teleop:=true`

9. Data collection
  - **Straight walking**. The orientation of the robot will remain constant. Open-loop robot motion control in effect. The endpoint must have an obstacle that can be detected by the SONAR ring. 
    ```bash
    roslaunch gait_training_robot test5_comkf.launch straight_line_mode:=true
    ```
  - **Curved walking**. The environment must have been mapped. File waypoints.xml will be loaded and goal_generator will publish the goals in sequence.
    ```bash
    roslaunch gait_training_robot test5_comkf.launch
    ```
  
9. Temporary test
  `roslaunch gait_training_robot test5_comkf.launch record_bag:=false play_bag:=true bag_name:=new/data028 enable_gait_analyzer:=true record_gait_analytics:=false`
  `roslaunch gait_training_robot test5_comkf.launch record_bag:=true play_bag:=true bag_name:=new/data028 enable_gait_analyzer:=true record_gait_analytics:=true`

  # ROS Topics

Published topics:
 * /gait_analyzer/pcom_pelvis_measurement [geometry_msgs/PointStamped] 1 publisher
 * /gait_analyzer/cop [geometry_msgs/PointStamped] 1 publisher
 * /tf [tf2_msgs/TFMessage] 3 publishers
 * /clicked_point [geometry_msgs/PointStamped] 1 publisher
 * /odom [nav_msgs/Odometry] 1 publisher
 * /move_base_simple/goal [geometry_msgs/PoseStamped] 1 publisher
 * /gait_analyzer/pcom_vel_estimate [geometry_msgs/Vector3Stamped] 1 publisher
 * /gait_analyzer/foot_pose_measurement_r [geometry_msgs/PointStamped] 1 publisher
 * /gait_analyzer/foot_pose_measurement_l [geometry_msgs/PointStamped] 1 publisher
 * /tf_static [tf2_msgs/TFMessage] 2 publishers
 * /gait_analyzer/mos_value_measurement1 [std_msgs/Float64] 1 publisher
 * /imu [sensor_msgs/Imu] 1 publisher
 * /sport_sole_publisher/sport_sole [sport_sole/SportSole] 1 publisher
 * /gait_analyzer/bos [geometry_msgs/PolygonStamped] 1 publisher
 * /gait_analyzer/pcom_pos_measurement [geometry_msgs/PointStamped] 1 publisher
 * /cmd_vel [geometry_msgs/Twist] 1 publisher
 * /gait_analyzer/xcom_estimate [geometry_msgs/PointStamped] 1 publisher
 * /gait_analyzer/xcom_measurement [geometry_msgs/PointStamped] 1 publisher
 * /gait_analyzer/mos_value_measurement2 [std_msgs/Float64] 1 publisher
 * /gait_analyzer/ankle_pose_measurement_r [geometry_msgs/PointStamped] 1 publisher
 * /gait_analyzer/gait_state [std_msgs/UInt8] 1 publisher
 * /gait_analyzer/mos_value_measurement0 [std_msgs/Float64] 1 publisher
 * /joint_states [sensor_msgs/JointState] 1 publisher
 * /rosout [rosgraph_msgs/Log] 4 publishers
 * /gait_analyzer/mos_value_estimate1 [std_msgs/Float64] 1 publisher
 * /gait_analyzer/mos_value_estimate2 [std_msgs/Float64] 1 publisher
 * /initialpose [geometry_msgs/PoseWithCovarianceStamped] 1 publisher
 * /gait_analyzer/mos_value_estimate0 [std_msgs/Float64] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /gait_analyzer/ground_clearance_left [std_msgs/Float64] 1 publisher
 * /gait_analyzer/mos [visualization_msgs/MarkerArray] 1 publisher
 * /gait_analyzer/footprint_r [geometry_msgs/PolygonStamped] 1 publisher
 * /gait_analyzer/ankle_pose_measurement_l [geometry_msgs/PointStamped] 1 publisher
 * /body_tracking_data [visualization_msgs/MarkerArray] 1 publisher
 * /clock [rosgraph_msgs/Clock] 1 publisher
 * /gait_analyzer/footprint_l [geometry_msgs/PolygonStamped] 1 publisher
 * /gait_analyzer/ground_clearance_right [std_msgs/Float64] 1 publisher
 * /gait_analyzer/pcom_vel_measurement [geometry_msgs/Vector3Stamped] 1 publisher
 * /gait_analyzer/pcom_pos_estimate [geometry_msgs/PointStamped] 1 publisher

Subscribed topics:
 * /move_base/global_costmap/costmap_updates [map_msgs/OccupancyGridUpdate] 1 subscriber
 * /trajectory [nav_msgs/Path] 1 subscriber
 * /tf [tf2_msgs/TFMessage] 2 subscribers
 * /scan [sensor_msgs/LaserScan] 1 subscriber
 * /gait_analyzer/xcom_estimate [geometry_msgs/PointStamped] 1 subscriber
 * /tf_static [tf2_msgs/TFMessage] 2 subscribers
 * /imu [sensor_msgs/Imu] 1 subscriber
 * /map_updates [map_msgs/OccupancyGridUpdate] 1 subscriber
 * /sport_sole_publisher/sport_sole [sport_sole/SportSole] 1 subscriber
 * /gait_analyzer/bos [geometry_msgs/PolygonStamped] 1 subscriber
 * /map [nav_msgs/OccupancyGrid] 1 subscriber
 * /move_base/global_costmap/footprint [geometry_msgs/PolygonStamped] 1 subscriber
 * /joint_states [sensor_msgs/JointState] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /gait_analyzer/footprint_r [geometry_msgs/PolygonStamped] 1 subscriber
 * /body_tracking_data [visualization_msgs/MarkerArray] 1 subscriber
 * /clock [rosgraph_msgs/Clock] 4 subscribers
 * /move_base/global_costmap/costmap [nav_msgs/OccupancyGrid] 1 subscriber
 * /gait_analyzer/footprint_l [geometry_msgs/PolygonStamped] 1 subscriber