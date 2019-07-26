# gait_training_robot

## Prerequisites
- Ubuntu 16.04
- ROS Kinetic
- Aria
- rosaria
- [BodyTrackerAzure](https://github.com/ral-stevens/BodyTrackerAzure)

## Checklist for Kinect v1
1. Install pointcloud_to_laserscan
  ```bash
  sudo apt-get install ros-kinetic-pointcloud-to-laserscan
  ```
2. Install openni_launch
  ```bash
  sudo apt-get install ros-kinetic-openni-launch
  ```
3. Install gmapping (optional)
  ```bash
  sudo apt-get install ros-kinetic-gmapping
  ```
4. Install hector_trajectory_server
  ```bash
  sudo apt-get install ros-kinetic-hector-trajectory-server
  ```
5. Install move_base
  ```bash
  sudo apt-get install ros-kinetic-move-base
  ```
6. Install rtabmap_ros
  ```bash
  sudo apt-get install ros-kinetic-rtabmap-ros
  ```
7. Build rosserial_windows from a forked repo.
  ```bash
  git clone https://github.com/ral-stevens/rosserial.git
  cd ~/catkin_ws
  catkin_make
  ```

## Checklist for the robot
1. Install libaria from source
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

# Examples

1. SLAM using gmapping
  - **One Machine**. run `roslaunch gait_training_robot test1_slam_headless.launch`.
  - **Two Machines**. 
    1. Find IP addresses of both machines by running `hostname -I`. add `export ROS_IP=$(hostname -I)` to the  `~/.bashrc` file of each of the machines.
    2. On the robot computer (master), run `roslaunch gait_training_robot test1_slam_headless.launch`.
    3. On the desktop computer, add `export ROS_MASTER_URI=http://MASTER_IP:11311/` to `~/.bashrc` and run `roslaunch gait_training_robot test1_slam_rviz.launch`.
    4. Note that if kinect2_bridge package is started via ssh, it is necessary to tell GLFW to be launched on a server screen by running `export DISPLAY=:0` after `ssh`ing to the server.
    ![alt text](images/screenshot1.png)

2. Planning
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
5. Human distance controller
  - **Three Machines**. Requires a map built by rtabmap. Requires waypoints to be recorded.
    1. On the robot computer (master), run `roslaunch gait_training_robot test1_rtabmap_headless.launch localization:=true enable_distance_controller:=true`.
    2. On the desktop computer, start rviz by running `roslaunch gait_training_robot test1_rviz_rtabmap.launch`.
    3. On the Windows computer, start `BodyTrackerAzure.exe`.
    4. To play back waypoints that have been recorded on the robot computer, run `rosrun gait_training_robot goal_generator`.
    