# gait_training_robot

# Installation
## Prerequisites
- Ubuntu 16.04
- ROS Kinetic
- Aria
- rosaria


## Checklist for the robot
1. Install libaria from source
   ```bash
   cd ~/projects
   git clone https://github.com/zchenpds/Aria.git
   make
   sudo make install
   echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Aria/lib' >> ~/.bashrc
   ldconfig
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

1. SLAM
  - **One Machine**. run `roslaunch gait_training_robot test1_slam_headless.launch`.
  - **Two Machines**. 
    1. Find IP addresses of both machines by running `hostname -I`. add `export ROS_IP=$(hostname -I)` to the  `~/.bashrc` file of each of the machines.
    2. On the robot computer (master), run `roslaunch gait_training_robot test1_slam_headless.launch`.
    3. On the desktop computer, add `export ROS_MASTER_URI=http://MASTER_IP:11311/` to `~/.bashrc` and run `roslaunch gait_training_robot test1_slam_rviz.launch`.