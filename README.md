# gait_training_robot

# Installation
## Prerequisites
- Ubuntu 16.04
- ROS Kinetic


## Checklist for the robot
1. Install libaria
   ```bash
   sudo dpkg -i libaria_2.9.1+ubuntu16_i368.deb
   ```
   Or you can download the source and build it yourself
   ```bash
   cd ~/projects
   git clone https://github.com/cinvesrob/Aria.git
   ```
2. Download and `catkin_make` rosaria
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/amor-ros-pkg/rosaria.git
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

1. `roslaunch gait_training_robot test1_slam.launch`