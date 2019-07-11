//Purpose: bring together data from  1) accel, quaternion, pressures of left & right sport soles  2) kinect IMU  3) kinect skeleton info

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "sport_sole/SportSole.h"
#include "gait_training_robot/HumanSkeletonAzure.h"

void listenerCallback (const sport_sole::SportSole& msg)
{
    //TODO: Kalman filter??
    //process the subcribed data here
}
enum left_right_t {
    LR_left=0,
    LR_right=1
};

void sportsole_callback(const sport_sole::SportSole::ConstPtr &msg, const left_right_t left_right) {
    ; // Do stuff
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "kalman_filter_subscriber");
    ros::NodeHandle node_handle;
 
    //create subscriber to collect data from sport soles, IMU, skeleton
    ros::Subscriber sportsole_left_sub = node_handle.subscribe<sport_sole::SportSole>("sportsole_left", 1000,  boost::bind(sportsole_callback, _1, LR_left));
    ros::Subscriber sportsole_right_sub = node_handle.subscribe<sport_sole::SportSole>("sportsole_right", 1000, boost::bind(sportsole_callback, _1, LR_right));
/*    ros::Subscriber kinect_IMU_sub = node_handle.subscribe<sensor_msgs::Imu>("kinect_azure_imu", 1000, listenerCallback);  
    ros::Subscriber kinect_skeleton_sub = node_handle.subscribe<gait_training_robot::HumanSkeletonAzure>("skeleton", 1000, listenerCallback);
 */
    
    
    
    
    


    //create publishers here

}