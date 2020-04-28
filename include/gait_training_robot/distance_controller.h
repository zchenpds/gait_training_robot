#ifndef DISTANCE_CONTROLLER_H
#define DISTANCE_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <k4abt.h>

// LIST_ENTRY is:
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// The format of these list entries is :
//
// LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)
//
// param_variable: the variable name which will be created in the k4a_ros_device class to hold the contents of the
//    parameter
// param_help_string: a string containing help information for the parameter
// param_type: the type of the parameter
// param_default_val: the default value of the parameter
//
// Example:
// LIST_ENTRY(sensor_sn, "The serial number of the sensor this node should connect with", std::string, std::string(""))
#define ROS_PARAM_LIST                                                                                                      \
  LIST_ENTRY(dist_desired, "The desired distance from the subject to the robot", double, 2.5)    \
  LIST_ENTRY(use_marker, "If true, use marker array to determine human state", bool, true)    \
  LIST_ENTRY(timeout_threshold, "If the time that has elapse since last state update is longer than this value, we consider the state update has timed out", double, 0.5)    \
  LIST_ENTRY(v_in_threshold, "If the linear command velocity is smaller than this value, then the distance controller is disengaged.", double, 0.5)    \
  LIST_ENTRY(u_i_max, "The max value for the integral control", double, 0.5)    \
  LIST_ENTRY(K_p, "The coefficient for the proportional control", double, 1.0)    \
  LIST_ENTRY(K_i, "The coefficient for the integral control", double, 0.5)    \
  LIST_ENTRY(v_nominal, "The nominal linear velocity", double, 0.5)    \
  LIST_ENTRY(v_max, "The max linear velocity", double, 1.2)    \
  LIST_ENTRY(bypass_move_base, "If true, go in straight line regardless of input cmd_vel",bool, false)    \




struct DistanceControllerParams 
{
  // Print the value of all parameters
  void print();

  // Parameters
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
    ROS_PARAM_LIST
  #undef LIST_ENTRY
};

struct HumanState {
    ros::Time stamp;
    double distance;
    double bearing;
    //double speed;
    HumanState operator-(HumanState & rhs)
    {
      HumanState ret;
      ret.stamp = (stamp > rhs.stamp) ? stamp : rhs.stamp;
      ret.distance = distance - rhs.distance;
      ret.bearing = angles::normalize_angle(bearing - rhs.bearing);
      return ret;
    }
};
std::ostream & operator<<(std::ostream & os, const HumanState & state)
{
  return os << state.distance << " m, " << state.bearing << " rad";
}

class DistanceController {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  DistanceControllerParams params_;

  ros::Subscriber sub_cmd_vel_in_;
  ros::Publisher pub_cmd_vel_out_;

  ros::Subscriber sub_skeletons_;
  
  tf::TransformListener tf_listener_;
  HumanState estimated_state_, desired_state_;
  tf::StampedTransform tf_base_to_pelvis_;

  // Subject selection
    int32_t sub_id_;

public:
  DistanceController(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));
  ~DistanceController();
  void cmdVelCB(const geometry_msgs::Twist & cmd_vel_in);
  void skeletonsCB(const visualization_msgs::MarkerArray& msg);
};

template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi)
{
    return assert( !(hi < lo) ),
        (v < lo) ? lo : ((hi < v) ? hi : v);
}

#endif