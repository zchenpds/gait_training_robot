#include <vector>
#include <geometry_msgs/Pose.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#ifndef GOAL_GENERATOR_H
#define GOAL_GENERATOR_H

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
  LIST_ENTRY(yaml_file_path, "The path to the yaml file where the list of goals is stored", std::string, ros::package::getPath("gait_training_robot") + "/data/waypoints.yaml")    \
  LIST_ENTRY(interpolation_enabled, "Smooth the paths between waypoints by adding more waypoints", bool, false)    \
  LIST_ENTRY(dist_tolerance, "The critical distance between the current and the goal robot positions, below which the robot will start pursuing the next goal in the sequence.", float, 1.0f)    \
  LIST_ENTRY(preview, "If true, the goal will not be executed.", bool, false)    \


struct GoalGeneratorParams 
{
  // Print the value of all parameters
  void print();

  // Parameters
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
    ROS_PARAM_LIST
  #undef LIST_ENTRY
};

class GoalGenerator 
{
private:
  // ROS Node variables
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  GoalGeneratorParams params_;

  MoveBaseClient action_client_;
  std::vector<geometry_msgs::Pose> poses_;
  uint32_t pose_index_; 
  move_base_msgs::MoveBaseGoal cur_goal_;

  ros::Publisher pub_goal_poses_;

protected:
  bool readFromYaml(const std::string &file_name);
  void previewPoses();

public:
  GoalGenerator(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));
  ~GoalGenerator() {}
  void setNextGoal();
  void setNextGoalCircular();
  void sendGoal();
  

  void doneCB(const actionlib::SimpleClientGoalState & state, const move_base_msgs::MoveBaseResultConstPtr & result);
  void activeCB();
  void feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr & feedback);
};

inline void printPoseMessage(const char * text, const geometry_msgs::Pose & pose) 
{
  ROS_INFO(text, pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
}


#endif