#include <vector>
#include <geometry_msgs/Pose.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#ifndef GOAL_GENERATOR_H
#define GOAL_GENERATOR_H

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalGenerator 
{
private:
  MoveBaseClient action_client_;
  std::vector<geometry_msgs::Pose> poses_;
  uint32_t pose_index_; 
  move_base_msgs::MoveBaseGoal cur_goal_;

public:
  GoalGenerator();
  ~GoalGenerator() {}
  bool readFromYaml(const std::string &file_name);
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