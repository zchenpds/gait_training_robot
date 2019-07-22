#include <ros/ros.h>

#include <angles/angles.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <gait_training_robot/goal_generator.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

GoalGenerator::GoalGenerator(): pose_index_(0), action_client_("move_base", true) 
{
  
  while(!action_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
}


bool GoalGenerator::readFromYaml(const std::string & file_name)
{
  std::vector<YAML::Node> nodes = YAML::LoadAllFromFile(file_name);
  ROS_INFO("%lu waypoints are found in %s", nodes.size(), file_name.c_str());
  for (const auto & node : nodes) {
    try 
    {
      ROS_INFO("Reading pose with seq = %d", node["header"]["seq"].as<int>());
      poses_.push_back(geometry_msgs::Pose());
      const YAML::Node & node_position = node["pose"]["position"];
      poses_.back().position.x = node_position["x"].as<double>();
      poses_.back().position.y = node_position["y"].as<double>();
      const YAML::Node & node_orientation = node["pose"]["orientation"];
      poses_.back().orientation.z = node_orientation["z"].as<double>();
      poses_.back().orientation.w = node_orientation["w"].as<double>();
    }
    catch (YAML::InvalidNode & exception)
    {
      ROS_WARN("Failed to read a node. Error message: %s", exception.what());
    }
  }
  
}

void GoalGenerator::setNextGoal() 
{
  if (poses_.empty()) {
    ROS_ERROR("GoalGenerator::poses_ is empty!");
    ros::shutdown();
  }

  ROS_INFO("%u, %zu", pose_index_, poses_.size());

  const geometry_msgs::Pose & pose = poses_[pose_index_];
  cur_goal_.target_pose.header.frame_id = "map";
  cur_goal_.target_pose.header.stamp = ros::Time::now();
  cur_goal_.target_pose.pose.position.x = pose.position.x;
  cur_goal_.target_pose.pose.position.y = pose.position.y;
  cur_goal_.target_pose.pose.orientation.z = pose.orientation.z;
  cur_goal_.target_pose.pose.orientation.w = pose.orientation.w;

  if (pose_index_ >= poses_.size() - 1) pose_index_ = 0;
  else pose_index_++;

}

void GoalGenerator::setNextGoalCircular()
{
  const double dth = -0.3, r = 0.8, x0 = 0.2, y0 = -0.8;
  static double th = M_PI/2;
  double x = x0 + r * cos(th);
  double y = y0 + r * sin(th);
  cur_goal_.target_pose.header.frame_id = "map";
  cur_goal_.target_pose.header.stamp = ros::Time::now();
  cur_goal_.target_pose.pose.position.x = x;
  cur_goal_.target_pose.pose.position.y = y;
  tf::Quaternion q;
  q.setRPY(0, 0, th - M_PI/2);
  tf::quaternionTFToMsg(q, cur_goal_.target_pose.pose.orientation);
  th = angles::normalize_angle(th + dth);

}

void GoalGenerator::sendGoal()
{
  printPoseMessage("Sending goal [%.2f m, %.2f m, %.2f rad]", cur_goal_.target_pose.pose);
  action_client_.sendGoal(cur_goal_, 
    boost::bind(&GoalGenerator::doneCB, this, _1, _2) ,
    boost::bind(&GoalGenerator::activeCB, this),
    boost::bind(&GoalGenerator::feedbackCB, this, _1) 
  );
}

void GoalGenerator::doneCB(const actionlib::SimpleClientGoalState & state, const move_base_msgs::MoveBaseResultConstPtr & result)
{
  if(action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    printPoseMessage("Hooray, the base moved to pose [%.2f m, %.2f m, %.2f rad]!", cur_goal_.target_pose.pose);
    setNextGoal();
    sendGoal();
  }  
  else{
    printPoseMessage("The base failed to move to pose [%.2f m, %.2f m, %.2f rad] for some reason", cur_goal_.target_pose.pose);
    ros::shutdown();
  }
}
void GoalGenerator::activeCB()
{
  printPoseMessage("Goal [%.2f m, %.2f m, %.2f rad] just went active", cur_goal_.target_pose.pose);
}

void GoalGenerator::feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr & feedback)
{
  printPoseMessage("Received pose feedback [%.2f m, %.2f m, %.2f rad]", feedback->base_position.pose);
  const auto & goal_position = cur_goal_.target_pose.pose.position;
  const auto & actual_position = feedback->base_position.pose.position;
  double dist_to_goal = hypot(goal_position.x - actual_position.x, goal_position.y - actual_position.y);
  if (dist_to_goal < 0.5) {
    setNextGoal();
    sendGoal();
  }
}



int main(int argc, char** argv){
  ros::init(argc, argv, "goal_generator");

  GoalGenerator gg;
  gg.readFromYaml(ros::package::getPath("gait_training_robot") + "/data/waypoints.yaml"); // TO-DO: read as a ROS parameter
  gg.setNextGoal();
  gg.sendGoal();

  ros::spin();
  return 0;
}