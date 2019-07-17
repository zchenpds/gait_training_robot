#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <angles/angles.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <gait_training_robot/goal_generator.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool GoalGenerator::readFromYaml(const std::string & fileName)
{
  std::vector<YAML::Node> nodes = YAML::LoadAllFromFile(fileName);
  ROS_INFO("%lu waypoints are found in %s", nodes.size(), fileName.c_str());
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

void GoalGenerator::setNextGoal(move_base_msgs::MoveBaseGoal & goal) 
{
  if (poses_.empty()) {
    ROS_ERROR("GoalGenerator::poses_ is empty!");
  }
  ROS_INFO("%u, %zu", num_goals_requested, poses_.size());
  const geometry_msgs::Pose & pose = poses_[num_goals_requested % poses_.size()];
  goal.target_pose.pose.position.x = pose.position.x;
  goal.target_pose.pose.position.y = pose.position.y;
  goal.target_pose.pose.orientation.z = pose.orientation.z;
  goal.target_pose.pose.orientation.w = pose.orientation.w;
  num_goals_requested++;

}

void GoalGenerator::setNextGoalCircular(move_base_msgs::MoveBaseGoal & goal)
{
  const double dth = -0.3, r = 0.8, x0 = 0.2, y0 = -0.8;
  static double th = M_PI/2;
  double x = x0 + r * cos(th);
  double y = y0 + r * sin(th);
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  tf::Quaternion q;
  q.setRPY(0, 0, th - M_PI/2);
  tf::quaternionTFToMsg(q, goal.target_pose.pose.orientation);
  th = angles::normalize_angle(th + dth);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "goal_generator");

  GoalGenerator gg;
  gg.readFromYaml(ros::package::getPath("gait_training_robot") + "/data/waypoints.yaml"); // TO-DO: read as a ROS parameter
  
  //std::terminate();

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  

  while (1) {
    
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    gg.setNextGoal(goal);
    //gg.setNextGoalCircular(goal);
       
    ROS_INFO("Sending goal [%.2f m, %.2f m, %.2f rad]", 
        goal.target_pose.pose.position.x, 
        goal.target_pose.pose.position.y, 
        tf::getYaw(goal.target_pose.pose.orientation));
    ac.sendGoal(goal);
    
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved to pose [%.2f m, %.2f m, %.2f rad]!", 
        goal.target_pose.pose.position.x, 
        goal.target_pose.pose.position.y, 
        tf::getYaw(goal.target_pose.pose.orientation));
    else {
      ROS_INFO("The base failed to move to pose [%.2f m, %.2f m, %.2f rad] for some reason", 
        goal.target_pose.pose.position.x, 
        goal.target_pose.pose.position.y, 
        tf::getYaw(goal.target_pose.pose.orientation));
      break;
    }

  }


  return 0;
}