#include <ros/ros.h>

#include <angles/angles.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>

#include <gait_training_robot/goal_generator.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <signal.h>

void GoalGeneratorParams::print()
{
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)         \
    ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

    ROS_PARAM_LIST
  #undef LIST_ENTRY
}

GoalGenerator::GoalGenerator(const ros::NodeHandle& n, const ros::NodeHandle& p): 
nh_(n),
private_nh_(p),
pose_index_(0), num_laps_(0),
action_client_("move_base", true) 
{
  // Collect ROS parameters from the param server or from the command line
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
    private_nh_.param(#param_variable, params_.param_variable, param_default_val);

    ROS_PARAM_LIST
  #undef LIST_ENTRY

  params_.yaml_file_path.insert(params_.yaml_file_path.size() - 5, params_.suffix);
  params_.print();

  readFromYaml(params_.yaml_file_path);
  
  if (params_.preview == false)
  {
    while(!action_client_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ros::Duration(params_.delay_start_secs).sleep();
    setNextGoal();
    sendGoal();
  }
  else
  {
    pub_goal_poses_ = private_nh_.advertise<geometry_msgs::PoseArray>("goal_poses", 1);
    ros::Duration(0.5).sleep();
    previewPoses();
    for (std::string line; 
         std::cout << "Please provide changes to make: {q|{x|y|h} <0-" << poses_.size() - 1 << "> <float>}: " , 
         std::cout.flush(), std::cin >> std::ws, 
         std::getline(std::cin, line);)
    {
      std::istringstream iss(line);

      char command;
      double * field = nullptr;
      if (!(iss >> command)) continue;
      if (command == 'q')
      {
        std::cout << "Quitting." << std::endl;
        break;
      }
      if (command != 'x' && command != 'y' && command != 'h')
      {
        std::cout << "Invalid input." << std::endl; continue;
      }

      int seq = 0;
      if (!(iss >> seq) || seq < 0 || seq >= poses_.size())
      {
        std::cout << "Invalid input." << std::endl; continue;
      }

      double change = 0.0;
      if (!(iss >> change))
      {
        std::cout << "Invalid input." << std::endl; continue;
      }


      if (command == 'x') poses_[seq].position.x += change;
      else if (command == 'y') poses_[seq].position.y += change;
      else
      {
        double yaw = atan2(poses_[seq].orientation.z, poses_[seq].orientation.w) * 2.0;
        yaw += change * M_PI / 180;
        poses_[seq].orientation.z = sin(yaw / 2.0);
        poses_[seq].orientation.w = cos(yaw / 2.0);
      }
      previewPoses();
    }
    writeToYaml(params_.yaml_file_path);
    ros::shutdown();

  }
  
}

void GoalGenerator::sigintHandler(int sig)
{
  if (params_.preview == false)
  {
    if (action_client_.isServerConnected())
    {
      action_client_.cancelAllGoals();
    }
    // Wait in case the robot is running at full speed before killing rosaria
    system("sleep 4; rosnode kill -a");
  }
  ros::shutdown();
}

bool GoalGenerator::readFromYaml(const std::string & file_name)
{
  std::vector<YAML::Node> nodes = YAML::LoadAllFromFile(file_name);
  if (nodes.size() > 0 && nodes[0]["pose"].IsDefined())
  {
    ROS_INFO("%lu waypoints are found in %s", nodes.size(), file_name.c_str());
    for (const auto & node : nodes) {
      try 
      {
        ROS_INFO("Reading pose with seq = %d", node["header"]["seq"].as<int>());
        if (global_frame_id_.empty())
        {
          global_frame_id_ = node["header"]["frame_id"].as<std::string>();
          ROS_INFO_STREAM("Global frame id set to '" << global_frame_id_ << "'");
        }
        poses_.push_back(geometry_msgs::Pose());
        const YAML::Node & node_position = node["pose"]["position"];
        const YAML::Node & node_orientation = node["pose"]["orientation"];
        if (!params_.interpolation_enabled)
        {
          poses_.back().position.x = node_position["x"].as<double>();
          poses_.back().position.y = node_position["y"].as<double>();
          poses_.back().position.z = 0.0;
          poses_.back().orientation.z = node_orientation["z"].as<double>();
          poses_.back().orientation.w = node_orientation["w"].as<double>();
        }
        else
        {
          // Interpolate waypoints
        }
        
      }
      catch (YAML::InvalidNode & exception)
      {
        ROS_WARN("Failed to read a node. Error message: %s", exception.what());
      }
    }
  }
  if (global_frame_id_.empty())
  {
    global_frame_id_ = "map";
  }
  
}

void GoalGenerator::writeToYaml(const std::string &file_name)
{
  YAML::Emitter out;
  int i = 0;
  for (const auto & pose : poses_)
  {
    // Begin waypoint
    out << YAML::BeginMap;

    // header
    out << YAML::Key << "header";
    out << YAML::BeginMap;
    out << YAML::Key << "seq";
    out << YAML::Value << i;
    out << YAML::Key << "frame_id";
    out << YAML::Value << global_frame_id_;
    out << YAML::EndMap;

    // Begin pose
    out << YAML::Key << "pose";
    out << YAML::BeginMap;
    
    // position
    out << YAML::Key << "position";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << pose.position.x;
    out << YAML::Key << "y";
    out << YAML::Value << pose.position.y;
    out << YAML::EndMap;

    // orientation
    out << YAML::Key << "orientation";
    out << YAML::BeginMap;
    out << YAML::Key << "z";
    out << YAML::Value << pose.orientation.z;
    out << YAML::Key << "w";
    out << YAML::Value << pose.orientation.w;
    out << YAML::EndMap;
    
    // End pose
    out << YAML::EndMap;

    // End waypoint
    out << YAML::EndMap;

    ++i;
  }

  std::ofstream fout(file_name);
  fout << out.c_str();
}

void GoalGenerator::previewPoses()
{
  if (poses_.empty()) {
    ROS_ERROR("GoalGenerator::poses_ is empty!");
    ros::shutdown();
  }

  geometry_msgs::PoseArrayPtr msg_pose_array(new geometry_msgs::PoseArray);
  msg_pose_array->header.frame_id = global_frame_id_;
  msg_pose_array->header.stamp = ros::Time::now();
  
  for (const auto & pose : poses_) {
    msg_pose_array->poses.push_back(pose);
  }

  pub_goal_poses_.publish(msg_pose_array);
  ROS_INFO("Prview of goals published.");
}

void GoalGenerator::setNextGoal() 
{
  if (poses_.empty()) {
    ROS_ERROR("GoalGenerator::poses_ is empty!");
    ros::shutdown();
  }

  ROS_INFO("%u, %zu", pose_index_, poses_.size());

  const geometry_msgs::Pose & pose = poses_[pose_index_];
  cur_goal_.target_pose.header.frame_id = global_frame_id_;
  cur_goal_.target_pose.header.stamp = ros::Time::now();
  cur_goal_.target_pose.pose.position.x = pose.position.x;
  cur_goal_.target_pose.pose.position.y = pose.position.y;
  cur_goal_.target_pose.pose.orientation.z = pose.orientation.z;
  cur_goal_.target_pose.pose.orientation.w = pose.orientation.w;

  if (pose_index_ >= poses_.size() - 1) {
    pose_index_ = 0;
    num_laps_++;
  }

  else pose_index_++;

}

void GoalGenerator::setNextGoalCircular()
{
  const double dth = -0.3, r = 0.8, x0 = 0.2, y0 = -0.8;
  static double th = M_PI/2;
  double x = x0 + r * cos(th);
  double y = y0 + r * sin(th);
  cur_goal_.target_pose.header.frame_id = global_frame_id_;
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

  if (num_laps_ >= params_.max_num_laps) {
    sigintHandler(SIGTERM);
  }
  else
  {
    action_client_.sendGoal(cur_goal_, 
      boost::bind(&GoalGenerator::doneCB, this, _1, _2) ,
      boost::bind(&GoalGenerator::activeCB, this),
      boost::bind(&GoalGenerator::feedbackCB, this, _1) 
    );
  }
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
  if (dist_to_goal < params_.dist_tolerance) {
    setNextGoal();
    sendGoal();
  }
}

// https://www.cs.technion.ac.il/users/yechiel/c++-faq/memfnptr-vs-fnptr.html
GoalGenerator * gg_ptr = nullptr;
void sigintHandlerWrapper(int sig)
{
  if (gg_ptr) gg_ptr->sigintHandler(sig);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_generator", ros::init_options::NoSigintHandler);

  GoalGenerator gg;
  gg_ptr = &gg;

  signal(SIGINT, sigintHandlerWrapper);
  

  ros::spin();
  return 0;
}