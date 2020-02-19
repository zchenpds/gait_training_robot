#include "gait_training_robot/distance_controller.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

using MarkerArray = visualization_msgs::MarkerArray;

void DistanceControllerParams::print()
{
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)         \
    ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

    ROS_PARAM_LIST
  #undef LIST_ENTRY
}

DistanceController::DistanceController(const ros::NodeHandle& n, const ros::NodeHandle& p):
  nh_(n),
  private_nh_(p),
  tf_base_to_pelvis_({}),
  estimated_state_({.stamp = ros::Time::now()}),
  desired_state_({.stamp = ros::Time::now(), .distance = 1.4}),
  sub_id_(-1)
{
  // Collect ROS parameters from the param server or from the command line
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
    private_nh_.param(#param_variable, params_.param_variable, param_default_val);

    ROS_PARAM_LIST
  #undef LIST_ENTRY
  params_.print();

  sub_cmd_vel_in_ = nh_.subscribe("/distance_controller/cmd_vel_in", 1, &DistanceController::cmdVelCB, this);
  pub_cmd_vel_out_ = nh_.advertise<geometry_msgs::Twist>("/distance_controller/cmd_vel_out", 1);

  sub_skeletons_ = nh_.subscribe("/body_tracking_data", 5, &DistanceController::skeletonsCB, this );

  desired_state_.distance = params_.dist_desired;
  desired_state_.bearing = M_PI;
}

DistanceController::~DistanceController()
{
  pub_cmd_vel_out_.shutdown();
  sub_cmd_vel_in_.shutdown();
}

void DistanceController::cmdVelCB(const geometry_msgs::Twist & cmd_vel_in)
{
  static geometry_msgs::Twist cmd_vel_out = {};

  if (!params_.use_marker)  {
    try
    {
      tf_listener_.lookupTransform("/base_link", "/joint_0", ros::Time(0), tf_base_to_pelvis_);
      tf::Vector3 disp_vec =  tf_base_to_pelvis_.getOrigin();
      estimated_state_.stamp = ros::Time::now();
      ROS_INFO_STREAM("DEBUG: At t=" << estimated_state_.stamp << ", tf_stamp=" << tf_base_to_pelvis_.stamp_);
      estimated_state_.distance = hypot(disp_vec.getX(), disp_vec.getY());
      estimated_state_.bearing = atan2(disp_vec.getY(), disp_vec.getX());
      ROS_INFO_STREAM("Estimated state: " << estimated_state_);
      //ROS_INFO_STREAM("disp_vector: [" << disp_vec.getX() << ", " << disp_vec.getY() << ", " << disp_vec.getZ() << "]" );
    }
    catch (tf::TransformException ex)
    {
      const std::string str_error = ex.what();
      auto pos = str_error.find_first_of("source_frame does not exist");
      if (pos != std::string::npos)
        ROS_WARN_DELAYED_THROTTLE(1.0, "Waiting for subject to come in the FOV of the Azure Kinect sensor...");
      else
        ROS_WARN_DELAYED_THROTTLE(1.0, "%s", ex.what());
      //ros::Duration(1.0).sleep();
    }
  }
  else{
    // Callback function DistanceController::skeletonCB takes care of updating the state.
  }

  const ros::Time stamp_now = ros::Time::now();
  ros::Duration time_since_last_state_update = stamp_now - estimated_state_.stamp;
  static ros::Time stamp_last_control_update = stamp_now;
  ros::Duration time_since_last_control_update = stamp_now - stamp_last_control_update;
  stamp_last_control_update = stamp_now;

  if (time_since_last_state_update < ros::Duration(params_.timeout_threshold)) 
  {
    // Looks like the human state is up-to-date
    if (cmd_vel_in.linear.x > params_.v_in_threshold)
    {
      // Input linear velocity is large enough.
      // Presumably no obstacles ahead and far from the next goal point.
      // Use a PI controller to maintain the distance.
      HumanState error_state_ = desired_state_ - estimated_state_;
      double K_p = params_.K_p;
      double K_i = params_.K_i;
      double v_nominal = params_.v_nominal;
      static double u_i;
      u_i += K_i * error_state_.distance * clamp(time_since_last_control_update.toSec(), 0.0, 0.2);
      u_i = clamp(u_i, -params_.u_i_max, params_.u_i_max);
      double u_p = K_p * error_state_.distance;
      cmd_vel_out.linear.x = clamp(v_nominal + u_p + u_i, 0.0, params_.v_max);
      cmd_vel_out.angular.z = cmd_vel_in.angular.z / cmd_vel_in.linear.x * cmd_vel_out.linear.x ;
      ROS_INFO_STREAM("u_p: " << u_p << ", u_i: " << u_i 
        << ", v_out" << cmd_vel_out.linear.x );
    }
    else
    {
      // Input linear velocity is too small.
      // Either obstacles or the goal is imminent.
      // Let the robot find its way without regard to the human state.
      ROS_WARN_STREAM("Input linear velocity is too small." );
      cmd_vel_out.linear.x = cmd_vel_in.linear.x ;
      cmd_vel_out.angular.z = cmd_vel_in.angular.z;
    }
  }
  else
  {
    //Human state is out of date
    ROS_WARN_STREAM("Human state is out of date. time_since_last_state_update = " 
      << time_since_last_state_update.toSec() << " The robot will stop moving until the user enters its field of view.");
    cmd_vel_out.linear.x = 0.0 ;
    cmd_vel_out.angular.z = 0.0;
  }
  pub_cmd_vel_out_.publish(cmd_vel_out);
  
  
}

void DistanceController::skeletonsCB(const MarkerArray& msg)
{
  if (!params_.use_marker)
    return;

  double dist_min_pelvis = 100.0;
  //double idx = -1; // body id with the min dist of pelvis from camera center
  auto it_pelvis_closest = msg.markers.end();// iterator of the pelvis marker of the closest body

  // Find the closest body, K4ABT_JOINT_PELVIS = 0
  if (sub_id_ < 0) {
    for (auto it = msg.markers.begin(); it < msg.markers.end(); it += K4ABT_JOINT_COUNT) {
      // The coordinates are in expressed in /depth_camera_link
      double dist_pelvis = hypot(it->pose.position.x, it->pose.position.z);
      if (dist_pelvis < dist_min_pelvis) {
        dist_pelvis = dist_min_pelvis;
        it_pelvis_closest = it;
        sub_id_ = it_pelvis_closest->id / 100;
      }
    }
  }
  else {
    for (auto it = msg.markers.begin(); it < msg.markers.end(); it += K4ABT_JOINT_COUNT) {
      if (it->id / 100 == sub_id_) {
        it_pelvis_closest = it;
        break;
      }
    }
  }

  // Process the closest body and update the human state
  if (it_pelvis_closest != msg.markers.end())
  {
    estimated_state_.stamp = it_pelvis_closest->header.stamp;
    estimated_state_.distance = hypot(it_pelvis_closest->pose.position.x, it_pelvis_closest->pose.position.z);
    estimated_state_.bearing = atan2(it_pelvis_closest->pose.position.x, -it_pelvis_closest->pose.position.z);

  }

}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "distance_controller");
  DistanceController dc;
  ros::spin();
  return 0;
}