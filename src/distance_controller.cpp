#include "gait_training_robot/distance_controller.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

DistanceController::DistanceController():
  tf_base_to_pelvis_({}),
  estimated_state_({.stamp = ros::Time::now()}),
  desired_state_({.stamp = ros::Time::now(), .distance = 1.4})
{
  sub_cmd_vel_in_ = nh_.subscribe("/distance_controller/cmd_vel_in", 1, &DistanceController::cmdVelCB, this);
  pub_cmd_vel_out_ = nh_.advertise<geometry_msgs::Twist>("/distance_controller/cmd_vel_out", 1);

  desired_state_.distance = 1.5;
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
  try
  {
    tf_listener_.lookupTransform("/base_link", "/skeleton_pelvis_link", ros::Time(0), tf_base_to_pelvis_);
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
    ROS_ERROR("%s", ex.what());
    //ros::Duration(1.0).sleep();
  }

  const ros::Time stamp_now = ros::Time::now();
  ros::Duration time_since_last_state_update = stamp_now - estimated_state_.stamp;
  static ros::Time stamp_last_control_update = stamp_now;
  ros::Duration time_since_last_control_update = stamp_now - stamp_last_control_update;
  stamp_last_control_update = stamp_now;

  if (time_since_last_state_update < ros::Duration(0.8)) 
  {
    // Looks like the human state is up-to-date
    if (cmd_vel_in.linear.x > 0.2)
    {
      // Input linear velocity is large enough.
      // Presumably no obstacles ahead and far from the next goal point.
      // Use a PI controller to maintain the distance.
      HumanState error_state_ = desired_state_ - estimated_state_;
      double K_p = 1.0;
      double K_i = 0.5;
      double v_nominal = 0.4;
      static double u_i;
      u_i += K_i * error_state_.distance * clamp(time_since_last_control_update.toSec(), 0.0, 0.2);
      u_i = clamp(u_i, -0.4, 0.4);
      double u_p = K_p * error_state_.distance;
      cmd_vel_out.linear.x = clamp(v_nominal + u_p + u_i, 0.0, 0.7);
      cmd_vel_out.angular.z = cmd_vel_in.angular.z / cmd_vel_in.linear.x * cmd_vel_out.linear.x ;
      ROS_INFO_STREAM_NAMED("DistanceController", "u_p: " << u_p << ", u_i: " << u_i 
        << ", v_out" << cmd_vel_out.linear.x );
    }
    else
    {
      // Input linear velocity is too small.
      // Either obstacles or the goal is imminent.
      // Let the robot find its way without regard to the human state.
      ROS_INFO_STREAM_NAMED("DistanceController", "Input linear velocity is too small." );
      cmd_vel_out.linear.x = cmd_vel_in.linear.x ;
      cmd_vel_out.angular.z = cmd_vel_in.angular.z;
    }
  }
  else
  {
    //Human state is out of date
    ROS_INFO_STREAM_DELAYED_THROTTLE_NAMED(2, "DistanceController", 
      "Human state is out of date. The robot will stop moving until the user enters its field of view.");
    cmd_vel_out.linear.x = 0.0 ;
    cmd_vel_out.angular.z = 0.0;
  }
  pub_cmd_vel_out_.publish(cmd_vel_out);
  
  
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "distance_controller");
  DistanceController dc;
  ros::spin();
  return 0;
}