#ifndef FOOT_POSE_ESTIMATOR_H
#define FOOT_POSE_ESTIMATOR_H

#include <k4abttypes.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>
#include <sport_sole/SportSole.h>

class FootPoseEstimator
{
public:
  FootPoseEstimator(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));
  void skeletonsCB(const visualization_msgs::MarkerArray& msg);

private:
  // Subject selection
  int32_t sub_id_;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscribers
  ros::Subscriber sub_skeletons_;

  // tf
  tf2_ros::TransformBroadcaster tf_broadcaster_;

};



#endif // End of FOOT_POSE_ESTIMATOR_H