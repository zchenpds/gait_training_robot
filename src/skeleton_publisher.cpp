#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <sport_sole/sport_sole_common.h>
#include <gait_training_robot/skeleton_common.h>

#include <array>

using PoseType = geometry_msgs::PoseWithCovarianceStamped;
using namespace visualization_msgs;

tf2_ros::Buffer tf_buffer_;
std::array<tf2::Transform, K4ABT_JOINT_COUNT> joint_tfs_;

std::string global_frame_;

int sub_id_ = -1;

ros::Publisher pub_skeleton_raw_;
ros::Publisher pub_skeleton_fused_;
MarkerArray segment_marker_array_;

geometry_msgs::PoseStamped fused_foot_poses_[LEFT_RIGHT];

inline bool isAnkle(int joint_id) {return joint_id == K4ABT_JOINT_ANKLE_LEFT || joint_id == K4ABT_JOINT_ANKLE_RIGHT; }

void skeletonsCB(const visualization_msgs::MarkerArray& msg)
{
  if (global_frame_.empty())
  {
    ROS_WARN_DELAYED_THROTTLE(5, "Waiting for fused foot pose to be updated.");
    return;
  }

  double dist_min_pelvis = 100.0;
  auto it_pelvis_closest = msg.markers.end();// iterator of the pelvis marker of the closest body

  // Find the closest body, K4ABT_JOINT_PELVIS = 0
  if (sub_id_ < 0) {
    for (auto it = msg.markers.begin(); it < msg.markers.end(); it += K4ABT_JOINT_COUNT) {
      // The coordinates are in expressed in /depth_camera_link
      double dist_pelvis = hypot(it->pose.position.x, it->pose.position.z);
      if (dist_pelvis < dist_min_pelvis) {
        dist_min_pelvis = dist_pelvis;
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

  // Process the closest body
  if (it_pelvis_closest != msg.markers.end())
  {
    // Get the current timestamp of the skeleton message
    const auto & stamp_skeleton_curr = it_pelvis_closest->header.stamp;

    // Get transform
    tf2::Transform tf_depth_to_global;
    try
    {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg = tf_buffer_.lookupTransform(global_frame_, "depth_camera_link", stamp_skeleton_curr, ros::Duration(0.2));
      tf2::fromMsg(tf_msg.transform, tf_depth_to_global);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
  
    for (size_t i = 0; i < K4ABT_JOINT_COUNT; ++i)
    {
      tf2::fromMsg((it_pelvis_closest + i)->pose, joint_tfs_[i]);
      joint_tfs_[i] = tf_depth_to_global * joint_tfs_[i];
    }

    // skeleton_raw
    for (size_t i = 0; i < vec_segments.size(); ++i)
    {
      Marker* segment_msg = &segment_marker_array_.markers[i];
      k4abt_joint_id_t proximal_joint_id = vec_segments[i].joint_pair_.first;
      k4abt_joint_id_t distal_joint_id = vec_segments[i].joint_pair_.second;
      auto proximal_joint_msg = it_pelvis_closest + proximal_joint_id;
      auto distal_joint_msg = it_pelvis_closest + distal_joint_id;

      // if (isAnkle(proximal_joint_id)) continue;

      segment_msg->header.frame_id = global_frame_;
      segment_msg->header.stamp = distal_joint_msg->header.stamp;
      segment_msg->id = distal_joint_msg->id + 1000;
      segment_msg->lifetime = ros::Duration(1000);

      segment_msg->color.a = distal_joint_msg->color.a;
      segment_msg->color.r = distal_joint_msg->color.r;
      segment_msg->color.g = distal_joint_msg->color.g;
      segment_msg->color.b = distal_joint_msg->color.b;

      if (isAnkle(proximal_joint_id))
      {
        segment_msg->type = Marker::MESH_RESOURCE;
        segment_msg->mesh_resource = "package://gait_training_robot/meshes/shoe.stl";
        
        segment_msg->scale.x = segment_msg->scale.y = segment_msg->scale.z = 0.00013;

        tf2::toMsg(joint_tfs_[proximal_joint_id], segment_msg->pose);
        correctAnkleMarkerOrientation(segment_msg, proximal_joint_id);
        correctAnkleMarkerDisplacement(segment_msg);
        correctShoeMarkerOrientation(segment_msg, proximal_joint_id);
      }
      else
      {        
        tf2::toMsg(joint_tfs_[proximal_joint_id].getOrigin(), segment_msg->points[0]);
        tf2::toMsg(joint_tfs_[distal_joint_id].getOrigin(), segment_msg->points[1]);
      }
    }
    pub_skeleton_raw_.publish(segment_marker_array_);

    // Skeleton_fused
    for (left_right_t lr: {LEFT, RIGHT})
    {
      auto ankle_joint_id = lr == LEFT ? K4ABT_JOINT_ANKLE_LEFT: K4ABT_JOINT_ANKLE_RIGHT;
      auto knee_joint_id  = lr == LEFT ? K4ABT_JOINT_KNEE_LEFT:  K4ABT_JOINT_KNEE_RIGHT;

      // Get fused poses
      tf2::Vector3 position; tf2::fromMsg(fused_foot_poses_[lr].pose.position, position);
      tf2::Quaternion quat; tf2::fromMsg(fused_foot_poses_[lr].pose.orientation, quat);
      position += tf2::quatRotate(quat, {-0.1, 0.0, 0.1});

      // Get delta (correction)
      tf2::Vector3 delta_position_ankle = position - joint_tfs_[ankle_joint_id].getOrigin();

      // Correct ankle and foot joints
      joint_tfs_[ankle_joint_id].setOrigin(position);
      joint_tfs_[ankle_joint_id].setRotation(quat);
      joint_tfs_[knee_joint_id].setOrigin(joint_tfs_[knee_joint_id].getOrigin() + delta_position_ankle * 0.5);
    }

    for (size_t i = 0; i < vec_segments.size(); ++i)
    {
      Marker* segment_msg = &segment_marker_array_.markers[i];
      k4abt_joint_id_t proximal_joint_id = vec_segments[i].joint_pair_.first;
      k4abt_joint_id_t distal_joint_id = vec_segments[i].joint_pair_.second;

      if (isAnkle(proximal_joint_id))
      {
        tf2::toMsg(joint_tfs_[proximal_joint_id], segment_msg->pose);
        correctAnkleMarkerDisplacement(segment_msg);
        correctShoeMarkerOrientation(segment_msg, proximal_joint_id);
      }
      else
      {        
        tf2::toMsg(joint_tfs_[proximal_joint_id].getOrigin(), segment_msg->points[0]);
        tf2::toMsg(joint_tfs_[distal_joint_id].getOrigin(), segment_msg->points[1]);
      }
    }
    pub_skeleton_fused_.publish(segment_marker_array_);

  }
}

void FusedPoseLCB(const PoseType& pose_src)
{
  fused_foot_poses_[0].header.stamp = pose_src.header.stamp;
  fused_foot_poses_[0].pose = pose_src.pose.pose;
  if (global_frame_.empty()) global_frame_ = pose_src.header.frame_id;
}

void FusedPoseRCB(const PoseType& pose_src)
{
  fused_foot_poses_[1].header.stamp = pose_src.header.stamp;
  fused_foot_poses_[1].pose = pose_src.pose.pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "skeleton_publisher");
  
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf_listener_(tf_buffer_);
  ros::Subscriber sub_skeletons = nh.subscribe("/body_tracking_data", 10, skeletonsCB);
  ros::Subscriber sub_foot_poses[LEFT_RIGHT] = {
      nh.subscribe("/foot_pose_estimator/fused_pose_l", 10, FusedPoseLCB), 
      nh.subscribe("/foot_pose_estimator/fused_pose_r", 10, FusedPoseRCB)};

  pub_skeleton_raw_   = nh.advertise<MarkerArray>("/skeleton_raw",   1);
  pub_skeleton_fused_ = nh.advertise<MarkerArray>("/skeleton_fused", 1);
  segment_marker_array_.markers.resize(body_segment_count);
  for (int i = 0; i < body_segment_count; ++i)
  {
    auto & marker = segment_marker_array_.markers[i];
    marker.type = Marker::ARROW;
    marker.points.resize(2);
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.001;
    marker.pose.orientation.w = 1.0;
  }

  ros::spin();
  return 0;
}