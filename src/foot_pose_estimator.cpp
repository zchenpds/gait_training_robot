#include "gait_training_robot/foot_pose_estimator.h"

FootPoseEstimator::FootPoseEstimator(const ros::NodeHandle& n, const ros::NodeHandle& p):
  sub_id_(-1),
  nh_(n),
  private_nh_(p)
{
  sub_skeletons_ = nh_.subscribe("/body_tracking_data", 5, &FootPoseEstimator::skeletonsCB, this );
  std::cout << "Waiting for /body_tracking_data" << std::flush;

}


void FootPoseEstimator::skeletonsCB(const visualization_msgs::MarkerArray& msg)
{
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

  static int tf_cnt;

  // Process the closest body
  if (it_pelvis_closest != msg.markers.end())
  {
    ++tf_cnt;
    // Get the current timestamp of the skeleton message
    const auto & stamp_skeleton_curr = it_pelvis_closest->header.stamp;

    // Broadcast some tfs.
    for (auto ankle_id: {K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_ANKLE_RIGHT})
    {
      const auto & it_ankle = it_pelvis_closest + ankle_id;

      geometry_msgs::TransformStamped tf_joint;
      tf_joint.header.stamp = stamp_skeleton_curr;
      tf_joint.header.frame_id = "depth_camera_link";
      
      // Orientation
      tf2::Quaternion quat;
      tf2::fromMsg(it_ankle->pose.orientation, quat);
      switch (ankle_id)
      {
        case K4ABT_JOINT_ANKLE_LEFT: 
          tf_joint.child_frame_id = "sport_sole_left";
          quat *= tf2::Quaternion({1.0, 1.0, 1.0}, M_PI / 3 * 2) ;
          break;
        case K4ABT_JOINT_ANKLE_RIGHT: 
          tf_joint.child_frame_id = "sport_sole_right";
          quat *= tf2::Quaternion({-1.0, 1.0, 1.0}, -M_PI / 3 * 2) ;
          break;
        default:
          tf_joint.child_frame_id = std::string("joint_") + std::to_string(ankle_id);
      }
      tf_joint.transform.rotation = tf2::toMsg(quat);

      // Position
      tf2::Vector3 position_ankle;
      tf2::fromMsg(it_ankle->pose.position, position_ankle);
      auto disp_imu = tf2::quatRotate(quat, {0.0, 0.0, -0.1});
      tf_joint.transform.translation = tf2::toMsg(position_ankle + disp_imu);

      tf_broadcaster_.sendTransform(tf_joint);
    }
  }

  std::cout << "\rBody id: " << sub_id_ << ". " << "Num of tf published: " << tf_cnt << std::flush;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_pose_estimator");
  FootPoseEstimator fpe;

  ros::spin();
  std::cout << std::endl;
  return 0;
}