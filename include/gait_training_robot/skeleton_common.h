#ifndef SKELETON_PUBLISHER_H
#define SKELETON_PUBLISHER_H

#include <utility>
#include <k4abt.hpp>
#include <array>

struct BodySegment
{
  std::pair<k4abt_joint_id_t, k4abt_joint_id_t> joint_pair_;
  double weight_ratio_;
  double com_len_ratio_;
    
public:
  /* BodySegment Constructor
  * \param joint_pair: the pair of joints that define the segment
  * \param weight_ratio: the weight of the segment divided by the weight of the whole body
  * \param com_len_ratio: the segment CoM distance to the proximal joint divided by the segment length
  */
  BodySegment(std::pair<k4abt_joint_id_t, k4abt_joint_id_t> && joint_pair, double weight_ratio, double com_len_ratio):
      joint_pair_(joint_pair),
      weight_ratio_(weight_ratio),
      com_len_ratio_(com_len_ratio)
  {}
};


// Define the bone list based on the documentation
constexpr size_t body_segment_count = 31;
const std::array<BodySegment, body_segment_count> vec_segments =
{
  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_SPINE_NAVEL), 0.139, 0.44), // Abdomen (T12-L1/L4-L5)
  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_PELVIS), 0.142, 0.1), // Pelvis (L4-L5/greater trochanter)
  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_NECK), 0.216, 0.18), // Thorax (C7-T1/T12-L1), reverse
  BodySegment(std::make_pair(K4ABT_JOINT_NECK, K4ABT_JOINT_HEAD), 0.081, 1.4), // Head and neck
  BodySegment(std::make_pair(K4ABT_JOINT_HEAD, K4ABT_JOINT_NOSE), 0.0, 0.0),

  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_CLAVICLE_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_ELBOW_LEFT), 0.028, 0.436), // Upper arm, left
  BodySegment(std::make_pair(K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_WRIST_LEFT), 0.022, 0.682), // Forearm and hand, left
  BodySegment(std::make_pair(K4ABT_JOINT_WRIST_LEFT, K4ABT_JOINT_HAND_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_HAND_LEFT, K4ABT_JOINT_HANDTIP_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_WRIST_LEFT, K4ABT_JOINT_THUMB_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_KNEE_LEFT), 0.100, 0.433), // Thigh, left
  BodySegment(std::make_pair(K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_ANKLE_LEFT), 0.0465, 0.433), // Leg, left
  BodySegment(std::make_pair(K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_FOOT_LEFT), 0.0145, 0.5), // Foot, left
  BodySegment(std::make_pair(K4ABT_JOINT_NOSE, K4ABT_JOINT_EYE_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_EYE_LEFT, K4ABT_JOINT_EAR_LEFT), 0.0, 0.0),

  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_CLAVICLE_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_ELBOW_RIGHT), 0.028, 0.436), // Upper arm, right
  BodySegment(std::make_pair(K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_WRIST_RIGHT), 0.022, 0.682), // Forearm and hand, right
  BodySegment(std::make_pair(K4ABT_JOINT_WRIST_RIGHT, K4ABT_JOINT_HAND_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_HAND_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_WRIST_RIGHT, K4ABT_JOINT_THUMB_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT), 0.100, 0.433), // Thigh, right
  BodySegment(std::make_pair(K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT), 0.0465, 0.433), // Leg, right
  BodySegment(std::make_pair(K4ABT_JOINT_ANKLE_RIGHT, K4ABT_JOINT_FOOT_RIGHT), 0.0145, 0.5), // Foot, right
  BodySegment(std::make_pair(K4ABT_JOINT_NOSE, K4ABT_JOINT_EYE_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_EYE_RIGHT, K4ABT_JOINT_EAR_RIGHT), 0.0, 0.0)
};



void correctAnkleMarkerOrientation(visualization_msgs::Marker* foot_marker_msg, int joint_id)
{
  if (joint_id != K4ABT_JOINT_ANKLE_LEFT && joint_id != K4ABT_JOINT_ANKLE_RIGHT) return;

  // Convert messages to tf2 objects
  geometry_msgs::Quaternion& quat_msg = foot_marker_msg->pose.orientation;
  tf2::Quaternion quat;
  tf2::fromMsg(quat_msg, quat);

  // Correct the orientation
  if (joint_id == K4ABT_JOINT_ANKLE_LEFT)
  {
    quat *= tf2::Quaternion({1.0, 1.0, 1.0}, M_PI / 3 * 2);
  }
  else
  {
    quat *= tf2::Quaternion({-1.0, 1.0, 1.0}, -M_PI / 3 * 2);
  }

  // Update the messages with tf2 objects
  quat_msg = tf2::toMsg(quat);
}

void correctAnkleMarkerDisplacement(visualization_msgs::Marker* foot_marker_msg)
{
  // Convert messages to tf2 objects
  geometry_msgs::Quaternion& quat_msg = foot_marker_msg->pose.orientation;
  tf2::Quaternion quat;
  tf2::fromMsg(quat_msg, quat);
  geometry_msgs::Point& position_msg = foot_marker_msg->pose.position;
  tf2::Vector3 position;
  tf2::fromMsg(position_msg, position);

  // Correct the displacement
  position += tf2::quatRotate(quat, {0.1, 0.0, -0.1});

  // Update the messages with tf2 objects
  quat_msg = tf2::toMsg(quat);
  tf2::toMsg(position, position_msg);
}

void correctShoeMarkerDisplacement(visualization_msgs::Marker* foot_marker_msg)
{
  // Convert messages to tf2 objects
  geometry_msgs::Quaternion& quat_msg = foot_marker_msg->pose.orientation;
  tf2::Quaternion quat;
  tf2::fromMsg(quat_msg, quat);
  geometry_msgs::Point& position_msg = foot_marker_msg->pose.position;
  tf2::Vector3 position;
  tf2::fromMsg(position_msg, position);

  // Correct the displacement
  position += tf2::quatRotate(quat, {-0.1, 0.0, 0.1});

  // Update the messages with tf2 objects
  quat_msg = tf2::toMsg(quat);
  tf2::toMsg(position, position_msg);
}


void correctShoeMarkerOrientation(visualization_msgs::Marker* foot_marker_msg, int joint_id)
{
  if (joint_id != K4ABT_JOINT_ANKLE_LEFT && joint_id != K4ABT_JOINT_ANKLE_RIGHT) return;

  // Convert messages to tf2 objects
  geometry_msgs::Quaternion& quat_msg = foot_marker_msg->pose.orientation;
  tf2::Quaternion quat;
  tf2::fromMsg(quat_msg, quat);

  // Correct the orientation (mirror and rotate)
  if (joint_id == K4ABT_JOINT_ANKLE_RIGHT)
  {
    foot_marker_msg->scale.x *= -1.0;
  }
  quat *= tf2::Quaternion({0.0, 0.0, 1.0}, -M_PI / 2);

  // Update the messages with tf2 objects
  quat_msg = tf2::toMsg(quat);
}

void correctShoeMarkerOrientation(visualization_msgs::Marker* foot_marker_msg, left_right_t lr)
{
  correctShoeMarkerOrientation(foot_marker_msg, lr == LEFT ? K4ABT_JOINT_ANKLE_LEFT: K4ABT_JOINT_ANKLE_RIGHT);
}


#endif // SKELETON_PUBLISHER_H