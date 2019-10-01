#ifndef GAIT_ANALYZER_H
#define GAIT_ANALYZER_H

#include <k4abttypes.h>
#include <array>
#include <vector>
#include <list>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <utils/differentiator.h>

#include "sport_sole/SportSole.h"

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

// The format of these list entries is :
//
// LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)
//
// param_variable: the variable name which will be created in the k4a_ros_device class to hold the contents of the parameter
// param_help_string: a string containing help information for the parameter
// param_type: the type of the parameter
// param_default_val: the default value of the parameter
//
// Example:
// LIST_ENTRY(sensor_sn, "The serial number of the sensor this node should connect with", std::string, std::string(""))
#define ROS_PARAM_LIST \
  LIST_ENTRY(belt_speed, "The speed at which the treadmill belt is running, in m/s", double, 0.0)

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
const std::array<BodySegment, 25> vec_segments =
{
  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_SPINE_NAVAL), 0.139, 0.44), // Abdomen (T12-L1/L4-L5)
  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_NAVAL, K4ABT_JOINT_PELVIS), 0.142, 0.1), // Pelvis (L4-L5/greater trochanter)
  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_NECK), 0.216, 0.18), // Thorax (C7-T1/T12-L1), reverse
  BodySegment(std::make_pair(K4ABT_JOINT_NECK, K4ABT_JOINT_HEAD), 0.081, 1.4), // Head and neck
  BodySegment(std::make_pair(K4ABT_JOINT_HEAD, K4ABT_JOINT_NOSE), 0.0, 0.0),

  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_CLAVICLE_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_ELBOW_LEFT), 0.028, 0.436), // Upper arm, left
  BodySegment(std::make_pair(K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_WRIST_LEFT), 0.022, 0.682), // Forearm and hand, left
  BodySegment(std::make_pair(K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_KNEE_LEFT), 0.100, 0.433), // Thigh, left
  BodySegment(std::make_pair(K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_ANKLE_LEFT), 0.0465, 0.433), // Leg, left
  BodySegment(std::make_pair(K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_FOOT_LEFT), 0.0145, 0.5), // Foot, left
  BodySegment(std::make_pair(K4ABT_JOINT_NOSE, K4ABT_JOINT_EYE_LEFT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_EYE_LEFT, K4ABT_JOINT_EAR_LEFT), 0.0, 0.0),

  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_CLAVICLE_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_ELBOW_RIGHT), 0.0, 0.0), // Upper arm, right
  BodySegment(std::make_pair(K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_WRIST_RIGHT), 0.022, 0.682), // Forearm and hand, right
  BodySegment(std::make_pair(K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT), 0.100, 0.433), // Thigh, right
  BodySegment(std::make_pair(K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT), 0.0465, 0.433), // Leg, right
  BodySegment(std::make_pair(K4ABT_JOINT_ANKLE_RIGHT, K4ABT_JOINT_FOOT_RIGHT), 0.0145, 0.5), // Foot, right
  BodySegment(std::make_pair(K4ABT_JOINT_NOSE, K4ABT_JOINT_EYE_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_EYE_RIGHT, K4ABT_JOINT_EAR_RIGHT), 0.0, 0.0)
};

typedef std::array<tf2::Vector3,K4ABT_JOINT_COUNT> vec_joints_t;
typedef tf2::Vector3 com_t;
typedef tf2::Vector3 comv_t;
typedef std::vector<tf2::Vector3> bos_t;

// Margin of stability struct
struct mos_t
{
  enum {
    mos_anteroposterior = 0, 
    mos_mediolateral, 
    mos_shortest,
    mos_count
  };

  struct {
    // The point on the BoS polygon with the shortest distance to XCoM
    tf2::Vector3 pt; 
    // The distance from XCoM to pt.
    // Positive indicates pt is inside BoS.
    double dist;
  } values[mos_count];
};

class GaitAnalyzer
{
public:
  GaitAnalyzer();
  void skeletonsCB(const visualization_msgs::MarkerArray& msg);
  void sportSoleCB(const sport_sole::SportSole& msg);
  void updateGaitState(const uint8_t& msgs);
  com_t getCoM();
  comv_t getCoMv(const com_t & com, ros::Time ts);
  bos_t getBoS();
  mos_t getMoS(const bos_t & bos, const com_t & xcom);

  void updateGaitPhase();

private:
  enum left_right_t {LEFT = 0, RIGHT, LEFT_RIGHT};
  enum ankle_foot_t {FOOT = 0, ANKLE, FOOT_ANKLE};

  // Define the possible models for calculating CoM
  enum com_model_t {
    COM_MODEL_PELVIS = 0,
    COM_MODEL_14_SEGMENT
  } com_model_;

  // Define the phases in a gait cycle
  enum gait_phase_t {
    GAIT_PHASE_UNKNOWN = 0,
    GAIT_PHASE_HEEL_STRIKE,
    GAIT_PHASE_FOOT_FLAT,
    GAIT_PHASE_HEEL_OFF,
    GAIT_PHASE_SWING
  } gait_phase_[LEFT_RIGHT];

  bool touches_ground_[FOOT_ANKLE][LEFT_RIGHT];

  // Internal state
  vec_joints_t vec_joints_;


  // The z coordinate of the ground
  double z_ground_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_skeletons_;
  message_filters::Subscriber<sport_sole::SportSole> sub_sport_sole_;
  message_filters::Cache<sport_sole::SportSole> cache_sport_sole_;

  ros::Publisher pub_pcom_; // Center of mass projected onto the ground
  ros::Publisher pub_xcom_; // Extrapolated center of mass projected onto the ground  
  ros::Publisher pub_bos_; // Base of support polygon
  ros::Publisher pub_mos_; // Margin of stability
  ros::Publisher pub_mos_values_[3]; // Margin of stability

  ros::Publisher pub_ground_clearance_left_; // ground clearance
  ros::Publisher pub_ground_clearance_right_; // ground clearance
  

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform tf_depth_to_map_;

  Differentiator comv_differentiator_x_, comv_differentiator_y_;

  // Params
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
    ROS_PARAM_LIST
  #undef LIST_ENTRY

private:
  // Helper method for converting a tf2::Vector3 object to a geometry_msgs::Point32 object
  geometry_msgs::Point32 vector3ToPoint32(const tf2::Vector3 & vec);
};



std::ostream & operator<<(std::ostream & lhs, tf2::Vector3 v)
{
  return lhs << "(" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")";
}

#endif