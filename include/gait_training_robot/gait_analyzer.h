#ifndef GAIT_ANALYZER_H
#define GAIT_ANALYZER_H

#include <k4abttypes.h>
#include <array>
#include <vector>
#include <list>
#include <deque>
#include <numeric>

#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float64.h>

#include "sport_sole/SportSole.h"
#include "sport_sole/GaitState.h"

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "comkf/comkf.hpp"
#include "sport_sole/sport_sole_common.h"

typedef double T;
#include "kalman_filter_common.h"
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
#define GA_PARAM_LIST \
  LIST_ENTRY(belt_speed, "The speed at which the treadmill belt is running, in m/s", double, 0.0)                             \
  LIST_ENTRY(global_frame, "The global frame ID, e.g. map or odom.", std::string, std::string("odom"))                        \
  LIST_ENTRY(publish_frame, "The reference frame for pose messages.", std::string, std::string("odom"))                       \
  LIST_ENTRY(foot_pose_topic, "The topic name for foot poses.", std::string, std::string("/foot_pose_estimator/fused_pose_")) \
  LIST_ENTRY(data_source, "Must be either 'k4a' or 'optitrack'.", std::string, std::string("k4a"))                            \
  LIST_ENTRY(smoother_enabled, "Enable moving average smoother or not. May cause latency.", bool, false)                      \


#define COMKF_PARAM_LIST \
  LIST_ENTRY(sampling_period, "The sampling period that is used by the system equation for prediction.", T, 0.01)     \
  LIST_ENTRY(system_noise_p, "The standard deviation of noise added to the linear position state.", T, 2e-3)          \
  LIST_ENTRY(system_noise_v, "The standard deviation of noise added to the linear velocity state.", T, 2e-2)          \
  LIST_ENTRY(system_noise_b, "The standard deviation of noise added to the CoM offset state.", T, 1e-1)               \
  LIST_ENTRY(system_noise_cop, "The standard deviation of noise added to the CoP state.", T, 3e-1)                    \
  LIST_ENTRY(measurement_noise_p, "The standard deviation of position measurement noise.", T, 2e-2)                   \
  LIST_ENTRY(measurement_noise_v, "The standard deviation of velocity measurement noise.", T, 4e-1)                   \
  LIST_ENTRY(measurement_noise_cop, "The standard deviation of CoP measurement noise.", T, 1e-1)                      \


namespace comkf {
  using S = State<T>;
  using C = Control<T>;
  using ZP = PositionMeasurement<T>;
  struct KalmanFilterParams 
  {
    // Print the value of all parameters
    void print();

    // Parameters
    #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
      COMKF_PARAM_LIST
    #undef LIST_ENTRY
  };

}

struct GaitAnalyzerParams 
{
  // Print the value of all parameters
  void print();

  // Parameters
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
    GA_PARAM_LIST
  #undef LIST_ENTRY
};



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
  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_SPINE_NAVEL), 0.139, 0.44), // Abdomen (T12-L1/L4-L5)
  BodySegment(std::make_pair(K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_PELVIS), 0.142, 0.1), // Pelvis (L4-L5/greater trochanter)
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
  BodySegment(std::make_pair(K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_ELBOW_RIGHT), 0.028, 0.436), // Upper arm, right
  BodySegment(std::make_pair(K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_WRIST_RIGHT), 0.022, 0.682), // Forearm and hand, right
  BodySegment(std::make_pair(K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT), 0.100, 0.433), // Thigh, right
  BodySegment(std::make_pair(K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT), 0.0465, 0.433), // Leg, right
  BodySegment(std::make_pair(K4ABT_JOINT_ANKLE_RIGHT, K4ABT_JOINT_FOOT_RIGHT), 0.0145, 0.5), // Foot, right
  BodySegment(std::make_pair(K4ABT_JOINT_NOSE, K4ABT_JOINT_EYE_RIGHT), 0.0, 0.0),
  BodySegment(std::make_pair(K4ABT_JOINT_EYE_RIGHT, K4ABT_JOINT_EAR_RIGHT), 0.0, 0.0)
};

typedef std::array<tf2::Vector3,K4ABT_JOINT_COUNT> vec_joints_t;
typedef std::array<std::array<tf2::Vector3, LEFT_RIGHT>, FORE_HIND> vec_refpoints_t;
typedef std::array<tf2::Vector3, LEFT_RIGHT> vec_refvecs_t;
typedef tf2::Vector3 com_t;
typedef tf2::Vector3 comv_t;
typedef std::vector<tf2::Vector3> bos_t;

// Margin of stability struct
struct mos_t
{
  enum mos_name_t {
    mos_anteroposterior = 0, 
    mos_mediolateral, 
    mos_shortest,
    mos_count
  };

  static std::string toString(int i)
  {
    mos_name_t name = static_cast<mos_name_t>(i);
    if (name == mos_anteroposterior) return "mos_ap";
    else if (name == mos_mediolateral) return "mos_ml";
    else return "mos";
  }

  struct {
    // The point on the BoS polygon with the shortest distance to XCoM
    tf2::Vector3 pt; 
    // The distance from XCoM to pt.
    // Positive indicates pt is inside BoS.
    double dist;
  } values[mos_count];
};

typedef tf2::Vector3 cop_t;

// Regex to be replaced with commas: (?<![{\n )+])   (?=[ -])
namespace sport_sole {
  cop_t getCoP(
    const SportSole::_pressures_type & pressures, 
    const vec_refpoints_t & vec_refpoints, 
    const vec_refvecs_t & vec_refvecs);
}

template<typename Vector>
class FIRSmoother {
  std::deque<Vector> dq_;
  size_t window_size_;
public:
  FIRSmoother(size_t window_size = 10): window_size_(window_size) {}
  void operator()(Vector & x) {
    while (dq_.size() >= window_size_) dq_.pop_front();
    dq_.push_back(x);
    x.setZero();
    x = std::accumulate(dq_.begin(), dq_.end(), x) / dq_.size();
  }
};

constexpr double FOOT_LENGTH = 0.205;

class GaitAnalyzer
{

public:
  using PoseType   = geometry_msgs::PoseWithCovarianceStamped;
  using PointType  = geometry_msgs::PointStamped;
  using VectorType = geometry_msgs::Vector3Stamped;

  GaitAnalyzer(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));
  void skeletonsCB(const visualization_msgs::MarkerArray& msg);
  void comCB(const geometry_msgs::PointStamped& msg);
  void mlVecCB(const geometry_msgs::Vector3Stamped& msg);
  void timeSynchronizerCB(const PoseType::ConstPtr&, const PoseType::ConstPtr&, const PointType::ConstPtr&, const VectorType::ConstPtr&);
  void sportSoleCB(const sport_sole::SportSole& msg);
  void updateGaitState(const uint8_t& msgs);
  void updateTfCB(const ros::TimerEvent& event);
  // Update both CoM and CoMv measurements
  void updateCoMMeasurementFromSkeletons(const ros::Time & stamp, com_t & com_curr, comv_t & com_vel, const vec_joints_t & vec_joints);
  void updateCoMMeasurement(const ros::Time & stamp, com_t & com_curr, comv_t & com_vel, const geometry_msgs::PointStamped& msg);
  // Update XCoM
  void updateXCoM(com_t & xcom, const com_t & com, const comv_t & com_vel);
  // Update CoM CoMv and XCoM estimates
  void updateCoMEstimate(const ros::Time & stamp, const comkf::S & x);
  void updateBoS(bos_t & res);
  void updateMoS(mos_t & res, const com_t & xcom, const bos_t & bos_points);

  void updateGaitPhase();

private:

  // Define the possible models for calculating CoM
  enum com_model_t {
    COM_MODEL_PELVIS = 0,
    COM_MODEL_14_SEGMENT
  } com_model_;

  sport_sole::GaitPhaseFSM2 gait_phase_fsm_;
  bool touches_ground_[FORE_HIND][LEFT_RIGHT];

  // Internal state
  vec_joints_t vec_joints_;
  vec_refpoints_t vec_refpoints_;
  vec_refvecs_t vec_refvecs_;
  sport_sole::SportSole::_pressures_type pressures_;
  tf2::Vector3 ml_vec_;
  tf2::Vector3 ap_vec_;

  // The z coordinate of the ground
  double z_ground_;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  GaitAnalyzerParams ga_params_;
  ros::NodeHandle comkf_nh_;
  comkf::KalmanFilterParams comkf_params_;
  

  enum measurement_estimate_t{
    MEASUREMENT=0,
    ESTIMATE,
    MEASUREMENT_ESTIMATE
  };
  
  // Subscribers
  ros::Subscriber sub_skeletons_;
  ros::Subscriber sub_com_;
  ros::Subscriber sub_ml_vec_;
  message_filters::Subscriber<sport_sole::SportSole>                            sub_sport_sole_;
  message_filters::Cache     <sport_sole::SportSole>                          cache_sport_sole_;
  message_filters::Subscriber<PoseType>                                         sub_foot_poses_[LEFT_RIGHT];
  message_filters::TimeSynchronizer<PoseType, PoseType, PointType, VectorType>  time_synchronizer_;

  // Publishers
  ros::Publisher pub_gait_state_;

  FIRSmoother<com_t>   com_smoother_[MEASUREMENT_ESTIMATE];
  FIRSmoother<comv_t> comv_smoother_[MEASUREMENT_ESTIMATE];

  com_t               com_[MEASUREMENT_ESTIMATE];
  ros::Publisher  pub_com_[MEASUREMENT_ESTIMATE];  // Center of mass projected onto the ground
  comv_t             comv_[MEASUREMENT_ESTIMATE];
  ros::Publisher pub_comv_[MEASUREMENT_ESTIMATE]; // Center of mass velocity projected onto the ground
  com_t              xcom_[MEASUREMENT_ESTIMATE];
  ros::Publisher pub_xcom_[MEASUREMENT_ESTIMATE];  // Extrapolated center of mass calculated from measurements 

  ros::Publisher pub_hindfoot_refpoint_[LEFT_RIGHT];
  ros::Publisher pub_forefoot_refpoint_[LEFT_RIGHT];

  cop_t               cop_;
  ros::Publisher  pub_cop_; // Center of Pressure

  bos_t              footprints_[LEFT_RIGHT];
  ros::Publisher pub_footprints_[LEFT_RIGHT];
  bos_t              bos_points_;
  ros::Publisher pub_bos_; // Base of support polygon
  mos_t              mos_       [MEASUREMENT_ESTIMATE];
  ros::Publisher pub_mos_       [MEASUREMENT_ESTIMATE]; // Margin of stability
  ros::Publisher pub_mos_vector_[MEASUREMENT_ESTIMATE]; // Margin of stability

  ros::Publisher pub_ground_clearance_left_; // ground clearance
  ros::Publisher pub_ground_clearance_right_; // ground clearance

  ros::Publisher pub_comkf_state_;
  ros::Publisher pub_comkf_cop_measurement_;
  ros::Publisher pub_comkf_com_measurement_;
  
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform tf_depth_to_global_;
  tf2::Transform tf_imu_to_local_; // from imu to depth
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2::Transform tf_global_to_publish_;
  // Becomes true if the publish frame is different from the global frame and the tf lookup succeeds
  bool is_initialized_tf_global_to_publish_;
  ros::Timer timer_update_tf_global_to_publish_;

  // comkf
  comkf::KalmanFilter<T> comkf_;
  bool comkf_initialized_;
  ros::Time stamp_sport_sole_prev_;
  ros::Time stamp_predict_prev_;
  ros::Time stamp_com_prev_;
  ros::Time stamp_base_;

  // Subject selection
  int32_t sub_id_;

private:
  // Helper method for converting a tf2::Vector3 object to a geometry_msgs::Point32 object
  geometry_msgs::Point32 vector3ToPoint32(const tf2::Vector3 & vec);
  geometry_msgs::PointStampedPtr constructPointStampedMessage(const ros::Time & stamp, const tf2::Vector3 & vec);
  geometry_msgs::Vector3StampedPtr constructVector3StampedMessage(const ros::Time & stamp, const comv_t & vec);
  geometry_msgs::PolygonStamped constructPolygonMessage(const ros::Time & stamp, const bos_t & bos_points);
  visualization_msgs::MarkerArrayPtr constructMosMarkerArrayMessage(const ros::Time & stamp, const com_t & xcom, const mos_t & mos);

  inline tf2::Vector3 constructMosVector(const mos_t& mos)
  {
    return tf2::Vector3(
      mos.values[mos_t::mos_shortest].dist,
      mos.values[mos_t::mos_anteroposterior].dist,
      mos.values[mos_t::mos_mediolateral].dist );
  }
};



std::ostream & operator<<(std::ostream & lhs, const tf2::Vector3 & v)
{
  return lhs << "(" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")";
}

tf2::Matrix3x3 constructCrossProdMatrix(const tf2::Vector3 & v) {
  return tf2::Matrix3x3( 0.0,   -v.z(),  v.y(),
                         v.z(),  0.0,   -v.x(),
                        -v.y(),  v.x(),  0.0   );
}

// #define PUBLISH_MOS_MEASUREMENT
// #define PRINT_MOS_DELAY
#endif