#ifndef GAIT_ANALYZER_H
#define GAIT_ANALYZER_H

#include <k4abttypes.h>
#include <array>
#include <vector>
#include <list>

#include <std_msgs/UInt8.h>
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

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include "COMKF/SystemModel.hpp"
#include "COMKF/PVMeasurementModel.hpp"

typedef float T;
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
  LIST_ENTRY(belt_speed, "The speed at which the treadmill belt is running, in m/s", double, 0.0) \
  LIST_ENTRY(global_frame, "The global frame ID, e.g. map or odom.", std::string, std::string("odom")) \


#define COMKF_PARAM_LIST \
  LIST_ENTRY(sampling_period, "The sampling period that is used by the system equation for prediction.", float, 0.01f)    \
  LIST_ENTRY(system_noise_p, "The standard deviation of noise added to the linear position state.", float, 0.0f)          \
  LIST_ENTRY(system_noise_v, "The standard deviation of noise added to the linear velocity state.", float, 0.09f)        \
  LIST_ENTRY(measurement_noise_p, "The standard deviation of position measurement noise.", float, 0.06f)                  \
  LIST_ENTRY(measurement_noise_v, "The standard deviation of velocity measurement noise.", float, 0.1f)                  \


namespace com_kf {
  using State = KalmanExamples::COMKF::State<T>;
  using Control = KalmanExamples::COMKF::Control<T>;
  using SystemModel = KalmanExamples::COMKF::SystemModel<T>;

  using Measurement = KalmanExamples::COMKF::PVMeasurement<T>;
  using MeasurementModel = KalmanExamples::COMKF::PVMeasurementModel<T>;

  using ExtendedKalmanFilter = Kalman::ExtendedKalmanFilter<State>;

  struct KalmanFilterParams 
  {
    // Print the value of all parameters
    void print();

    // Parameters
    #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
      COMKF_PARAM_LIST
    #undef LIST_ENTRY
  };

  struct KalmanFilter: public ExtendedKalmanFilter
  {
    KalmanFilter();
    
    /**
     * @brief Predict the state with customized step length
     * 
     * @param u The control input vector.
     * @param dt The step length. Will be ignored if negative.
     */
    const State& predict(const Control & u, T dt = T(-1.0));
    const State& update(const Measurement& zpv);

    // Set covariance matrices
    void setSystemCov(T sigma_p, T sigma_v, T sampling_period);
    void setMeasurementCov(T sigma_p, T sigma_v);


    static SystemModel sys;
    static MeasurementModel mm;
  };
}

enum left_right_t {LEFT = 0, RIGHT, LEFT_RIGHT};
enum ankle_foot_t {FOOT = 0, ANKLE, FOOT_ANKLE};


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
typedef std::array<std::array<tf2::Vector3, LEFT_RIGHT>, FOOT_ANKLE> vec_refpoints_t;
typedef std::array<tf2::Vector3, LEFT_RIGHT> vec_refvecs_t;
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

typedef tf2::Vector3 cop_t;

// Regex to be replaced with commas: (?<![{\n )+])   (?=[ -])
namespace sport_sole {
  constexpr size_t NUM_PSENSOR = 8;
  constexpr size_t NUM_2XPSENSOR = NUM_PSENSOR * 2;
  std::array<double, NUM_2XPSENSOR> 
    Rho = {0.2106, 0.1996, 0.1648, 0.1562, 0.1497, 0.0174, 0.0128, 0.0865, 0.2106, 0.1996, 0.1648, 0.1562, 0.1497, 0.0174, 0.0128, 0.0865}, 
    Theta = {-0.1064, 0.0662,-0.1415, 0.0465, 0.2570,-1.1735, 1.0175, 0.2395, 0.1064,-0.0662, 0.1415,-0.0465,-0.2570, 1.1735,-1.0175,-0.2395};

  cop_t getCoP(
    const SportSole::_pressures_type & pressures, 
    const vec_refpoints_t & vec_refpoints, 
    const vec_refvecs_t & vec_refvecs);

  class GaitPhaseFSM {
    enum class GaitPhase : uint8_t {
      Swing = 0b00, // Swing
      Stance1 = 0b10, // Heel contact
      Stance2 = 0b11, // Foot flat
      Stance3 = 0b01 // Heel off
    };

    GaitPhase gait_phases_[LEFT_RIGHT];
    const double p_threshold_ = 1000.0;
    
  public:
    GaitPhaseFSM(): 
      gait_phases_{GaitPhase::Stance2, GaitPhase::Stance2}
    {}
      
    void update(const SportSole::_pressures_type & pressures);
    uint8_t getGaitPhase();
  };
}

class GaitAnalyzer
{
public:
  GaitAnalyzer(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));
  void skeletonsCB(const visualization_msgs::MarkerArray& msg);
  void sportSoleCB(const sport_sole::SportSole& msg);
  void updateGaitState(const uint8_t& msgs);
  void updateCoMMeasurement(const ros::Time & stamp);
  void updateCoMEstimate(const ros::Time & stamp, const com_kf::State & x);
  void updateBoS();
  void updateMoS(const com_t & xcom);

  void updateGaitPhase();

private:

  // Define the possible models for calculating CoM
  enum com_model_t {
    COM_MODEL_PELVIS = 0,
    COM_MODEL_14_SEGMENT
  } com_model_;

  sport_sole::GaitPhaseFSM gait_phase_fsm_;
  bool touches_ground_[FOOT_ANKLE][LEFT_RIGHT];

  // Internal state
  vec_joints_t vec_joints_;
  vec_refpoints_t vec_refpoints_;
  vec_refvecs_t vec_refvecs_;
  sport_sole::SportSole::_pressures_type pressures_;


  // The z coordinate of the ground
  double z_ground_;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  GaitAnalyzerParams ga_params_;
  ros::NodeHandle comkf_nh_;
  com_kf::KalmanFilterParams comkf_params_;
  
  //geometry_msgs::Pose pose_estimates_[LEFT_RIGHT];

  // Subscribers
  ros::Subscriber sub_skeletons_;
  message_filters::Subscriber<sport_sole::SportSole> sub_sport_sole_;
  message_filters::Cache<sport_sole::SportSole> cache_sport_sole_;

  // Publishers
  ros::Publisher pub_gait_state_;

  ros::Time stamp_pcom_measurement_;
  com_t pcom_pos_measurement_;
  ros::Publisher pub_pcom_pos_measurement_; // Center of mass projected onto the ground
  comv_t pcom_vel_measurement_;
  ros::Publisher pub_pcom_vel_measurement_; // Center of mass velocity projected onto the ground
  com_t xcom_measurement_;
  ros::Publisher pub_xcom_measurement_; // Extrapolated center of mass calculated from measurements 
  com_t pcom_pos_estimate_;
  ros::Time stamp_pcom_estimate_;
  ros::Publisher pub_pcom_pos_estimate_; // Center of mass projected onto the ground as estimated by the KF
  comv_t pcom_vel_estimate_;
  ros::Publisher pub_pcom_vel_estimate_; // Center of mass projected onto the ground as estimated by the KF
  com_t xcom_estimate_;
  ros::Publisher pub_xcom_estimate_; // Extrapolated center of mass calculated from measurements 

  ros::Publisher pub_ankle_pose_measurements_[LEFT_RIGHT];
  ros::Publisher pub_foot_pose_measurements_[LEFT_RIGHT];

  cop_t cop_;
  ros::Publisher pub_cop_; // Center of Pressure

  bos_t footprints_[LEFT_RIGHT];
  ros::Publisher pub_footprints_[LEFT_RIGHT];
  bos_t bos_points_;
  ros::Publisher pub_bos_; // Base of support polygon
  mos_t mos_;
  ros::Publisher pub_mos_; // Margin of stability
  ros::Publisher pub_mos_value_measurements_[3]; // Margin of stability
  ros::Publisher pub_mos_value_estimates_[3]; // Margin of stability

  ros::Publisher pub_ground_clearance_left_; // ground clearance
  ros::Publisher pub_ground_clearance_right_; // ground clearance
  
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform tf_depth_to_global_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // comkf
  com_kf::KalmanFilter comkf_;
  bool comkf_initialized_;
  ros::Time stamp_sport_sole_prev_;
  ros::Time stamp_skeleton_prev_;
  ros::Time stamp_base_;

  // Subject selection
  int32_t sub_id_;

private:
  // Helper method for converting a tf2::Vector3 object to a geometry_msgs::Point32 object
  geometry_msgs::Point32 vector3ToPoint32(const tf2::Vector3 & vec);
  geometry_msgs::PointStamped constructPointStampedMessage(const ros::Time & stamp, const tf2::Vector3 & vec);
  geometry_msgs::Vector3Stamped constructVector3StampedMessage(const ros::Time & stamp, const comv_t & vec);
  geometry_msgs::PolygonStamped constructBosPolygonMessage(const ros::Time & stamp);
  visualization_msgs::MarkerArrayPtr constructMosMarkerArrayMessage(const ros::Time & stamp, const com_t & xcom);
};



std::ostream & operator<<(std::ostream & lhs, tf2::Vector3 v)
{
  return lhs << "(" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")";
}

#endif