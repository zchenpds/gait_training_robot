#ifndef FOOT_POSE_ESTIMATOR_H
#define FOOT_POSE_ESTIMATOR_H

#include <k4abttypes.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sport_sole/SportSole.h>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include <sport_sole_ekf/ExtendedKalmanFilter.hpp>

#include <gait_training_robot/SportSoleEkfState.h>
#include <gait_training_robot/SportSoleEkfSportSoleMeasurement.h>
#include <gait_training_robot/SportSoleEkfKinectMeasurement.h>

#include <sport_sole/sport_sole_common.h>

#include "robust_estimator.h"
#include <deque>


typedef double FloatType;

#define ROS_PARAM_LIST                                                                                                           \
  LIST_ENTRY(relay_k4a_measurement, "If set to true, the filter state will assigned k4a measurement.", bool, true)               \
  LIST_ENTRY(system_noise_q, "The standard deviation of noise added to the quaternion state.", FloatType, 1e-3)                  \
  LIST_ENTRY(system_noise_w, "The standard deviation of noise added to the angular velocity state.", FloatType, 7e-2)            \
  LIST_ENTRY(system_noise_p, "The standard deviation of noise added to the linear position state.", FloatType, 1e-3)             \
  LIST_ENTRY(system_noise_v, "The standard deviation of noise added to the linear velocity state.", FloatType, 1e-3)             \
  LIST_ENTRY(system_noise_a, "The standard deviation of noise added to the linear acceleration state.", FloatType, 7e0)          \
  LIST_ENTRY(system_noise_ab, "The standard deviation of noise added to the accelerometer bias state.", FloatType, 1e-2)         \
  LIST_ENTRY(system_noise_wb, "The standard deviation of noise added to the gyroscope bias state.", FloatType, 1e-5)             \
  LIST_ENTRY(measurement_noise_a, "The standard deviation of acceleration measurement noise.", FloatType, 5e-1)                  \
  LIST_ENTRY(measurement_noise_g, "The standard deviation of gyroscope measurement noise.", FloatType, 5e-2)                     \
  LIST_ENTRY(measurement_noise_p, "The standard deviation of position measurement noise.", FloatType, 1e-2)                      \
  LIST_ENTRY(measurement_noise_v, "The standard deviation of velocity measurement noise.", FloatType, 1e-1)                      \
  LIST_ENTRY(measurement_noise_q, "The standard deviation of quaternion measurement noise.", FloatType, 2e-1)                    \
  LIST_ENTRY(measurement_noise_y, "The standard deviation of yaw measurement noise.", FloatType, 1e-2)                           \
  LIST_ENTRY(measurement_noise_va, "The standard deviation of va measurement noise.", FloatType, 1e-5)                           \
  LIST_ENTRY(confidence_pos_a_priori_alpha, "Test.", FloatType, 1e-1)                                                            \
  LIST_ENTRY(robustness_factor_swing, "Test.", FloatType, 0.05)                                                                  \
  LIST_ENTRY(outlier_threashold, "The residual (in sigma) beyond which the measurement is considered an outlier.", FloatType, 3.0)  \
  LIST_ENTRY(global_frame, "The reference frame for the filter output.", std::string, std::string("odom"))                       \
  LIST_ENTRY(publish_frame, "The reference frame for pose messages.", std::string, std::string("odom"))                          \
  LIST_ENTRY(sport_sole_time_offset, "The reference frame for pose messages.", double , 0.0)                                     \
  LIST_ENTRY(enable_debug_log, "If set to true, verbose log will be saved at '~/.ros/gait_training_robot/fpe.log'.", bool, false)\


struct FootPoseEstimatorParams
{
  // Print the value of all parameters
  void print();

  // Parameters
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
    ROS_PARAM_LIST
  #undef LIST_ENTRY
};


class FootPoseEstimator
{
  typedef sport_sole::ExtendedKalmanFilter<FloatType> ekf_t;
public:
  FootPoseEstimator(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));
  void skeletonsCB(const visualization_msgs::MarkerArray& msg);
  void sportSoleCB(const sport_sole::SportSole& msg);
  void updateTf(const ros::Time& stamp);

protected:
  void sportSoleUpdate(sport_sole::SportSoleConstPtr msg_ptr);
  void kinectUpdate(geometry_msgs::TransformStampedConstPtr msg_ptrs[LEFT_RIGHT],
                    geometry_msgs::TransformStampedConstPtr prev_msg_ptrs[LEFT_RIGHT]);
  void publishFusedPoses(const ros::Time& stamp);
  void printDebugMessage(const char* message, const ros::Time& stamp, left_right_t lr) const;
  geometry_msgs::PoseWithCovarianceStampedPtr constructPoseWithCovarianceStamped(tf2::Vector3 position, tf2::Quaternion quat) const;
  
private:
  // params 
  FootPoseEstimatorParams params_;

  // Subject selection
  int32_t sub_id_;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscribers
  ros::Subscriber sub_skeletons_;
  message_filters::Cache<geometry_msgs::TransformStamped> cache_kinect_measurements_[LEFT_RIGHT];
  ros::Subscriber sub_sport_sole_;
  message_filters::Cache<sport_sole::SportSole> cache_sport_sole_;

  // Stance Phase M-Estimator
  StancePhaseMEstimator<ekf_t::ZP> spme_[LEFT_RIGHT];
  // std::deque<ros::Time> spme_ts_queue_[LEFT_RIGHT];

  // SportSoleEKF
  ekf_t ekf_[LEFT_RIGHT];
  FloatType confidence_pos_a_priori_[LEFT_RIGHT] = {0.0, 0.0};
  FloatType zg_y_max_[LEFT_RIGHT];
  ros::Publisher pub_fused_poses_[LEFT_RIGHT];
  ros::Publisher pub_raw_poses_[LEFT_RIGHT];
  ros::Publisher pub_ekf_state_[LEFT_RIGHT];
  ros::Publisher pub_ekf_sport_sole_measurement_[LEFT_RIGHT];
  ros::Publisher pub_ekf_kinect_measurement_[LEFT_RIGHT];
  sport_sole::GaitPhase current_gait_phase_[LEFT_RIGHT] = {sport_sole::GaitPhase::Stance2,sport_sole::GaitPhase::Stance2};
  sport_sole::GaitPhase previous_gait_phase_[LEFT_RIGHT] = {sport_sole::GaitPhase::Stance2,sport_sole::GaitPhase::Stance2};
  tf2::Vector3 refUnitY[LEFT_RIGHT] = {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}};

  gait_training_robot::SportSoleEkfSportSoleMeasurement msg_sport_sole_measurement_[LEFT_RIGHT];
  gait_training_robot::SportSoleEkfKinectMeasurement msg_kinect_measurement_[LEFT_RIGHT];
  gait_training_robot::SportSoleEkfState msg_state_[LEFT_RIGHT];

  // Incident counter
  sport_sole::IncidentCounter incident_counter_[LEFT_RIGHT];

  // ts
  ros::Time ts_kinect_last_;
  ros::Time ts_sport_sole_last_;
  ros::Time ts_predict_last_;
  ros::Time ts_init_;
  ros::Time ts_next_desired_publish_;
  ros::Time ts_last_quaternion_update_;

  // tf broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // tf listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform tf_global_to_publish_;
  // Becomes true if the publish frame is different from the global frame and the tf lookup succeeds
  bool is_initialized_tf_global_to_publish_;

  sport_sole::GaitPhaseFSM2 gait_phase_fsm_;
};



#endif // End of FOOT_POSE_ESTIMATOR_H