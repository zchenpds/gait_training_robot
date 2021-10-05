#ifndef KINECT_POSE_ESTIMATOR_H
#define KINECT_POSE_ESTIMATOR_H


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include "kpekf/ExtendedKalmanFilter.hpp"
#include "gait_training_robot/kalman_filter_common.h"

typedef double FloatType;

#define PARAM_LIST \
  LIST_ENTRY(kimu_frame, "The Kinect IMU frame ID, e.g. imu_link.", std::string, std::string("imu_link"))      \
  LIST_ENTRY(global_frame, "The global frame ID, e.g. odom.", std::string, std::string("odom"))                                  \
  LIST_ENTRY(output_frame, "The global frame ID, e.g. fused_odom.", std::string, std::string("fused_odom"))                      \
  LIST_ENTRY(system_noise_q, "The standard deviation of noise added to the quaternion state.", FloatType, 1e-3)                  \
  LIST_ENTRY(system_noise_w, "The standard deviation of noise added to the angular velocity state.", FloatType, 7e-2)            \
  LIST_ENTRY(system_noise_p, "The standard deviation of noise added to the linear position state.", FloatType, 1e-3)             \
  LIST_ENTRY(system_noise_v, "The standard deviation of noise added to the linear velocity state.", FloatType, 1e-3)             \
  LIST_ENTRY(system_noise_a, "The standard deviation of noise added to the linear acceleration state.", FloatType, 7e0)          \
  LIST_ENTRY(system_noise_ab, "The standard deviation of noise added to the accelerometer bias state.", FloatType, 1e-2)         \
  LIST_ENTRY(system_noise_wb, "The standard deviation of noise added to the gyroscope bias state.", FloatType, 1e-5)             \
  LIST_ENTRY(measurement_noise_a, "The standard deviation of acceleration measurement noise.", FloatType, 1e-1)                  \
  LIST_ENTRY(measurement_noise_g, "The standard deviation of gyroscope measurement noise.", FloatType, 5e-2)                     \
  LIST_ENTRY(measurement_noise_p, "The standard deviation of position measurement noise.", FloatType, 1e-2)                      \
  LIST_ENTRY(measurement_noise_v, "The standard deviation of velocity measurement noise.", FloatType, 1e-1)                      \
  LIST_ENTRY(measurement_noise_q, "The standard deviation of quaternion measurement noise.", FloatType, 2e-1)                    \
  LIST_ENTRY(measurement_noise_y, "The standard deviation of yaw measurement noise.", FloatType, 1e-1)                           \
  LIST_ENTRY(measurement_noise_va, "The standard deviation of va measurement noise.", FloatType, 1e-5)                           \
  LIST_ENTRY(odom_lead_time, "The time offset of wheel odometry.", FloatType, 0.0)                                               \
 


struct KinectPoseEstimatorParams 
{
  // Print the value of all parameters
  void print();

  // Parameters
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
    PARAM_LIST
  #undef LIST_ENTRY
};



class KinectPoseEstimator
{
  typedef kpekf::ExtendedKalmanFilter<FloatType> ekf_t;
public:
  KinectPoseEstimator(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));
  ~KinectPoseEstimator();
  void odomCB(const nav_msgs::Odometry & msg);
  void kimuCB(const sensor_msgs::Imu & msg);
  void kimuPredictAndUpdate(const ros::Time& stamp);
  void odomUpdate(const ros::Time& stamp);
  void broadcastTf();

private:
  // node_handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // tf
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped pose_estimate_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform tf_global_to_kimu_;
  tf2::Transform tf_global_to_optitrack_;
  bool tf_global_to_optitrack_initialized_{false};

  // State
  ekf_t ekf_;
  bool ekf_initialized_ = false;
  tf2::Vector3 refUnitY_ = {0., 1., 0.};
  Averager<ekf_t::ZA> averager_za_;
  Averager<ekf_t::ZG> averager_zg_;
  tf2::Vector3 zv_ros_last_ = {0., 0., 0.};

  // stats
  int num_of_kimu_messages_ = 0;
  int num_of_predictions_between_odom_updates_ = 0;
  
  // Subscribers
  message_filters::Subscriber<sensor_msgs::Imu> sub_kimu_;
  message_filters::Cache<sensor_msgs::Imu> cache_kimu_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
  message_filters::Cache<nav_msgs::Odometry> cache_odom_;

  // Publishers
  ros::Publisher pub_filtered_odom_;
  nav_msgs::Odometry odom_filtered_;
  ros::Publisher pub_ekf_state_;
  ros::Publisher pub_ekf_odom_measurement_;
  ros::Publisher pub_ekf_kimu_measurement_;


  // ts
  bool kimu_received_ = false;
  ros::Time ts_ekf_;
  ros::Time ts_odom_update_last_;
  ros::Time ts_odom_last_;
  ros::Time ts_kimu_last_;
  ros::Time ts_predict_last_;
  ros::Time ts_init_;
  ros::Time ts_next_desired_publish_;

  // params
  const ros::Duration desired_publish_period_{1.0 / 100.0};
  KinectPoseEstimatorParams params_;
};


#endif // KINECT_POSE_ESTIMATOR_H