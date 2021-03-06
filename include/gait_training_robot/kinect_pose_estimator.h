#ifndef KINECT_POSE_ESTIMATOR_H
#define KINECT_POSE_ESTIMATOR_H


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#define PARAM_LIST \
  LIST_ENTRY(output_frame, "The global frame ID, e.g. fused_odom.", std::string, std::string("fused_odom"))    \

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
public:
  KinectPoseEstimator(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));
  ~KinectPoseEstimator();
  void odomCB(const nav_msgs::Odometry & msg);
  void kimuCB(const sensor_msgs::Imu & msg);
  void broadcastTf();

private:
  // node_handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // tf
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped pose_estimate_;

  // State
  struct State
  {
    bool ready = false;
    ros::Time t;
    double x;
    double y;
    double th;
    double vx;
    double bax;
    double bwz;
    void predict(double ax, double wz, double dt);
    State& operator=(const State& rhs);
  } state_, state_posterior_;
  

  // Subscribers
  message_filters::Subscriber<sensor_msgs::Imu> sub_kimu_;
  message_filters::Cache<sensor_msgs::Imu> cache_kimu_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
  message_filters::Cache<nav_msgs::Odometry> cache_odom_;

  // Publishers
  ros::Publisher pub_filtered_odom_;
  nav_msgs::Odometry odom_filtered_;

  // ts
  ros::Time ts_odom_last_;
  ros::Time ts_kimu_last_;
  ros::Time ts_init_;
  ros::Time ts_next_desired_publish_;

  // params
  const ros::Duration desired_publish_period_{1.0 / 100.0};
  KinectPoseEstimatorParams params_;
};

template<typename T>
T wrapToPi(T angle)
{
  return atan2(sin(angle), cos(angle));
}


#endif // KINECT_POSE_ESTIMATOR_H