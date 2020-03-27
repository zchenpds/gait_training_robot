#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#define _USE_MATH_DEFINES
#include <cmath>

#include <array>
#include <fstream>

#include "SportSoleSimple/SystemModel.hpp"
#include "SportSoleSimple/PositionMeasurementModel.hpp"
#include "SportSoleSimple/OrientationMeasurementModel.hpp"
#include "SportSoleSimple/VelocityMeasurementModel.hpp"
#include "SportSoleSimple/ExtendedErrorStateKalmanFilter.hpp"

#include "sport_sole/SportSole.h"
//#include <kalman/ExtendedKalmanFilter.hpp>

#include <ros/ros.h>
//#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h> 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

typedef float T;
#include "kalman_filter_common.h"


// LIST_ENTRY is:
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// The format of these list entries is :
//
// LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)
//
// param_variable: the variable name which will be created in the k4a_ros_device class to hold the contents of the
//    parameter
// param_help_string: a string containing help information for the parameter
// param_type: the type of the parameter
// param_default_val: the default value of the parameter
//
// Example:
// LIST_ENTRY(sensor_sn, "The serial number of the sensor this node should connect with", std::string, std::string(""))
#define ROS_PARAM_LIST                                                                                                    \
  LIST_ENTRY(sampling_period, "The sampling period that is used by the system equation for prediction.", float, 0.01f)    \
  LIST_ENTRY(system_noise_p, "The standard deviation of noise added to the linear position state.", float, 0.0f)          \
  LIST_ENTRY(system_noise_v, "The standard deviation of noise added to the linear velocity state.", float, 0.001f)        \
  LIST_ENTRY(system_noise_th, "The standard deviation of noise added to the angular position state.", float, 0.0f)        \
  LIST_ENTRY(system_noise_ba, "The standard deviation of noise added to the acc bias state.", float, 0.01f)               \
  LIST_ENTRY(measurement_noise_p, "The standard deviation of position measurement noise.", float, 0.02f)                  \
  LIST_ENTRY(measurement_noise_v, "The standard deviation of velocity measurement noise.", float, 0.01f)                  \
  LIST_ENTRY(measurement_noise_th, "The standard deviation of orientation measurement noise.", float, 0.2f)               \
  LIST_ENTRY(global_frame, "The reference frame for the filter output.", std::string, std::string("odom"))                \



namespace sport_sole {
  using State = KalmanExamples::SportSole::State<T>;
  using Control = KalmanExamples::SportSole::Control<T> ;
  using SystemModel = KalmanExamples::SportSole::SystemModel<T> ;

  using PositionMeasurement = KalmanExamples::SportSole::PositionMeasurement<T>;
  using PositionModel = KalmanExamples::SportSole::PositionMeasurementModel<T> ;

  using OrientationMeasurement = KalmanExamples::SportSole::OrientationMeasurement<T>;
  using OrientationModel = KalmanExamples::SportSole::OrientationMeasurementModel<T>;

  using VelocityMeasurement = KalmanExamples::SportSole::VelocityMeasurement<T>;
  using VelocityModel = KalmanExamples::SportSole::VelocityMeasurementModel<T>;

  using ExtendedKalmanFilter = KalmanExamples::SportSole::ExtendedErrorStateKalmanFilter<T>;

  struct KalmanFilterParams 
  {
    // Print the value of all parameters
    void print();

    // Parameters
    #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
      ROS_PARAM_LIST
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
    const State& update(const PositionMeasurement& zp);
    const State& update(const OrientationMeasurement& zth);
    const State& update(const VelocityMeasurement& zv);


    static SystemModel sys;
    static PositionModel pm;
    static OrientationModel om;
    static VelocityModel vm;
  };
}

class KalmanFilterNode
{
public:
  KalmanFilterNode(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));

  void sportSoleCB(const sport_sole::SportSole& msg);
  void skeletonsCB(const visualization_msgs::MarkerArray& msg);

private:
  enum left_right_t {LEFT = 0, RIGHT, LEFT_RIGHT};

  sport_sole::KalmanFilterParams params_;

  // ROS Node variables
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  
  message_filters::Subscriber<sport_sole::SportSole> sub_sport_sole_;
  message_filters::Cache<sport_sole::SportSole> cache_sport_sole_;
  ros::Subscriber sub_skeletons_;

  ros::Publisher pub_pose_estimates_[LEFT_RIGHT];
  ros::Publisher pub_twist_estimates_[LEFT_RIGHT];
  ros::Publisher pub_pose_measurements_[LEFT_RIGHT];

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform tf_depth_to_map_;

  tf2_ros::TransformBroadcaster tf_boradcaster_;

  sport_sole::KalmanFilter ekf_[LEFT_RIGHT];
  sport_sole::KalmanFilter predictor_[LEFT_RIGHT];
  ros::Time stamp_skeleton_prev_;
  ros::Time stamp_sport_sole_prev_[LEFT_RIGHT];
  ros::Time stamp_base_;

  // Acceleration bias
  Kalman::Vector<T, 3> acc_bias_[LEFT_RIGHT];
  Average<Kalman::Vector<T, 3>> acc_bias_ave_[LEFT_RIGHT];

  // Data logging
  std::ofstream ofs_process_[LEFT_RIGHT];
  std::ofstream ofs_measurement_[LEFT_RIGHT];
  static Eigen::IOFormat csv_format_;

  // Subject selection
  int32_t sub_id_;

};

// Define static member variable
Eigen::IOFormat KalmanFilterNode::csv_format_(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "");


#endif