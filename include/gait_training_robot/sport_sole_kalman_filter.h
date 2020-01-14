#ifndef SPORT_SOLE_KALMAN_FILTER_H
#define SPORT_SOLE_KALMAN_FILTER_H

#define _USE_MATH_DEFINES
#include <cmath>

#include <array>
#include <fstream>
#include <ctime>
#include <iomanip>

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

// Some type shortcuts
typedef KalmanExamples::SportSole::State<T> State;
typedef KalmanExamples::SportSole::Control<T> Control;
typedef KalmanExamples::SportSole::SystemModel<T> SystemModel;

typedef KalmanExamples::SportSole::PositionMeasurement<T> PositionMeasurement;
typedef KalmanExamples::SportSole::PositionMeasurementModel<T> PositionModel;

typedef KalmanExamples::SportSole::OrientationMeasurement<T> OrientationMeasurement;
typedef KalmanExamples::SportSole::OrientationMeasurementModel<T> OrientationModel;

typedef KalmanExamples::SportSole::VelocityMeasurement<T> VelocityMeasurement;
typedef KalmanExamples::SportSole::VelocityMeasurementModel<T> VelocityModel;

typedef KalmanExamples::SportSole::ExtendedErrorStateKalmanFilter<T> ExtendedKalmanFilter;

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
#define ROS_PARAM_LIST                                                                                                      \
    LIST_ENTRY(sampling_period, "The sampling period that is used by the system equation for prediction.", float, 0.01f)    \
    LIST_ENTRY(system_noise_p, "The standard deviation of noise added to the linear position state.", float, 0.0f)          \
    LIST_ENTRY(system_noise_v, "The standard deviation of noise added to the linear velocity state.", float, 0.001f)        \
    LIST_ENTRY(system_noise_th, "The standard deviation of noise added to the angular position state.", float, 0.0f)        \
    LIST_ENTRY(system_noise_ba, "The standard deviation of noise added to the acc bias state.", float, 0.01f)        \
    LIST_ENTRY(measurement_noise_p, "The standard deviation of position measurement noise.", float, 0.02f)                  \
    LIST_ENTRY(measurement_noise_v, "The standard deviation of velocity measurement noise.", float, 0.01f)               \
    LIST_ENTRY(measurement_noise_th, "The standard deviation of orientation measurement noise.", float, 0.2f)               \
    LIST_ENTRY(global_frame, "The reference frame for the filter output.", std::string, std::string("odom"))               \




struct SportSoleKalmanFilterParams 
{
    // Print the value of all parameters
    void print();

    // Parameters
    #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
        ROS_PARAM_LIST
    #undef LIST_ENTRY
};


struct SportSoleEKF: public ExtendedKalmanFilter
{
    SportSoleEKF();
    
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

/**
 * @brief Function template for setting the covariance matrix of different models
 * 
 * @param StateType The vector-type of the system state (usually some type derived from Kalman::Vector)
 * @param model Various models derived from base Kalman::StandardBase<StateType> such as: 
 *        SystemModel, PositionModel, and OrientationModel.
 * @param var_vec The elements on the diagonal of the covariance matrix.
 */
template<class StateType>
void setModelCovariance(Kalman::StandardBase<StateType>& model, 
    const std::array<T, StateType::RowsAtCompileTime>& var_vec)
{
    auto cov = decltype(model.getCovariance()){};
    cov.setZero();
    for (size_t i = 0; i < StateType::RowsAtCompileTime; i++)
        cov(i, i) = var_vec[i];
    model.setCovariance(cov);
}

template<class VectorType>
struct Average {
    VectorType sum;
    size_t num;
    Average() {sum.setZero(); num = 0;}
    void clear() {sum.setZero(); num = 0;}
    void put(const VectorType & v) { sum += v; num++;}
    VectorType getAverage() {return sum / num;}
};

class SportSoleKalmanFilterNode
{
public:
    SportSoleKalmanFilterNode(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));

    void sportSoleCB(const sport_sole::SportSole& msg);
    void skeletonsCB(const visualization_msgs::MarkerArray& msg);

protected:
    visualization_msgs::MarkerPtr constructMarkerMsg(const State& x, const Eigen::Quaternion<T>& q_imu, const ros::Time& stamp);

private:
    enum left_right_t {LEFT = 0, RIGHT, LEFT_RIGHT};

    SportSoleKalmanFilterParams params_;

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

    SportSoleEKF ekf_[LEFT_RIGHT];
    SportSoleEKF predictor_[LEFT_RIGHT];
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
Eigen::IOFormat SportSoleKalmanFilterNode::csv_format_(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "");

std::string generateDateString()
{
    std::time_t t = std::time(nullptr);
    auto timeinfo = std::localtime (&t);

    std::ostringstream oss;

    oss << std::put_time(timeinfo, "%F-%H-%M-%S");
    return oss.str();
}

template<class VectorType, typename T>
inline void assignVector3(VectorType & v1, Eigen::Matrix<T, 3, 1> v2)
{
    v1.x = v2(0);
    v1.y = v2(1);
    v1.z = v2(2);
}

template<class QuaternionType, typename T>
inline void assignQuaternion(QuaternionType & q1, Eigen::Quaternion<T> q2)
{
    q1.x = q2.x();
    q1.y = q2.y();
    q1.z = q2.z();
    q1.w = q2.w();
}

// Convert quaternion from that in geometry_msgs to that in Eigen
template<typename T>
inline Eigen::Quaternion<T> toEigenQuaternion(const geometry_msgs::Quaternion& q)
{
    return Eigen::Quaternion<T>(q.w, q.x, q.y, q.z);
}

template<typename T>
inline Eigen::Quaternion<T> getImuQuaternion(geometry_msgs::Quaternion q)
{
    return toEigenQuaternion<T>(q);
}

#endif