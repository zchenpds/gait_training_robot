#include "gait_training_robot/kinect_pose_estimator.h"
#include "gait_training_robot/KpekfState.h"
#include "gait_training_robot/KpekfKimuMeasurement.h"
#include "gait_training_robot/KpekfOdomMeasurement.h"
#include <iomanip>


void KinectPoseEstimatorParams::print()
{
  ROS_INFO_STREAM("kinect_pose_estimator Parameters: ");
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)     \
    ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

    PARAM_LIST
  #undef LIST_ENTRY
}

KinectPoseEstimator::KinectPoseEstimator(const ros::NodeHandle& n, const ros::NodeHandle& p) :
    nh_(n),
    private_nh_(p),
    tf_listener_(tf_buffer_),
    sub_kimu_(nh_, "/imu", 200),
    cache_kimu_(sub_kimu_, 800),
    sub_odom_(nh_, "/odom", 30),
    cache_odom_(sub_odom_, 60)
{ 
  // Collect ROS parameters from the param server or from the command line
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
    private_nh_.param(#param_variable, params_.param_variable, param_default_val);

    PARAM_LIST
  #undef LIST_ENTRY
  params_.print();

  // Load parameters
  const auto& var_q  = pow(params_.system_noise_q,      2);
  const auto& var_w  = pow(params_.system_noise_w,      2);
  const auto& var_p  = pow(params_.system_noise_p,      2);
  const auto& var_v  = pow(params_.system_noise_v,      2);
  const auto& var_a  = pow(params_.system_noise_a,      2);
  const auto& var_ab = pow(params_.system_noise_ab,     2);
  const auto& var_wb = pow(params_.system_noise_wb,     2);

  setModelCovariance(ekf_.sys, 
      {var_q, var_q, var_q, var_q, var_w, var_w, var_w,                   // q, w
        var_p, var_p, var_p, var_v, var_v, var_v, var_a, var_a, var_a, // p, v, a
        var_ab, var_ab, var_ab, var_wb, var_wb, var_wb});                    // ab wb
  // auto var_zp = pow(params_.measurement_noise_p,  2);
  setModelCovariance(ekf_.pmm,  pow(params_.measurement_noise_p,  2));
  setModelCovariance(ekf_.vmm,  pow(params_.measurement_noise_v,  2));
  setModelCovariance(ekf_.amm,  pow(params_.measurement_noise_a,  2));
  setModelCovariance(ekf_.gmm,  pow(params_.measurement_noise_g,  2));
  setModelCovariance(ekf_.qmm,  pow(params_.measurement_noise_q,  2));
  setModelCovariance(ekf_.ymm,  pow(params_.measurement_noise_y,  2));
  setModelCovariance(ekf_.vamm, pow(params_.measurement_noise_va, 2));

  cache_kimu_.registerCallback(&KinectPoseEstimator::kimuCB, this);
  cache_odom_.registerCallback(&KinectPoseEstimator::odomCB, this);
  
  pub_filtered_odom_ = private_nh_.advertise<nav_msgs::Odometry>("odom", 20);
  pub_ekf_state_ = private_nh_.advertise<gait_training_robot::KpekfState>("ekf/state", 200);
  pub_ekf_odom_measurement_ = private_nh_.advertise<gait_training_robot::KpekfOdomMeasurement>("ekf/odom_measurement", 20);
  pub_ekf_kimu_measurement_ = private_nh_.advertise<gait_training_robot::KpekfKimuMeasurement>("ekf/kimu_measurement", 200);
}

KinectPoseEstimator::~KinectPoseEstimator()
{
}


void KinectPoseEstimator::odomCB(const nav_msgs::Odometry & msg)
{
  ts_odom_last_ = msg.header.stamp;
  if (!kimu_received_) return;
  if (!ekf_initialized_) odomUpdate(msg.header.stamp); // Initialize
  kimuPredictAndUpdate(msg.header.stamp);
  
}


void KinectPoseEstimator::kimuCB(const sensor_msgs::Imu & msg)
{
  kimu_received_ = true;
  if (ekf_initialized_ && msg.header.stamp <= ts_odom_last_)
  {
    kimuPredictAndUpdate(ts_odom_last_);
  }
}

void KinectPoseEstimator::kimuPredictAndUpdate(const ros::Time& stamp)
{
  auto msg_ptrs = cache_kimu_.getInterval(ts_kimu_last_, stamp);
  
  for (auto msg_ptr : msg_ptrs)
  {
    double dt = (msg_ptr->header.stamp - ts_kimu_last_).toSec();
    if (dt < 1e-4) continue;
    
    ekf_t::ZA za;
    za << msg_ptr->linear_acceleration.x, msg_ptr->linear_acceleration.y, msg_ptr->linear_acceleration.z;
    averager_za_.put(za);
    ekf_t::ZG zg;
    zg << msg_ptr->angular_velocity.x, msg_ptr->angular_velocity.y, msg_ptr->angular_velocity.z;
    averager_zg_.put(zg);
    ts_kimu_last_ = msg_ptr->header.stamp;

    // Downsample
    ++num_of_kimu_messages_;
    if (num_of_kimu_messages_ % 8) continue;

    // Predict
    ++num_of_predictions_between_odom_updates_;
    ekf_.predict((ts_kimu_last_ - ts_predict_last_).toSec());
    ekf_.update(averager_za_.getAverage());
    ekf_.update(averager_zg_.getAverage());
    odomUpdate(ts_kimu_last_);
    averager_za_.clear();
    averager_zg_.clear();
    ts_ekf_ = ts_predict_last_ = msg_ptr->header.stamp;

    if (pub_ekf_kimu_measurement_.getNumSubscribers())
    {
      gait_training_robot::KpekfKimuMeasurement msg_kimu_measurement;
      msg_kimu_measurement.header.stamp = ts_ekf_;
      msg_kimu_measurement.header.frame_id = "imu_link";
      msg_kimu_measurement.a = tf2::toMsg(tf2::Vector3(za.x(), za.y(), za.z()));
      msg_kimu_measurement.w = tf2::toMsg(tf2::Vector3(zg.x(), zg.y(), zg.z()));
      pub_ekf_kimu_measurement_.publish(msg_kimu_measurement);
    }
  }
}

void KinectPoseEstimator::odomUpdate(const ros::Time& stamp)
{
  // Find the transform from kimu to global_frame
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = tf_buffer_.lookupTransform(params_.global_frame, params_.kimu_frame, stamp, ros::Duration(0.05));
    tf2::fromMsg(tf_msg.transform, tf_global_to_kimu_);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Assign measurement
  tf2::Vector3 trans = tf_global_to_kimu_.getOrigin();
  tf2::Quaternion rot = tf_global_to_kimu_.getRotation();

  // Initialize
  if (!ekf_initialized_)
  {
    ROS_INFO("INITIALIZING KPEKF.");
    ekf_.x.px() = trans.x();
    ekf_.x.py() = trans.y();
    ekf_.x.pz() = trans.z();
    ekf_.x.q0() = rot.w();
    ekf_.x.q1() = rot.x();
    ekf_.x.q2() = rot.y();
    ekf_.x.q3() = rot.z();
    ekf_.repairQuaternion();
    ekf_initialized_ = true;
    ts_init_ = ts_kimu_last_ = ts_predict_last_ = stamp;
  }
  else
  {
    // Update position
    ekf_t::ZP zp;
    zp << trans.x(), trans.y(), trans.z();
    ekf_.update(zp);

    // Update yaw
    tf2::Vector3 zy_ros = tf2::quatRotate(rot.inverse(), refUnitY_);
    ekf_t::ZY zy;
    zy << zy_ros.x(), zy_ros.y(), zy_ros.z();
    ekf_.update(zy);

    // Publish odom measurement
    gait_training_robot::KpekfOdomMeasurement msg_odom_measurement;
    msg_odom_measurement.header.stamp = ts_ekf_;
    msg_odom_measurement.header.frame_id = params_.global_frame;
    msg_odom_measurement.p = tf2::toMsg(trans);
    msg_odom_measurement.refUnitY = tf2::toMsg(refUnitY_);
    // msg_odom_measurement.y = tf2::toMsg(zy_ros);
    pub_ekf_odom_measurement_.publish(msg_odom_measurement);
  }

  // Publish messages
  broadcastTf();

  // Update for next iteration
  ts_ekf_ = ts_odom_update_last_ = ts_odom_last_;
  // std::cout << " " << num_of_predictions_between_odom_updates_;
  num_of_predictions_between_odom_updates_ = 0;
}

void KinectPoseEstimator::broadcastTf()
{
  // Publish tf
  pose_estimate_.header.frame_id = params_.kimu_frame;
  pose_estimate_.child_frame_id = params_.output_frame;
  pose_estimate_.header.stamp = ts_ekf_;
  tf2::Vector3 trans(ekf_.x.px(), ekf_.x.py(), ekf_.x.pz());
  tf2::Quaternion rot(ekf_.x.q1(), ekf_.x.q2(), ekf_.x.q3(), ekf_.x.q0());
  tf2::Transform tf_estimated(rot, trans);
  pose_estimate_.transform = tf2::toMsg(tf_estimated.inverse());
  tf_broadcaster_.sendTransform(pose_estimate_);

  // Debug output
  odom_filtered_.header.frame_id = params_.output_frame;
  odom_filtered_.child_frame_id = params_.kimu_frame;
  odom_filtered_.header.stamp = ts_ekf_;
  tf2::toMsg(trans, odom_filtered_.pose.pose.position);
  odom_filtered_.pose.pose.orientation = tf2::toMsg(rot);
  tf2::Vector3 vel(ekf_.x.vx(), ekf_.x.vy(), ekf_.x.vz());
  odom_filtered_.twist.twist.linear = tf2::toMsg(tf2::quatRotate(rot, vel));
  pub_filtered_odom_.publish(odom_filtered_);

  gait_training_robot::KpekfState msg_state;
  msg_state.header.stamp = ts_ekf_;
  msg_state.header.frame_id = params_.global_frame;
  msg_state.p  = tf2::toMsg(trans);;
  msg_state.v  = tf2::toMsg(vel);
  msg_state.a  = tf2::toMsg(tf2::Vector3(ekf_.x.ax(), ekf_.x.ay(), ekf_.x.az()));
  msg_state.q.x = ekf_.x.q1();
  msg_state.q.y = ekf_.x.q2();
  msg_state.q.z = ekf_.x.q3();
  msg_state.q.w = ekf_.x.q0();
  tf2::Matrix3x3(rot * tf2::Quaternion({1.0, 0.0, 0.0}, M_PI)).getEulerYPR(msg_state.rpy.x, msg_state.rpy.y, msg_state.rpy.z);
  msg_state.w  = tf2::toMsg(tf2::Vector3(ekf_.x.wx(), ekf_.x.wy(), ekf_.x.wz()));
  msg_state.wb = tf2::toMsg(tf2::Vector3(ekf_.x.wbx(), ekf_.x.wby(), ekf_.x.wbz()));
  msg_state.ab = tf2::toMsg(tf2::Vector3(ekf_.x.abx(), ekf_.x.aby(), ekf_.x.abz()));
  pub_ekf_state_.publish(msg_state);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_pose_estimator");
  KinectPoseEstimator kpe;

  ros::spin();
  std::cout << std::endl;
  return 0;
}