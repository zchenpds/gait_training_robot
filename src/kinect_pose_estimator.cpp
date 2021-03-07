#include "gait_training_robot/kinect_pose_estimator.h"
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
    sub_kimu_(nh_, "/imu", 100),
    cache_kimu_(sub_kimu_, 800),
    sub_odom_(nh_, "/odom", 3),
    cache_odom_(sub_odom_, 6)
{ 
  // Collect ROS parameters from the param server or from the command line
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
    private_nh_.param(#param_variable, params_.param_variable, param_default_val);

    PARAM_LIST
  #undef LIST_ENTRY
  params_.print();

  cache_kimu_.registerCallback(&KinectPoseEstimator::kimuCB, this);
  cache_odom_.registerCallback(&KinectPoseEstimator::odomCB, this);
  
  pub_filtered_odom_ = private_nh_.advertise<nav_msgs::Odometry>("odom", 20);
  
  // Initialize tf message
  pose_estimate_.header.frame_id = "base_link";
  pose_estimate_.child_frame_id = params_.output_frame;

  // Initialize filtered odom message
  odom_filtered_.header.frame_id = params_.output_frame;
  odom_filtered_.child_frame_id = "base_link";
}

KinectPoseEstimator::~KinectPoseEstimator()
{
}


void KinectPoseEstimator::odomCB(const nav_msgs::Odometry & msg)
{
  if (ts_odom_last_.isZero() && !ts_kimu_last_.isZero())
  {
    // State Initialization
    state_.t = ts_kimu_last_;
    state_.x = msg.pose.pose.position.x;
    state_.y = msg.pose.pose.position.y;
    state_.th = tf2::getYaw(msg.pose.pose.orientation);
    state_.vx = msg.twist.twist.linear.x;
    state_.bax = 0.0;
    state_.bwz = 0.0;
    pose_estimate_.transform.rotation = msg.pose.pose.orientation;
    state_.ready = true;
    ts_init_ = ts_odom_last_ = msg.header.stamp;
    state_posterior_ = state_;
  }
}


void KinectPoseEstimator::kimuCB(const sensor_msgs::Imu & msg)
{
  // Proceed only if the state is initialized with an odom message.
  if (ts_odom_last_.isZero()) 
  {
    ts_kimu_last_ = msg.header.stamp;
    return;
  }

  static int num_of_predictions_between_updates;
  // Predict only 0.1 sec into the future from the previuos measurement
  if (msg.header.stamp < ts_odom_last_ + ros::Duration(0.098))
  {
    auto msg_ptrs = cache_kimu_.getInterval(ts_kimu_last_, msg.header.stamp);
    
    num_of_predictions_between_updates = 0;
    for (auto msg_ptr : msg_ptrs)
    {
      if (msg_ptr->header.stamp == ts_kimu_last_) continue;

      ++num_of_predictions_between_updates;
      double dt = (msg_ptr->header.stamp - ts_kimu_last_).toSec();
      // ROS_INFO_STREAM("Predicting state: " << state_.x << " " << state_.y << " " << state_.th << " for " << dt);
      state_.predict(msg_ptr->linear_acceleration.x, -msg_ptr->angular_velocity.z, dt);
      broadcastTf();
      // ROS_INFO_STREAM("Predicted state: " << state_.x << " " << state_.y << " " << state_.th << " at " << state_.t.toSec());
      ts_kimu_last_ = msg_ptr->header.stamp;
    }

  }
  else
  {
    // Update:
    auto msg_odom_ptr = cache_odom_.getElemAfterTime(ts_odom_last_);
    if (!msg_odom_ptr) return;
    
    double z_x = msg_odom_ptr->pose.pose.position.x;
    double z_y = msg_odom_ptr->pose.pose.position.y;
    double z_vx = msg_odom_ptr->twist.twist.linear.x;
    double z_th = tf2::getYaw(msg_odom_ptr->pose.pose.orientation);

    auto msg_odom_ptr_prev = cache_odom_.getElemAfterTime(ts_odom_last_ - ros::Duration(0.01));
    if (msg_odom_ptr_prev)
    {
      double z_dt = (msg_odom_ptr->header.stamp - msg_odom_ptr_prev->header.stamp).toSec();
      double z_vx_1 = msg_odom_ptr_prev->twist.twist.linear.x;
      double z_th_1 = tf2::getYaw(msg_odom_ptr_prev->pose.pose.orientation);

      const double K_bax = 0.1;
      const double K_bwz = 0.05;

      state_.bax += K_bax *
          ( (state_.vx - state_posterior_.vx) / (state_.t - state_posterior_.t).toSec() - 
            (z_vx - z_vx_1) / z_dt
          );
      state_.bwz += K_bwz *
          ( wrapToPi(state_.th - state_posterior_.th) / (state_.t - state_posterior_.t).toSec() -
            wrapToPi(z_th - z_th_1) / z_dt
          );
    }
    else
    {
      ROS_WARN("Impossible! msg_odom_ptr_prev == nullptr! This shouldn't happen!");
    }

    const double K = 0.1;
    state_.vx = state_.vx + K * (z_vx - state_.vx);
  
    state_.x = state_.x + K * (z_x - state_.x);
    state_.y = state_.y + K * (z_y - state_.y);
    state_.th = state_.th + 0.02 * wrapToPi(z_th - state_.th);
    // state_.th = wrapToPi(state_.th);

    // Debug output
    static bool once = [](){
      printf("|   ts   |  bax   |   vx   |  bwz   |   th   |  cnt   |\n");
      printf("|--------+--------+--------+--------+--------+--------|\n");
      return true;
    }();
    printf("|%8.4lf|%8.4lf|%8.4lf|%8.4lf|%8.4lf|%8d|\r", 
      (state_.t - ts_init_).toSec(), state_.bax, state_.vx, state_.bwz, state_.th, num_of_predictions_between_updates);
    fflush(stdout);

    // Update for next iteration
    ts_odom_last_ = msg_odom_ptr->header.stamp;
    state_posterior_ = state_;
    state_.ready = true;
  }


}

void KinectPoseEstimator::broadcastTf()
{

  if (state_.t < ts_next_desired_publish_) return;

  pose_estimate_.header.seq++;
  pose_estimate_.header.stamp = state_.t;
  pose_estimate_.transform.translation.x = state_.x;
  pose_estimate_.transform.translation.y = state_.y;
  tf2::Vector3 translation(state_.x, state_.y, 0);
  tf2::Quaternion quat_yaw;
  quat_yaw.setRPY(0.0, 0.0, -state_.th);
  translation = tf2::quatRotate(quat_yaw, translation);
  tf2::convert(quat_yaw, pose_estimate_.transform.rotation);
  tf2::convert(translation, pose_estimate_.transform.translation);
  // ROS_INFO_STREAM("Sending tf: " << state_.x << " " << state_.y << " " << state_.th << " at " << state_.t.toSec());
  tf_broadcaster_.sendTransform(pose_estimate_);

  state_.ready = false;
  if (state_.t < ts_next_desired_publish_ + desired_publish_period_)
  {
    ts_next_desired_publish_ += desired_publish_period_;
  }
  else
  {
    ts_next_desired_publish_ = state_.t + desired_publish_period_;
  }

  // Debug output
  odom_filtered_.header.seq++;
  odom_filtered_.header.stamp = state_.t;
  odom_filtered_.pose.pose.position.x = state_.x;
  odom_filtered_.pose.pose.position.y = state_.y;
  odom_filtered_.twist.twist.linear.x = state_.vx;
  odom_filtered_.pose.pose.orientation = pose_estimate_.transform.rotation;
  pub_filtered_odom_.publish(odom_filtered_);


}

void KinectPoseEstimator::State::predict(double ax, double wz, double dt)
{
  vx += (ax - bax) * dt;
  x += vx * cos(th) * dt;
  y += vx * sin(th) * dt;
  th += (wz - bwz) * dt;
  // th = wrapToPi(th);
  t += ros::Duration(dt);
  ready = true;
}

KinectPoseEstimator::State& KinectPoseEstimator::State::operator=(const KinectPoseEstimator::State& rhs)
{
  ready = rhs.ready;
  t = rhs.t;
  x = rhs.x;
  y = rhs.y;
  th = rhs.th;
  vx = rhs.vx;
  bax = rhs.bax;
  bwz = rhs.bwz;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_pose_estimator");
  KinectPoseEstimator kpe;

  ros::spin();
  std::cout << std::endl;
  return 0;
}