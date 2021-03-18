//Purpose: bring together data from  1) accel, quaternion, pressures of left & right sport soles  2) kinect IMU  3) kinect skeleton info

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "gait_training_robot/gait_analyzer.h"
#include "gait_training_robot/ComkfState.h"
#include "gait_training_robot/ComkfCopMeasurement.h"
#include "gait_training_robot/ComkfComMeasurement.h"

using namespace visualization_msgs;

void comkf::KalmanFilterParams::print()
{
  ROS_INFO_STREAM("COMKF Parameters: ");
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)     \
    ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

    COMKF_PARAM_LIST
  #undef LIST_ENTRY
}


void GaitAnalyzerParams::print()
{
  ROS_INFO_STREAM("GaitAnalyzer Parameters: ");
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)         \
    ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

    GA_PARAM_LIST
  #undef LIST_ENTRY
}

cop_t sport_sole::getCoP(
  const SportSole::_pressures_type & pressures, 
  const vec_refpoints_t & vec_refpoints, 
  const vec_refvecs_t & vec_refvecs)
{
  std::array<cop_t, NUM_2XPSENSOR> psensor_locs = {};
  cop_t cop(.0, .0, .0);
  const std::array<float, NUM_PSENSOR> weights{1.0, 1.0, 1.0, 1.0, 1.0, 2.5, 2.5, 1.0};
  double pressure_sum = 0.0;
  for (size_t i = 0; i < NUM_2XPSENSOR; i++) {
    left_right_t lr = (i / NUM_PSENSOR == 0 ? LEFT : RIGHT);
    double th = atan2(vec_refvecs[lr].getY(), vec_refvecs[lr].getX()) + Theta[i];
    psensor_locs[i].setX(vec_refpoints[HIND][lr].getX() + Rho[i] * cos(th));
    psensor_locs[i].setY(vec_refpoints[HIND][lr].getY() + Rho[i] * sin(th));
    psensor_locs[i].setZ(0.0);

    // Avoid dividing by zero
    const double min_pressure = 0.1;
    auto pressure = std::max(pressures[i] * weights[i % NUM_PSENSOR], min_pressure);
    
    pressure_sum += pressure;
    cop += pressure * psensor_locs[i];
  }
  cop /= pressure_sum;
  return cop;
}

GaitAnalyzer::GaitAnalyzer(const ros::NodeHandle& n, const ros::NodeHandle& p):
  com_model_(COM_MODEL_PELVIS), // COM_MODEL_PELVIS, COM_MODEL_14_SEGMENT
  touches_ground_{
    {true, true}, // Left ankle, right ankle
    {true, true} // Left foot, right foot
    },
  ml_vec_(0, 0, 0),
  ap_vec_(0, 0, 0),
  z_ground_(0.0),
  nh_(n),
  private_nh_(p),
  comkf_nh_(private_nh_, "comkf"),
  sub_sport_sole_(nh_.subscribe("/sport_sole_publisher/sport_sole", 20, &GaitAnalyzer::sportSoleCB, this)),
  cache_sport_sole_(100),
  time_synchronizer_measure_(100),
  time_synchronizer_estimate_(600),
  tf_listener_(tf_buffer_),
  is_initialized_tf_global_to_publish_(false),
  comkf_initialized_(false),
  sub_id_(-1)
{
  // Collect ROS parameters from the param server or from the command line
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  private_nh_.param(#param_variable, ga_params_.param_variable, param_default_val);

  GA_PARAM_LIST
#undef LIST_ENTRY
  ga_params_.print();

  // Collect ROS parameters from the param server or from the command line
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  comkf_nh_.param(#param_variable, comkf_params_.param_variable, param_default_val);

  COMKF_PARAM_LIST
#undef LIST_ENTRY
  comkf_params_.print();

  // Set the parameters for comkf
  T var_p = pow(comkf_params_.system_noise_p, 2);
  T var_v = pow(comkf_params_.system_noise_v, 2);
  T var_b = pow(comkf_params_.system_noise_b, 2);
  T var_cop = pow(comkf_params_.system_noise_cop, 2);
  setModelCovariance(comkf_.sys, {var_p, var_p, var_v, var_v, var_b, var_b});
  T var_zp = pow(comkf_params_.measurement_noise_p, 2);
  // T var_zv = pow(comkf_params_.measurement_noise_v, 2);
  setModelCovariance(comkf_.pmm, {var_zp, var_zp});

  if (ga_params_.data_source == "k4a")
  {
    sub_skeletons_ = nh_.subscribe("/body_tracking_data", 5, &GaitAnalyzer::skeletonsCB, this );
  }
  else if (ga_params_.data_source == "optitrack")
  {
    // namespace optitrack
    ga_params_.foot_pose_topic = "/optitrack/foot_pose_";
    ga_params_.global_frame = "optitrack";
    sub_com_ = private_nh_.subscribe("/optitrack/com", 50, &GaitAnalyzer::comCB, this);
    sub_ml_vec_ = private_nh_.subscribe("/optitrack/ml_vec", 50, &GaitAnalyzer::mlVecCB, this);
  }
  else
  {
    ROS_ERROR("Unknown data_source");
  }

  timer_update_tf_global_to_publish_ = nh_.createTimer(ros::Duration(0.1), &GaitAnalyzer::updateTfCB, this);


  for (auto lr: {LEFT, RIGHT})
  {
    sub_foot_poses_[lr].subscribe(nh_, ga_params_.foot_pose_topic + (lr == LEFT ? "l" : "r"), 20);
  }
  time_synchronizer_measure_.connectInput(sub_foot_poses_[LEFT], sub_foot_poses_[RIGHT]);
  time_synchronizer_measure_.registerCallback(boost::bind(&GaitAnalyzer::timeSynchronizerMeasureCB, this, _1, _2, _3, _4));
  time_synchronizer_estimate_.connectInput(sub_foot_poses_[LEFT], sub_foot_poses_[RIGHT]);
  time_synchronizer_estimate_.registerCallback(boost::bind(&GaitAnalyzer::timeSynchronizerEstimateCB, this, _1, _2, _3, _4));
  
  const size_t buff_size = 100;
  for (size_t lr: {LEFT, RIGHT}) {
    auto str_lr = std::string(!lr?"_l": "_r");
    pub_footprints_[lr] = private_nh_.advertise<geometry_msgs::PolygonStamped>("footprint" + str_lr, buff_size);
  }

  pub_gait_state_ = private_nh_.advertise<sport_sole::GaitState>("gait_state", buff_size);
  for (auto me: {MEASUREMENT, ESTIMATE})
  {
    auto str_me = std::string( !me ? "measurement/": "estimate/");
    pub_com_[me]  = private_nh_.advertise<geometry_msgs::PointStamped>        (str_me + "com",  buff_size);
    pub_comv_[me] = private_nh_.advertise<geometry_msgs::Vector3Stamped>      (str_me + "comv", buff_size);
    pub_xcom_[me] = private_nh_.advertise<geometry_msgs::PointStamped>        (str_me + "xcom", buff_size);
    pub_mos_[me] = private_nh_.advertise<visualization_msgs::MarkerArray>     (str_me + "mos",  buff_size);
    pub_mos_vector_[me] = private_nh_.advertise<geometry_msgs::Vector3Stamped>(str_me + "mos_vec", buff_size);
  }

  for (auto lr: {LEFT, RIGHT})
  {
    auto str_lr = std::string(!lr?"_l": "_r");
    pub_hindfoot_refpoint_[lr] = private_nh_.advertise<geometry_msgs::PointStamped>("ankle_pose_measurement" + str_lr, buff_size);
    pub_forefoot_refpoint_[lr] = private_nh_.advertise<geometry_msgs::PointStamped>("foot_pose_measurement" + str_lr, buff_size);
  }

  pub_cop_ = private_nh_.advertise<geometry_msgs::PointStamped>  ("cop", buff_size);
  pub_bos_ = private_nh_.advertise<geometry_msgs::PolygonStamped>("bos", buff_size);
  
  pub_ground_clearance_left_ = private_nh_.advertise<std_msgs::Float64> ("ground_clearance_left", buff_size);
  pub_ground_clearance_right_ = private_nh_.advertise<std_msgs::Float64>("ground_clearance_right", buff_size);

  pub_comkf_state_           = comkf_nh_.advertise<gait_training_robot::ComkfState>         ("state", buff_size);
  pub_comkf_cop_measurement_ = comkf_nh_.advertise<gait_training_robot::ComkfCopMeasurement>("zcop",  buff_size);
  pub_comkf_com_measurement_ = comkf_nh_.advertise<gait_training_robot::ComkfComMeasurement>("zcom",  buff_size);
}

void GaitAnalyzer::skeletonsCB(const visualization_msgs::MarkerArray& msg)
{  
  double dist_min_pelvis = 100.0;
  auto it_pelvis_closest = msg.markers.end();// iterator of the pelvis marker of the closest body

  // Find the closest body, K4ABT_JOINT_PELVIS = 0
  if (sub_id_ < 0) {
    for (auto it = msg.markers.begin(); it < msg.markers.end(); it += K4ABT_JOINT_COUNT) {
      // The coordinates are in expressed in /depth_camera_link
      double dist_pelvis = hypot(it->pose.position.x, it->pose.position.z);
      if (dist_pelvis < dist_min_pelvis) {
        dist_min_pelvis = dist_pelvis;
        it_pelvis_closest = it;
        sub_id_ = it_pelvis_closest->id / 100;
      }
    }
  }
  else {
    for (auto it = msg.markers.begin(); it < msg.markers.end(); it += K4ABT_JOINT_COUNT) {
      if (it->id / 100 == sub_id_) {
        it_pelvis_closest = it;
        break;
      }
    }
  }

  // Process the closest body
  if (it_pelvis_closest != msg.markers.end())
  {
    // Get the current timestamp of the skeleton message
    const auto & stamp_skeleton_curr = it_pelvis_closest->header.stamp;

    try
    {
      // Find the tf from global frame to depth_camera_link frame
      // assert(msg.markers[0].header.frame_id == "depth_camera_link");
      geometry_msgs::TransformStamped tf_msg;
      tf_msg = tf_buffer_.lookupTransform("camera_mount_top", "imu_link", stamp_skeleton_curr, ros::Duration(0.2));
      // tf_msg = tf_buffer_.lookupTransform("depth_camera_link", "imu_link", stamp_skeleton_curr, ros::Duration(0.2));
      fromMsg(tf_msg.transform, tf_imu_to_local_);
      tf_msg = tf_buffer_.lookupTransform(ga_params_.global_frame, "depth_camera_link", stamp_skeleton_curr, ros::Duration(0.2));
      fromMsg(tf_msg.transform, tf_depth_to_global_);
    }
    catch (tf2::TransformException & ex)
    {
      ROS_WARN_THROTTLE(5.0, "Cannot transform CoM from 'depth_camera_link' to '%s'. %s", ga_params_.global_frame.c_str(), ex.what());
      return;
      //ros::Duration(1.0).sleep();
    }

    // Broadcast some tfs.
    for (auto joint_id: {K4ABT_JOINT_PELVIS, K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_ANKLE_RIGHT})
    {
      const auto & it_joint = it_pelvis_closest + joint_id;

      geometry_msgs::TransformStamped tf_joint;
      tf_joint.header.stamp = stamp_skeleton_curr;
      tf_joint.header.frame_id = "depth_camera_link"; // params_.global_frame, "depth_camera_link"
      switch (joint_id)
      {
        case K4ABT_JOINT_PELVIS: 
          tf_joint.child_frame_id = "joint_pelvis"; break;
        case K4ABT_JOINT_ANKLE_LEFT: 
          tf_joint.child_frame_id = "joint_ankle_left"; break;
        case K4ABT_JOINT_ANKLE_RIGHT: 
          tf_joint.child_frame_id = "joint_ankle_right"; break;
        default:
          tf_joint.child_frame_id = std::string("joint_") + std::to_string(joint_id);
      }
      tf_joint.transform.translation.x = it_joint->pose.position.x;
      tf_joint.transform.translation.y = it_joint->pose.position.y;
      tf_joint.transform.translation.z = it_joint->pose.position.z;
      tf_joint.transform.rotation.w = it_joint->pose.orientation.w;
      tf_joint.transform.rotation.x = it_joint->pose.orientation.x;
      tf_joint.transform.rotation.y = it_joint->pose.orientation.y;
      tf_joint.transform.rotation.z = it_joint->pose.orientation.z;
      tf_broadcaster_.sendTransform(tf_joint);
    }
    
    // Publish tf: joint_pelvis_rect
    {
      const auto & it_joint = it_pelvis_closest + K4ABT_JOINT_PELVIS;

      geometry_msgs::TransformStamped tf_joint;
      tf_joint.header.stamp = stamp_skeleton_curr;
      tf_joint.header.frame_id = ga_params_.global_frame;
      tf_joint.child_frame_id = "joint_pelvis_rect";

      const auto& pos_k4a = it_joint->pose.position;
      const auto& quat_k4a = it_joint->pose.orientation;
      tf2::Quaternion quat = 
        tf2::Quaternion(quat_k4a.x, quat_k4a.y, quat_k4a.z, quat_k4a.w) *
        tf2::Quaternion({1., 1., 1.}, M_PI / 3 * 2);
      tf2::Transform tf_pelvis = tf_depth_to_global_ *
        tf2::Transform(quat, {pos_k4a.x, pos_k4a.y, pos_k4a.z});
      quat = tf_pelvis.getRotation();
      double yaw, pitch, roll;
      tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);
      tf_pelvis.setRotation(tf2::Quaternion({0., 0., 1.}, yaw));
      
      tf_joint.transform = tf2::toMsg(tf_pelvis);
      tf_broadcaster_.sendTransform(tf_joint);
    }
    
    // Coordinate Transformation and Projection
    for (int i = 0; i < K4ABT_JOINT_COUNT; i++)
    {
      // Convert messages to tf2::vectors for easier calculation
      fromMsg((it_pelvis_closest + i)->pose.position, vec_joints_[i]);
      vec_joints_[i] = tf_depth_to_global_ * vec_joints_[i];
    }

    // Update AP and ML vectors
    ml_vec_ = ( vec_joints_[K4ABT_JOINT_CLAVICLE_LEFT] - vec_joints_[K4ABT_JOINT_CLAVICLE_RIGHT]
      + vec_joints_[K4ABT_JOINT_HIP_LEFT] - vec_joints_[K4ABT_JOINT_HIP_RIGHT] ).normalize();
    ap_vec_ = ml_vec_.rotate({0, 0, 1}, -M_PI_2);

    // Calculate com (projected center of mass), comv and xcom
    // if (ga_params_.smoother_enabled)  com_smoother_[MEASUREMENT](com_[MEASUREMENT]);
    updateCoMMeasurementFromSkeletons(stamp_skeleton_curr, com_[MEASUREMENT], comv_[MEASUREMENT], vec_joints_);
    // if (ga_params_.smoother_enabled) comv_smoother_[MEASUREMENT](comv_[MEASUREMENT]);
    updateXCoM(xcom_[MEASUREMENT], com_[MEASUREMENT], comv_[MEASUREMENT]);

    time_synchronizer_measure_.add2(constructPointStampedMessage(stamp_skeleton_curr, com_[MEASUREMENT]));
    time_synchronizer_measure_.add3(constructVector3StampedMessage(stamp_skeleton_curr, comv_[MEASUREMENT]));
    
  }
}

void GaitAnalyzer::comCB(const geometry_msgs::PointStamped& msg)
{
  // Calculate com (projected center of mass), comv and xcom
  if (ga_params_.smoother_enabled) com_smoother_[MEASUREMENT](com_[MEASUREMENT]);
  updateCoMMeasurement(msg.header.stamp, com_[MEASUREMENT], comv_[MEASUREMENT], msg);
  if (ga_params_.smoother_enabled) comv_smoother_[MEASUREMENT](comv_[MEASUREMENT]);
  updateXCoM(xcom_[MEASUREMENT], com_[MEASUREMENT], comv_[MEASUREMENT]);

  time_synchronizer_measure_.add2(constructPointStampedMessage(msg.header.stamp, com_[MEASUREMENT]));
  time_synchronizer_measure_.add3(constructVector3StampedMessage(msg.header.stamp, comv_[MEASUREMENT]));
}

void GaitAnalyzer::mlVecCB(const geometry_msgs::Vector3Stamped& msg)
{
  tf2::fromMsg(msg.vector, ml_vec_);
  ap_vec_ = ml_vec_.rotate({0, 0, 1}, -M_PI_2);
}

void GaitAnalyzer::timeSynchronizerMeasureCB(const PoseType::ConstPtr& msg_foot_l, const PoseType::ConstPtr& msg_foot_r, 
    const PointType::ConstPtr& msg_com, const VectorType::ConstPtr& msg_comv)
{
  ros::Time stamp_skeleton_curr = msg_com->header.stamp;
  updateRefPoints(msg_foot_l, msg_foot_r);

  if (ga_params_.data_source == "k4a")
  {
    double z_min = std::min(vec_joints_[K4ABT_JOINT_FOOT_LEFT].getZ(), vec_joints_[K4ABT_JOINT_FOOT_RIGHT].getZ());
    z_ground_ = 0.95 * std::min(z_ground_, z_min) + 0.05 * z_min;
  }
  //z_ground_ = 0.5;

  // Update gait phase
  //updateGaitPhase();

  // If at least one com/skeleton message has been received after the receipt of the first sport_sole message, do an EKF prediction
  if (!stamp_com_prev_.isZero())
  {
    auto sport_sole_ptrs = cache_sport_sole_.getInterval(stamp_com_prev_, stamp_skeleton_curr);
    for (const auto & ss_ptr : sport_sole_ptrs)
    {
      // CoMKF prediction is done here
      processSportSole(*ss_ptr);
    }
    // ROS_INFO_STREAM(sport_sole_ptrs.size() << " predictions between t=" <<
    //   (stamp_com_prev_ - stamp_base_).toSec() << "s and t=" << (stamp_skeleton_curr - stamp_base_).toSec() << "s are done");
  }
  else
  {
    stamp_base_ = stamp_skeleton_curr;
  }


  // Find the sport_sole message sampled immediately before the current skeleton message
  auto msg_sport_sole_ptr = cache_sport_sole_.getElemBeforeTime(stamp_skeleton_curr);

  // At least one sport_sole message should have been received before we can initialize the Kalman filter
  if (!msg_sport_sole_ptr)
  {
    // No msg_sport_sole_ptr has been found in the cache,
    // Check what's happened.
    if (cache_sport_sole_.getOldestTime().isZero())
    {
      ROS_WARN_DELAYED_THROTTLE(5.0, "Skeleton message has been received. \
        But at least one sport_sole message should have been received before we can initialize the Kalman filter.");
    }
    else
    {
      // This should not happen. In case it happens, increasing the cache size should help.
      ROS_WARN_DELAYED_THROTTLE(5.0, "The skeleton message is too old.");
    }
  }
  else
  {
    comkf::ZP zp;
    zp.px() = msg_com->point.x;
    zp.py() = msg_com->point.y;

    // Is this the first skeleton message received after the receipt of the first sport_sole message?
    if (comkf_initialized_ == false) {
      // This is the first time skeleton message received
      // Use the position measurement to initialize the state
      comkf::S x0 = comkf::S::Zero();
      x0.px() = msg_com->point.x;
      x0.py() = msg_com->point.y;
      x0.vx() = msg_comv->vector.x;
      x0.vy() = msg_comv->vector.y;
      comkf_.init(x0);
      updateCoMEstimate(stamp_skeleton_curr, x0); // Initial estimate
      ROS_DEBUG_STREAM("COMKF state initialized to " << x0.transpose());
      comkf_initialized_ = true;
      stamp_predict_prev_ = stamp_sport_sole_prev_ = stamp_skeleton_curr;
    }
    else {
      const auto & x = comkf_.update(zp);
      updateCoMEstimate(stamp_skeleton_curr, x); // A posteriori estimate
      gait_training_robot::ComkfCopMeasurement msg;
      msg.header.stamp = stamp_skeleton_curr;
      msg.header.frame_id = ga_params_.global_frame;
      msg.px = zp.px();
      msg.py = zp.py();
      pub_comkf_com_measurement_.publish(msg);
    }

    // Publish forefoot and hindfoot reference points
    for (size_t lr : {LEFT, RIGHT})
    {
      pub_hindfoot_refpoint_[lr].publish(constructPointStampedMessage(stamp_skeleton_curr, vec_refpoints_[HIND][lr]));
      pub_forefoot_refpoint_[lr].publish(constructPointStampedMessage(stamp_skeleton_curr, vec_refpoints_[FORE][lr]));
    }

    if (comv_[MEASUREMENT].length() < 1e-6) ROS_INFO_STREAM_THROTTLE(5.0, "B: CoMv = " << comv_[MEASUREMENT]);

    // Publish com comv xcom
    pub_com_[MEASUREMENT].publish(constructPointStampedMessage(stamp_skeleton_curr, com_[MEASUREMENT]));
    pub_comv_[MEASUREMENT].publish(constructVector3StampedMessage(stamp_skeleton_curr, comv_[MEASUREMENT]));
    pub_xcom_[MEASUREMENT].publish(constructPointStampedMessage(stamp_skeleton_curr, xcom_[MEASUREMENT]));
  }

  // Calculate the base of support polygon, and update footprint polygons
  updateBoS(bos_points_);
  pub_bos_.publish(constructPolygonMessage(stamp_skeleton_curr, bos_points_));
  for (size_t lr : {LEFT, RIGHT}) {
    pub_footprints_[lr].publish(constructPolygonMessage(stamp_skeleton_curr, footprints_[lr]));
  }

  // Calculate margin of stability measurement
  updateMoS(mos_[MEASUREMENT], xcom_[MEASUREMENT], bos_points_);
  pub_mos_[MEASUREMENT].publish(constructMosMarkerArrayMessage(stamp_skeleton_curr, xcom_[MEASUREMENT], mos_[MEASUREMENT]));
  pub_mos_vector_[MEASUREMENT].publish(constructVector3StampedMessage(stamp_skeleton_curr, constructMosVector(mos_[MEASUREMENT])));

#ifdef PRINT_MOS_DELAY
  std::cout << (ros::Time::now() - stamp_skeleton_curr).toSec() << std::endl;
#endif

  stamp_com_prev_ = stamp_skeleton_curr;
}

void GaitAnalyzer::timeSynchronizerEstimateCB(const PoseType::ConstPtr& msg_foot_l, const PoseType::ConstPtr& msg_foot_r,
    const PointType::ConstPtr& msg_com, const VectorType::ConstPtr& msg_comv)
{
  ros::Time stamp_curr = msg_com->header.stamp;
  updateRefPoints(msg_foot_l, msg_foot_r);
  
  // Find the sport_sole message sampled immediately before the current skeleton message
  auto msg_sport_sole_ptr = cache_sport_sole_.getElemBeforeTime(stamp_curr);
  if (!msg_sport_sole_ptr) return;

  // Calculate the base of support polygon, and update footprint polygons
  updateBoS(bos_points_);
  pub_bos_.publish(constructPolygonMessage(stamp_curr, bos_points_));
  for (size_t lr : {LEFT, RIGHT}) {
    pub_footprints_[lr].publish(constructPolygonMessage(stamp_curr, footprints_[lr]));
  }

  // Calculate margin of stability estimate
  updateMoS(mos_[ESTIMATE], xcom_[ESTIMATE], bos_points_);
  pub_mos_[ESTIMATE].publish(constructMosMarkerArrayMessage(stamp_curr, xcom_[ESTIMATE], mos_[ESTIMATE]));
  pub_mos_vector_[ESTIMATE].publish(constructVector3StampedMessage(stamp_curr, constructMosVector(mos_[ESTIMATE])));
}


void GaitAnalyzer::sportSoleCB(const sport_sole::SportSole& msg)
{
  sport_sole::SportSolePtr msg_new(new sport_sole::SportSole(msg));
  ros::Time ts_sport_sole_curr = msg.header.stamp + ros::Duration(ga_params_.sport_sole_time_offset);
  msg_new->header.stamp = ts_sport_sole_curr;
  cache_sport_sole_.add(msg_new);
}

void GaitAnalyzer::processSportSole(const sport_sole::SportSole& msg)
{
  const auto & stamp_sport_sole_curr = msg.header.stamp;

  gait_phase_fsm_.update(msg.pressures);
  uint8_t gait_state = gait_phase_fsm_.getGaitState();
  updateGaitState(gait_state);
  
  // Publish on /gait_analyzer/gait_state
  sport_sole::GaitState msg_gait_state;
  msg_gait_state.header.stamp = stamp_sport_sole_curr;
  for (auto lr: {LEFT, RIGHT})
  {
    msg_gait_state.gait_phase[lr] = static_cast<uint8_t>(sport_sole::getGaitPhaseLR(gait_state, lr));
  }
  pub_gait_state_.publish(msg_gait_state);


  // Calculate CoP
  pressures_ = msg.pressures;
  auto & cop = cop_;
  cop = sport_sole::getCoP(pressures_, vec_refpoints_, vec_refvecs_);

  //Update control input
  comkf::C u;
  u.copx() = cop.getX();
  u.copy() = cop.getY();

  if(!stamp_predict_prev_.isZero())
  {
    T dt = (stamp_sport_sole_curr - stamp_predict_prev_).toSec();
    // Predict only if dt is large enough
    if (dt > 1e-3) {
      const auto& x = comkf_.predict(u, dt);
      updateCoMEstimate(stamp_sport_sole_curr, x); // A priori estmiate
      stamp_predict_prev_ = stamp_sport_sole_curr;
    }
    gait_training_robot::ComkfCopMeasurement msg;
    msg.header.stamp = stamp_sport_sole_curr;
    msg.header.frame_id = ga_params_.global_frame;
    msg.px = u.x();
    msg.py = u.y();
    pub_comkf_cop_measurement_.publish(msg);
  }
  stamp_sport_sole_prev_ = stamp_sport_sole_curr;
  pub_cop_.publish(constructPointStampedMessage(stamp_sport_sole_curr, cop));
}

void GaitAnalyzer::updateGaitState(const uint8_t& gait_state)
{
  touches_ground_[HIND][LEFT] = gait_state & (1<<3);
  touches_ground_[FORE][LEFT] = gait_state & (1<<2);
  touches_ground_[HIND][RIGHT] = gait_state & (1<<1);
  touches_ground_[FORE][RIGHT] = gait_state & (1<<0);
}

void GaitAnalyzer::updateTfCB(const ros::TimerEvent& event)
{
  if (ga_params_.global_frame == ga_params_.publish_frame) return;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = tf_buffer_.lookupTransform(ga_params_.publish_frame, ga_params_.global_frame, ros::Time(), ros::Duration(0.001));
    tf2::fromMsg(tf_msg.transform, tf_global_to_publish_);
    is_initialized_tf_global_to_publish_ = true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0, "Will use the global_frame instead of the publish_frame in 'header.frame_id'. %s", ex.what());
    return;
  }
}



void GaitAnalyzer::updateCoMMeasurementFromSkeletons(const ros::Time & stamp, com_t & com_curr, comv_t & com_vel, const vec_joints_t & vec_joints)
{
  // Save CoM from the previous time step.
  auto com_prev = com_curr;

  // Calculate CoM
  if (com_model_ == COM_MODEL_PELVIS)
  {
    com_curr.setX(vec_joints[K4ABT_JOINT_PELVIS].getX());
    com_curr.setY(vec_joints[K4ABT_JOINT_PELVIS].getY());
    com_curr.setZ(vec_joints[K4ABT_JOINT_PELVIS].getZ());
  }
  else if (com_model_ == COM_MODEL_14_SEGMENT)
  {
    com_t sum(.0, .0, .0);
    double sum_mass = 0;
    for (const auto & segment: vec_segments)
    {
      const auto & proximal_joint = vec_joints[segment.joint_pair_.first];
      const auto & distal_joint = vec_joints[segment.joint_pair_.second];
      com_t segment_com = proximal_joint * segment.com_len_ratio_ + distal_joint * (1 - segment.com_len_ratio_);
      sum_mass += segment.weight_ratio_;
      sum += segment.weight_ratio_ * segment_com;
    }
    com_curr = sum / sum_mass;
  }
  else
  {
    ROS_ERROR("Unkown CoM model!");
    ros::shutdown();
  }
  // com_curr.setZ(z_ground_);

  // Calculate CoMv by discrete difference.
  auto delta_t = (stamp - stamp_com_prev_).toSec();
  // Skip the calculation if this is the first com ever sampled.
  if (!stamp_com_prev_.isZero()) {
    com_vel = (com_curr - com_prev) / delta_t;
    com_vel.setX(com_vel.getX() - ga_params_.belt_speed);
  }
}

void GaitAnalyzer::updateCoMMeasurement(const ros::Time & stamp, com_t & com_curr, comv_t & com_vel, 
                                        const geometry_msgs::PointStamped& msg)
{
  // Save CoM from the previous time step.
  auto com_prev = com_curr;

  // Calculate CoM
  tf2::fromMsg(msg.point, com_curr);
  // com_curr.setZ(z_ground_);

  // Calculate CoMv by discrete difference.
  auto delta_t = (stamp - stamp_com_prev_).toSec();
  // Skip the calculation if this is the first com ever sampled.
  if (!stamp_com_prev_.isZero()) {
    com_vel = (com_curr - com_prev) / delta_t;
    ROS_DEBUG_STREAM(delta_t << ": " << com_vel);
    com_vel.setX(com_vel.getX() - ga_params_.belt_speed);
  }
}

void GaitAnalyzer::updateXCoM(com_t & xcom, const com_t & com, const comv_t & com_vel)
{
  static double omega0 = sqrt(9.8 / (0.9));
  xcom = com + com_vel / omega0;
}

void GaitAnalyzer::updateCoMEstimate(const ros::Time & stamp, const comkf::S & x)
{
  com_[ESTIMATE].setX(x.px());
  com_[ESTIMATE].setY(x.py());
  com_[ESTIMATE].setZ(z_ground_);
  comv_[ESTIMATE].setX(x.vx());
  comv_[ESTIMATE].setY(x.vy());
  comv_[ESTIMATE].setZ(0);

  if (ga_params_.smoother_enabled)
  {
    com_smoother_[ESTIMATE] (com_[ESTIMATE]);
    comv_smoother_[ESTIMATE](comv_[ESTIMATE]);
  }
  updateXCoM(xcom_[ESTIMATE], com_[ESTIMATE], comv_[ESTIMATE]);

  const auto& msg_com_ptr =  constructPointStampedMessage(stamp, com_[ESTIMATE]);
  const auto& msg_comv_ptr = constructVector3StampedMessage(stamp, comv_[ESTIMATE]);
  time_synchronizer_estimate_.add2(msg_com_ptr);
  time_synchronizer_estimate_.add3(msg_comv_ptr);

  pub_com_ [ESTIMATE].publish(msg_com_ptr);
  pub_comv_[ESTIMATE].publish(msg_comv_ptr);
  pub_xcom_[ESTIMATE].publish(constructPointStampedMessage(stamp, xcom_[ESTIMATE]));

  gait_training_robot::ComkfState msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = ga_params_.global_frame;
  msg.px = x.px();
  msg.py = x.py();
  msg.vx = x.vx();
  msg.vy = x.vy();
  msg.bx = x.bx();
  msg.by = x.by();
  pub_comkf_state_.publish(msg);
}


void GaitAnalyzer::updateBoS(bos_t & res)
{
  bos_t bos_points;
  res.clear();
  
  // Define unit foot orientation vectors v0[]
  const auto& v0 = vec_refvecs_;

  // Forefoot and hindfoot reference points
  const auto& vec = vec_refpoints_;

  for(size_t lr: {LEFT, RIGHT}) footprints_[lr].clear();

  /*!
    addToBoS: Adds two points to the BoS polygon, one left and one right. Each point is
    specified by a base vector (fh) plus a vector in polar coordinates (rho, phi) in local foot frame. 
    \param fh: whether forefoot or hindfoot marker is used as the base vector 
    \param rho: the radial coordinate
    \param phi: the angular coordinate
  */
  auto addToBoS = [&, this](fore_hind_t fh, double rho, double phi){
    tf2::Vector3 pt[LEFT_RIGHT] = {
      vec[fh][LEFT] + (v0[LEFT] * rho).rotate({0, 0, 1}, phi),
      vec[fh][RIGHT] + (v0[RIGHT] * rho).rotate({0, 0, 1}, -phi)};
    
    tf2::Vector3 pt2[LEFT_RIGHT] = {
      vec[fh][LEFT] + (v0[LEFT] * rho*0.8).rotate({0, 0, 1}, phi),
      vec[fh][RIGHT] + (v0[RIGHT] * rho*0.8).rotate({0, 0, 1}, -phi)};

    for (size_t lr : {LEFT, RIGHT}) 
    {
      if (touches_ground_[fh][lr]) {
        footprints_[lr].push_back(pt2[lr]);
        bos_points.push_back(pt[lr]);
      }
    }
  };


#if 0
  // Oxford Foot Model 
  // https://www.c-motion.com/v3dwiki/index.php/Tutorial:_Oxford_Foot_Model

  addToBoS(FORE, 0.05, -M_PI_4);      // L/RD1M: Head of the first metatarsal
  addToBoS(FORE, 0.03, 0.0);        // L/RTOE: Center of the toe
  addToBoS(FORE, 0.03, M_PI_2);       // L/RD5M: Head of the fifth metatarsal
  addToBoS(FORE, 0.03, 6 * M_PI_4);

  addToBoS(HIND, 0.01, 5.5 * M_PI_4);   // L/RMMA: Medial malleolus
  addToBoS(HIND, 0.01, 2.5 * M_PI_4);   // L/RLMA: Lateral malleolus
  addToBoS(HIND, 0.02, 3 * M_PI_4);   // L/RLMA: Lateral malleolus
  addToBoS(HIND, 0.02, 5 * M_PI_4);   // L/RMMA: Medial malleolus
#else
  addToBoS(FORE, 0.0647,  2.1218);
  addToBoS(FORE, 0.0482, -0.1728); 
  addToBoS(FORE, 0.0488, -1.8540);

  addToBoS(HIND, 0.0316, -1.8014);
  addToBoS(HIND, 0.0159,  3.1652);
  addToBoS(HIND, 0.0393,  2.0510);
#endif


  if (bos_points.size() < 3)
    return;

  // Graham Scan (convex hull) https://en.wikipedia.org/wiki/Graham_scan
  // Step 1: find the point p0 with the min y coordinate, and put it in res
  struct {
    bool operator()(const bos_t::value_type & v1, const bos_t::value_type & v2) {
      return v1.getY() < v2.getY();
    }
  } compare_y;
  auto it_p0 = std::min_element(bos_points.begin(), bos_points.end(), compare_y);
  res.push_back(*it_p0);
  bos_points.erase(it_p0);

  // Step 2: sort the rest of the points by the polar angle with p0
  auto ccw = [](const bos_t::value_type & v0, const bos_t::value_type & v1, const bos_t::value_type & v2)->bool{
    return (v1 - v0).cross(v2 - v0).getZ() > 0;
  };
  it_p0 = res.begin();
  auto comparePolarAngle = [&ccw, &it_p0](const bos_t::value_type & v1, const bos_t::value_type & v2){
    return ccw(*it_p0, v1, v2);
  };
  std::sort(bos_points.begin(), bos_points.end(), comparePolarAngle);

  // Step 3: Loop through the rest of the points
  for (auto & point : bos_points)
  {
    while (res.size() > 1 && !ccw(*(res.end() - 2), *(res.end() - 1), point))
      res.resize(res.size() - 1);
    res.push_back(point);
  }
}

void GaitAnalyzer::updateMoS(mos_t & res, const com_t & xcom, const bos_t & bos_points)
{
  com_t pxcom = xcom;
  pxcom.setZ(z_ground_);

  double mos_sign = 1.0;
  double dist_min = 2.0;
  tf2::Vector3 pt_min;

  if (bos_points.size() < 3)
  {
    ROS_WARN_THROTTLE(10, "Ill-formed BoS polygon.");
    res.values[mos_t::mos_shortest].pt = pxcom;
    res.values[mos_t::mos_shortest].dist = 0.0;
    res.values[mos_t::mos_anteroposterior].pt = pxcom;
    res.values[mos_t::mos_anteroposterior].dist = 0.0;
    res.values[mos_t::mos_mediolateral].pt = pxcom;
    res.values[mos_t::mos_mediolateral].dist = 0.0;
    return;
  }

  // Part 1: Iterate through each line segment (*it1, *it2) in the bos polygon
  bos_t::const_iterator it1 = bos_points.cbegin(), it2;
  for (;it1 < bos_points.cend(); it1++)
  {
    if (it1 == bos_points.cend() - 1)
      it2 = bos_points.cbegin();
    else
      it2 = it1 + 1;
    
    tf2::Vector3 v1, v2;
    v1 = pxcom - *it1;
    v2 = *it2 - *it1;

    // Determin whether the XCoM is within the BoS polygon (mos_sign > 0.0)
    // assuming that we go counter-clockwise around the polygon
    if (mos_sign > 0.0)
      mos_sign = (v2.cross(v1).getZ() > 0) ? 1.0 : -1.0;

    // Project pxcom onto the line (*it1, *it2)
    double t = v1.dot(v2) / v2.length2();

    // Limit the projection to the range (0.0, 1.0)
    t = std::min(std::max(t, 0.0), 1.0);

    auto && pt = it1->lerp(*it2, t);
    double dist = (pt - pxcom).length();
    //std::cout << pt << pxcom << std::endl;
    if (dist < dist_min)
    {
      dist_min = dist;
      pt_min = pt;
    }
  }
  res.values[mos_t::mos_shortest].pt = pt_min;
  res.values[mos_t::mos_shortest].dist = mos_sign * dist_min;

  // Part 2: Calculate the bos in anteroposterior and mediolateral directions.
  // BoS extremities in AP and ML directions:
  int indices[4] = {};

  // The unit vector in the frontal plane pointing from the right to the left
  if (ml_vec_.isZero()) return;
  const tf2::Vector3& v0_ml = ml_vec_;

  // The unit vector in the sagitall plane pointing from the back to the front
  const tf2::Vector3& v0_ap = ap_vec_;

  // Iterate through each vertex of the BOS
  for (int i = 1; i < bos_points.size(); i++)
  {
    const auto & point = bos_points[i];

    double pos_ap = point.dot(v0_ap);
    double pos_ml = point.dot(v0_ml);

    if (pos_ap < bos_points[indices[0]].dot(v0_ap)) indices[0] = i; // bottom
    if (pos_ap > bos_points[indices[1]].dot(v0_ap)) indices[1] = i; // top
    if (pos_ml < bos_points[indices[2]].dot(v0_ml)) indices[2] = i; // right
    if (pos_ml > bos_points[indices[3]].dot(v0_ml)) indices[3] = i; // left
  }

  // Project xcom onto the line connecting the bottom BoS point with the top BoS point
  {
    double v1 = (pxcom - bos_points[indices[0]]).dot(v0_ap);
    double v2 = (bos_points[indices[1]] - bos_points[indices[0]]).dot(v0_ap);
    double t = v1 / v2;
    
    if (t > 0.5)
    {
      res.values[mos_t::mos_anteroposterior].dist = (1.0 - t) * v2;
      res.values[mos_t::mos_anteroposterior].pt = bos_points[indices[1]]; // top
    }
    else
    {
      res.values[mos_t::mos_anteroposterior].dist = (t - 0.0) * v2;
      res.values[mos_t::mos_anteroposterior].pt = bos_points[indices[0]]; // bottom
    }
  }

  // Project xcom onto the line connecting the left BoS point with the right BoS point
  {
    double v1 = (pxcom - bos_points[indices[2]]).dot(v0_ml);
    double v2 = (bos_points[indices[3]] - bos_points[indices[2]]).dot(v0_ml);
    double t = v1 / v2;
    
    if (t > 0.5)
    {
      res.values[mos_t::mos_mediolateral].dist = (1.0 - t) * v2;
      res.values[mos_t::mos_mediolateral].pt = bos_points[indices[3]]; // left
    }
    else
    {
      res.values[mos_t::mos_mediolateral].dist = (t - 0.0) * v2;
      res.values[mos_t::mos_mediolateral].pt = bos_points[indices[2]]; // right
    }
    //ROS_INFO_STREAM("mos: " << t);
  }
}

void GaitAnalyzer::updateRefPoints(const PoseType::ConstPtr& msg_foot_l, const PoseType::ConstPtr& msg_foot_r)
{
  PoseType::ConstPtr msg_feet[LEFT_RIGHT] = {msg_foot_l, msg_foot_r};
  
  for (size_t lr : {LEFT, RIGHT})
  {
    tf2::fromMsg(msg_feet[lr]->pose.pose.position, vec_refpoints_[HIND][lr]);
    tf2::Quaternion quat;
    tf2::fromMsg(msg_feet[lr]->pose.pose.orientation, quat);

    if (ga_params_.global_frame != msg_feet[lr]->header.frame_id)
    {
      tf2::Transform tf_to_global;
      try
      {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg = tf_buffer_.lookupTransform(ga_params_.global_frame, msg_feet[lr]->header.frame_id, msg_feet[lr]->header.stamp, ros::Duration(0, 0));
        tf2::fromMsg(tf_msg.transform, tf_to_global);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(5.0, "Cannot transform foot pose from '%s' to '%s'. %s", msg_feet[lr]->header.frame_id.c_str(), ga_params_.global_frame.c_str(), ex.what());
        return;
      }
      vec_refpoints_[HIND][lr] = tf_to_global * vec_refpoints_[HIND][lr];
      quat = tf_to_global * quat;
    }

    vec_refvecs_[lr] = tf2::quatRotate(quat, {1.0, 0.0, 0.0});
    vec_refpoints_[FORE][lr] = vec_refpoints_[HIND][lr] + vec_refvecs_[lr] * FOOT_LENGTH;
    
    vec_refvecs_[lr].setZ(0);
    vec_refvecs_[lr] = vec_refvecs_[lr].normalize();
    vec_refpoints_[FORE][lr].setZ(z_ground_);
    vec_refpoints_[HIND][lr].setZ(z_ground_);
  }
}


void GaitAnalyzer::updateGaitPhase()
{
  double ground_clearance[2] = 
  {
    vec_joints_[K4ABT_JOINT_FOOT_LEFT].getZ() - z_ground_,
    vec_joints_[K4ABT_JOINT_FOOT_RIGHT].getZ() - z_ground_
  };

  //ROS_INFO_STREAM("clearance: " << ground_clearance[0] << ", " << ground_clearance[1]);
  std_msgs::Float64 msg[2];
  msg[0].data = ground_clearance[0];
  msg[1].data = ground_clearance[1];
  pub_ground_clearance_left_.publish(msg[0]);
  pub_ground_clearance_right_.publish(msg[1]);

  //ROS_INFO_STREAM("gait phase: " << gait_phase_[0] << ", " << gait_phase_[1]);

}


// Helper method for converting a tf2::Vector3 object to a geometry_msgs::Point32 object
geometry_msgs::Point32 GaitAnalyzer::vector3ToPoint32(const tf2::Vector3 & vec)
{
  geometry_msgs::Point32 res;
  res.x = vec.getX();
  res.y = vec.getY();
  res.z = vec.getZ();
  return res;
}

geometry_msgs::PointStampedPtr GaitAnalyzer::constructPointStampedMessage(const ros::Time & stamp, const tf2::Vector3 & vec) 
{
  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  if (is_initialized_tf_global_to_publish_)
  {
    msg->header.frame_id = ga_params_.publish_frame;
    tf2::toMsg(tf_global_to_publish_ * vec, msg->point);
  }
  else
  {
    msg->header.frame_id = ga_params_.global_frame;
    tf2::toMsg(vec, msg->point);
  }

  msg->header.stamp = stamp;
  msg->point.z = z_ground_;
  return msg;
}

geometry_msgs::Vector3StampedPtr GaitAnalyzer::constructVector3StampedMessage(const ros::Time & stamp, const comv_t & vec) 
{
  geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
  msg->header.frame_id = ga_params_.global_frame;
  msg->header.stamp = stamp;
  msg->vector.x = vec.getX();
  msg->vector.y = vec.getY();
  msg->vector.z = vec.getZ();
  return msg;
}

geometry_msgs::PolygonStamped GaitAnalyzer::constructPolygonMessage(const ros::Time & stamp, const bos_t & bos_points)
{
  geometry_msgs::PolygonStamped msg_bos;
  msg_bos.header.stamp = stamp;
  std::function<geometry_msgs::Point32(const tf2::Vector3&)> getPoint32;
  if (is_initialized_tf_global_to_publish_)
  {
    msg_bos.header.frame_id = ga_params_.publish_frame;
    getPoint32 = [this](const tf2::Vector3& point){return vector3ToPoint32(tf_global_to_publish_ * point);};
  }
  else
  {
    msg_bos.header.frame_id = ga_params_.global_frame;
    getPoint32 = [this](const tf2::Vector3& point){return vector3ToPoint32(point);};
  }
  for (const auto & point : bos_points) {
    msg_bos.polygon.points.push_back(getPoint32(point));
    msg_bos.polygon.points.back().z = z_ground_;
  }
  return msg_bos;
}

visualization_msgs::MarkerArrayPtr GaitAnalyzer::constructMosMarkerArrayMessage(const ros::Time & stamp, const com_t & xcom, const mos_t & mos)
{
  MarkerArrayPtr markerArrayPtr(new MarkerArray);
  for (size_t i = 0; i < mos_t::mos_count; ++i)
  {
    MarkerPtr markerPtr(new Marker);
    markerPtr->header.stamp = stamp;
    markerPtr->header.frame_id = ga_params_.global_frame;
    markerPtr->lifetime = ros::Duration(0.05);
    markerPtr->ns = "gait_analyzer";
    markerPtr->id = i;
    markerPtr->type = Marker::ARROW;

    geometry_msgs::Point p1;
    p1.x = xcom.getX();
    p1.y = xcom.getY();
    p1.z = z_ground_;
    markerPtr->points.push_back(p1);

    // This is the point on the BoS polygon with the shortest dist to XCoM
    geometry_msgs::Point p2;
    p2.x = mos.values[i].pt.getX();
    p2.y = mos.values[i].pt.getY();
    p2.z = z_ground_;
    markerPtr->points.push_back(p2);
    
    markerPtr->scale.x = 0.02;
    markerPtr->scale.y = 0.04;
    markerPtr->scale.z = 0.0;
    markerPtr->color.a = 1.0;
    markerPtr->color.r = mos.values[i].dist > 0 ? 0.0 : 1.0; // red if mos is negative
    markerPtr->color.g = mos.values[i].dist > 0 ? 1.0 : 0.0; // green if mos is positve
    markerPtr->color.b = 0.0;

    markerArrayPtr->markers.push_back(*markerPtr);
  }
  return markerArrayPtr;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gait_analyzer");
  GaitAnalyzer ga;

  ros::spin();
  return 0;
}