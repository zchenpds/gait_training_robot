//Purpose: bring together data from  1) accel, quaternion, pressures of left & right sport soles  2) kinect IMU  3) kinect skeleton info

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "gait_training_robot/gait_analyzer.h"

using namespace visualization_msgs;

// Class com_kf::KalmanFilter
com_kf::KalmanFilter::KalmanFilter()
{
  // Set initial state
  x.setZero();
}

const com_kf::State& com_kf::KalmanFilter::predict(const com_kf::Control & u, T dt)
{
  if (dt > T(0.0))
    sys.setSamplingPeriod(dt);
  return ExtendedKalmanFilter::predict(sys, u);
}

const com_kf::State& com_kf::KalmanFilter::update(const com_kf::Measurement& zpv)
{
  return ExtendedKalmanFilter::update(mm, zpv);
}

void com_kf::KalmanFilter::setSystemCov(T sigma_p, T sigma_v, T sampling_period)
{
  com_kf::KalmanFilter::sys.setSamplingPeriod(sampling_period);

  T var_v = sigma_v * sigma_v * sampling_period * sampling_period;
  auto system_cov = decltype(sys.getCovariance()){};
  system_cov.setZero();
  system_cov.template block<2, 2>(com_kf::State::VX, com_kf::State::VX) = var_v * Eigen::Matrix<T, 2, 2>::Identity();
  sys.setCovariance(system_cov);
}
void com_kf::KalmanFilter::setMeasurementCov(T sigma_p, T sigma_v)
{
  auto measurement_cov = decltype(com_kf::KalmanFilter::mm.getCovariance()){};
  measurement_cov.setZero();
  measurement_cov.template block<2, 2>(com_kf::Measurement::PX, com_kf::State::PX) = sigma_p * sigma_p * Eigen::Matrix<T, 2, 2>::Identity();
  measurement_cov.template block<2, 2>(com_kf::Measurement::VX, com_kf::State::VX) = sigma_v * sigma_v * Eigen::Matrix<T, 2, 2>::Identity();
  
  com_kf::KalmanFilter::mm.setCovariance(measurement_cov);
}

com_kf::SystemModel com_kf::KalmanFilter::sys;
com_kf::MeasurementModel com_kf::KalmanFilter::mm;

void com_kf::KalmanFilterParams::print()
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
  cop_t cop = {};
  double pressure_sum = 0.0;
  for (size_t i = 0; i < NUM_2XPSENSOR; i++) {
    left_right_t lr = (i / NUM_PSENSOR == 0 ? LEFT : RIGHT);
    double th = atan2(vec_refvecs[lr].getY(), vec_refvecs[lr].getX()) + Theta[i];
    psensor_locs[i].setX(vec_refpoints[ANKLE][lr].getX() + Rho[i] * cos(th));
    psensor_locs[i].setY(vec_refpoints[ANKLE][lr].getY() + Rho[i] * sin(th));
    psensor_locs[i].setZ(0.0);

    // Avoid dividing by zero
    const double min_pressure = 0.1;
    auto pressure = std::max(pressures[i], min_pressure);
    
    pressure_sum += pressure;
    cop += pressure * psensor_locs[i];
  }
  cop /= pressure_sum;
  return cop;
}


void sport_sole::GaitPhaseFSM::update(const SportSole::_pressures_type & pressures)
{
  for (size_t lr : {LEFT, RIGHT}) {
    size_t i0 = (lr==LEFT ? 0 : 8);
    double p_hind_sum = pressures[i0 + 5] + pressures[i0 + 6]; // 6~7
    double p_fore_sum = pressures[i0 + 0] + pressures[i0 + 1] + pressures[i0 + 2] + pressures[i0 + 3] + pressures[i0 + 4]; // 1~5

    auto & gait_phase = gait_phases_[lr];
    switch (gait_phase) {
      case GaitPhase::Swing:
        if (p_hind_sum > p_threshold_) 
          gait_phase = GaitPhase::Stance1;
        break;
      case GaitPhase::Stance1:
        if (p_fore_sum > p_threshold_) 
          gait_phase = GaitPhase::Stance2;
        break;
      case GaitPhase::Stance2:
        if (p_hind_sum <= p_threshold_) 
          gait_phase = GaitPhase::Stance3;
        break;
      case GaitPhase::Stance3:
        if (p_fore_sum <= p_threshold_) 
          gait_phase = GaitPhase::Swing;
        break;
    }
  }
}

uint8_t sport_sole::GaitPhaseFSM::getGaitPhase()
{
  return static_cast<uint8_t>(gait_phases_[LEFT]) << 2 | static_cast<uint8_t>(gait_phases_[RIGHT]);
}

GaitAnalyzer::GaitAnalyzer(const ros::NodeHandle& n, const ros::NodeHandle& p):
  com_model_(COM_MODEL_14_SEGMENT), // COM_MODEL_PELVIS, COM_MODEL_14_SEGMENT
  touches_ground_{
    {true, true}, // Left ankle, right ankle
    {true, true} // Left foot, right foot
    },
  z_ground_(0.0),
  nh_(n),
  private_nh_(p),
  comkf_nh_(private_nh_, "comkf"),
  sub_sport_sole_(nh_, "/sport_sole_publisher/sport_sole", 1),
  cache_sport_sole_(sub_sport_sole_, 100),
  tf_listener_(tf_buffer_),
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
  comkf_.setSystemCov(comkf_params_.system_noise_p, comkf_params_.system_noise_v, comkf_params_.sampling_period);
  comkf_.setMeasurementCov(comkf_params_.measurement_noise_p, comkf_params_.measurement_noise_v);


  sub_skeletons_ = nh_.subscribe("/body_tracking_data", 5, &GaitAnalyzer::skeletonsCB, this );

  for (size_t lr: {LEFT, RIGHT}) {
    auto str_lr = std::string(!lr?"_l": "_r");
    pub_footprints_[lr] = private_nh_.advertise<geometry_msgs::PolygonStamped>("footprint" + str_lr, 1);
  }

  pub_pcom_pos_measurement_ = private_nh_.advertise<geometry_msgs::PointStamped>("pcom_pos_measurement", 1);
  pub_pcom_vel_measurement_ = private_nh_.advertise<geometry_msgs::Vector3Stamped>("pcom_vel_measurement", 1);
  pub_xcom_measurement_ = private_nh_.advertise<geometry_msgs::PointStamped>("xcom_measurement", 1);
  pub_pcom_pos_estimate_ = private_nh_.advertise<geometry_msgs::PointStamped>("pcom_pos_estimate", 1);
  pub_pcom_vel_estimate_ = private_nh_.advertise<geometry_msgs::Vector3Stamped>("pcom_vel_estimate", 1);
  pub_xcom_estimate_ = private_nh_.advertise<geometry_msgs::PointStamped>("xcom_estimate", 1);
  
  pub_bos_ = private_nh_.advertise<geometry_msgs::PolygonStamped>("bos", 1);
  pub_mos_ = private_nh_.advertise<visualization_msgs::MarkerArray>("mos", 1);
  for (int i = 0; i < mos_t::mos_count; ++i)
    pub_mos_values_[i] = private_nh_.advertise<std_msgs::Float64>("mos_values" + std::to_string(i), 1);

  pub_ground_clearance_left_ = private_nh_.advertise<std_msgs::Float64>("ground_clearance_left", 1);
  pub_ground_clearance_right_ = private_nh_.advertise<std_msgs::Float64>("ground_clearance_right", 1);
}

void GaitAnalyzer::skeletonsCB(const visualization_msgs::MarkerArray& msg)
{  
  double dist_min_pelvis = 100.0;
  //double idx = -1; // body id with the min dist of pelvis from camera center
  auto it_pelvis_closest = msg.markers.end();// iterator of the pelvis marker of the closest body

  // Find the closest body, K4ABT_JOINT_PELVIS = 0
  if (sub_id_ < 0) {
    for (auto it = msg.markers.begin(); it < msg.markers.end(); it += K4ABT_JOINT_COUNT) {
      // The coordinates are in expressed in /depth_camera_link
      double dist_pelvis = hypot(it->pose.position.x, it->pose.position.z);
      if (dist_pelvis < dist_min_pelvis) {
        dist_pelvis = dist_min_pelvis;
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
      geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(ga_params_.global_frame, "depth_camera_link", stamp_skeleton_curr);
      fromMsg(tf_msg.transform, tf_depth_to_global_);
    }
    catch (tf2::TransformException & ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
      //ros::Duration(1.0).sleep();
    }
    
    // Coordinate Transformation and Projection
    for (int i = 0; i < K4ABT_JOINT_COUNT; i++)
    {
      // Convert messages to tf2::vectors for easier calculation
      fromMsg((it_pelvis_closest + i)->pose.position, vec_joints_[i]);
      vec_joints_[i] = tf_depth_to_global_ * vec_joints_[i];
    }

    vec_refpoints_[FOOT][LEFT] = vec_joints_[K4ABT_JOINT_FOOT_LEFT];
    vec_refpoints_[FOOT][RIGHT] = vec_joints_[K4ABT_JOINT_FOOT_RIGHT];
    vec_refpoints_[ANKLE][LEFT] = vec_joints_[K4ABT_JOINT_ANKLE_LEFT];
    vec_refpoints_[ANKLE][RIGHT] = vec_joints_[K4ABT_JOINT_ANKLE_RIGHT];
    vec_refvecs_[LEFT] = vec_joints_[K4ABT_JOINT_FOOT_LEFT] - vec_joints_[K4ABT_JOINT_ANKLE_LEFT];
    vec_refvecs_[RIGHT] = vec_joints_[K4ABT_JOINT_FOOT_RIGHT] - vec_joints_[K4ABT_JOINT_ANKLE_RIGHT];
    for (size_t lr : {LEFT, RIGHT})
    {
      vec_refpoints_[FOOT][lr].setZ(z_ground_);
      vec_refpoints_[ANKLE][lr].setZ(z_ground_);
      vec_refvecs_[lr].setZ(0);
      vec_refvecs_[lr] = vec_refvecs_[lr].normalize();
    }

    double z_min = std::min(vec_joints_[K4ABT_JOINT_FOOT_LEFT].getZ(), vec_joints_[K4ABT_JOINT_FOOT_RIGHT].getZ());
    z_ground_ = 0.95 * std::min(z_ground_, z_min) + 0.05 * z_min;
    //z_ground_ = 0.5;

    // Update gait phase
    //updateGaitPhase();

    // Calculate pcom (projected center of mass), comv and xcom
    updateCoMMeasurement(stamp_skeleton_curr);

    // Publish measurements
    pub_pcom_pos_measurement_.publish(constructPointStampedMessage(stamp_skeleton_curr, pcom_pos_measurement_));
    pub_pcom_vel_measurement_.publish(constructVector3StampedMessage(stamp_skeleton_curr, pcom_vel_measurement_));
    pub_xcom_measurement_.publish(constructPointStampedMessage(stamp_skeleton_curr, xcom_measurement_));
    

    // If at least one skeleton message has been received after the receipt of the first sport_sole message, do an EKF prediction
    if (!stamp_skeleton_prev_.isZero())
    {
      auto sport_sole_ptrs = cache_sport_sole_.getInterval(stamp_skeleton_prev_, stamp_skeleton_curr);
      for (const auto & ss_ptr : sport_sole_ptrs)
      {
        // EKF prediction is done here
        sportSoleCB(*ss_ptr);
      }
      ROS_INFO_STREAM(sport_sole_ptrs.size() << " predictions between t=" <<
        (stamp_skeleton_prev_ - stamp_base_).toSec() << "s and t=" << (stamp_skeleton_curr - stamp_base_).toSec() << "s are done");
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
      gait_phase_fsm_.update(msg_sport_sole_ptr->pressures);
      updateGaitState(gait_phase_fsm_.getGaitPhase());
      // updateGaitState(msg_sport_sole_ptr->gait_state);
      
      com_kf::Measurement zpv;
      zpv.px() = pcom_pos_measurement_.getX();
      zpv.py() = pcom_pos_measurement_.getY();
      zpv.vx() = pcom_vel_measurement_.getX();
      zpv.vy() = pcom_vel_measurement_.getY();

      // Is this the first skeleton message received after the receipt of the first sport_sole message?
      if (comkf_initialized_ == false) {
        // This is the first time skeleton message received
        // Use the position measurement to initialize the state
        com_kf::State x0;
        x0.px() = pcom_pos_measurement_.getX();
        x0.py() = pcom_pos_measurement_.getY();
        x0.vx() = pcom_vel_measurement_.getX();
        x0.vy() = pcom_vel_measurement_.getY();
        comkf_.init(x0);
        ROS_INFO_STREAM("COMKF state initialized to " << x0.transpose());
        comkf_initialized_ = true;
        stamp_sport_sole_prev_ = stamp_skeleton_curr;
      }
      else {
        const auto & x = comkf_.update(zpv);
        updateCoMEstimate(stamp_skeleton_curr, x);
        ROS_INFO_STREAM_THROTTLE(1, "COMKF state updated to " << x.transpose());
      }
      

      // Publish estimates
      pub_pcom_pos_estimate_.publish(constructPointStampedMessage(stamp_skeleton_curr, pcom_pos_estimate_));
      pub_pcom_vel_estimate_.publish(constructVector3StampedMessage(stamp_skeleton_curr, pcom_vel_estimate_));
      pub_xcom_estimate_.publish(constructPointStampedMessage(stamp_skeleton_curr, xcom_estimate_));   
    }

    // xcom to be used for mos calculation
    com_t xcom = {};
    if (stamp_pcom_estimate_ >= stamp_pcom_measurement_) {
      xcom = xcom_estimate_;
    }
    else {
      xcom = xcom_measurement_;
      ROS_WARN_STREAM_THROTTLE(5, "Using measured xcom for calculating MoS. comkf skipped.");
    }

    // Calculate the base of support polygon, as well as update footprint polygons
    updateBoS();
    pub_bos_.publish(constructBosPolygonMessage(stamp_skeleton_curr));
    for (size_t lr : {LEFT, RIGHT}) {
      geometry_msgs::PolygonStamped msg_footprint;
      msg_footprint.header.stamp = it_pelvis_closest->header.stamp;
      msg_footprint.header.frame_id = ga_params_.global_frame;
      for (const auto & point : footprints_[lr])
        msg_footprint.polygon.points.push_back(vector3ToPoint32(point));
      pub_footprints_[lr].publish(msg_footprint);
    }
    
    // Calculate margin of stability
    updateMoS(xcom);
    pub_mos_.publish(constructMosMarkerArrayMessage(stamp_skeleton_curr, xcom));
    for (size_t i = 0; i < mos_t::mos_count; i++)
    {
      std_msgs::Float64 msg;
      msg.data = mos_.values[i].dist;
      pub_mos_values_[i].publish(msg);
    }

    stamp_skeleton_prev_ = stamp_skeleton_curr;
  }
}

void GaitAnalyzer::sportSoleCB(const sport_sole::SportSole& msg)
{
  const auto & stamp_sport_sole_curr = msg.header.stamp;

  pressures_ = msg.pressures;

  // Calculate CoP
  cop_t cop = sport_sole::getCoP(pressures_, vec_refpoints_, vec_refvecs_);
  com_kf::Control u;
  u.copx() = cop.getX();
  u.copy() = cop.getY();

  T dt;
  if(!stamp_sport_sole_prev_.isZero()) {
    dt = (stamp_sport_sole_curr - stamp_sport_sole_prev_).toSec();
  }
  else {
    // We will get here only the first time GaitAnalyzer::sportSoleCB is called.
    dt = 0.0;
    stamp_sport_sole_prev_ = stamp_sport_sole_curr;
  }
  
  // Predict only if dt is large enough
  if (dt > 0.001) {
    // Avoid too large a step. Nominal dt = 0.01
    dt = std::min(dt, T(0.1));
    comkf_.predict(u, dt);
    stamp_sport_sole_prev_ = stamp_sport_sole_curr;
  }

}

void GaitAnalyzer::updateGaitState(const uint8_t& gait_state)
{
  touches_ground_[ANKLE][LEFT] = gait_state & (1<<3);
  touches_ground_[FOOT][LEFT] = gait_state & (1<<2);
  touches_ground_[ANKLE][RIGHT] = gait_state & (1<<1);
  touches_ground_[FOOT][RIGHT] = gait_state & (1<<0);
}


void GaitAnalyzer::updateCoMMeasurement(const ros::Time & stamp)
{
  stamp_pcom_measurement_ = stamp;

  // Save CoM from the previous time step.
  auto & com_curr = pcom_pos_measurement_;
  auto com_prev = com_curr;

  // Calculate CoM
  if (com_model_ == COM_MODEL_PELVIS)
  {
    com_curr.setX(vec_joints_[K4ABT_JOINT_PELVIS].getX());
    com_curr.setY(vec_joints_[K4ABT_JOINT_PELVIS].getY());
    com_curr.setZ(vec_joints_[K4ABT_JOINT_PELVIS].getZ());
  }
  else if (com_model_ == COM_MODEL_14_SEGMENT)
  {
    com_t sum{};
    double sum_mass = 0;
    for (const auto & segment: vec_segments)
    {
      const auto & proximal_joint = vec_joints_[segment.joint_pair_.first];
      const auto & distal_joint = vec_joints_[segment.joint_pair_.second];
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

  // Calculate CoMv
  auto delta_t = (stamp - stamp_skeleton_prev_).toSec();
  // Skip the calculation if this is the first com ever sampled.
  if (!stamp_skeleton_prev_.isZero()) {
    pcom_vel_measurement_ = (com_curr - com_prev) / delta_t;
    pcom_vel_measurement_.setX(pcom_vel_measurement_.getX() - ga_params_.belt_speed);
  }

  // Calculate XCoM
  static double omega0 = sqrt(9.8 / (0.9));
  xcom_measurement_ = pcom_pos_measurement_ + pcom_vel_measurement_ / omega0;
}

void GaitAnalyzer::updateCoMEstimate(const ros::Time & stamp, const com_kf::State & x)
{
  stamp_pcom_estimate_ = stamp;

  pcom_pos_estimate_.setX(x.px());
  pcom_pos_estimate_.setY(x.py());
  pcom_pos_estimate_.setZ(z_ground_);
  pcom_vel_estimate_.setX(x.vx());
  pcom_vel_estimate_.setY(x.vy());
  pcom_vel_estimate_.setZ(0);

  // Calculate XCoM
  static double omega0 = sqrt(9.8 / (0.9));
  xcom_estimate_ = pcom_pos_estimate_ + pcom_vel_estimate_ / omega0;
}


void GaitAnalyzer::updateBoS()
{
  bos_t bos_points;
  bos_t & res = bos_points_;
  res.clear();
  
#if 1
  // Define unit foot orientation vectors v0[]
  bos_t::value_type v0[LEFT_RIGHT] = {
    vec_joints_[K4ABT_JOINT_FOOT_LEFT] - vec_joints_[K4ABT_JOINT_ANKLE_LEFT],
    vec_joints_[K4ABT_JOINT_FOOT_RIGHT] - vec_joints_[K4ABT_JOINT_ANKLE_RIGHT]
  };

  // Visual markers for feet and ankles
  bos_t::value_type vec[FOOT_ANKLE][LEFT_RIGHT] = {
    {vec_joints_[K4ABT_JOINT_FOOT_LEFT], vec_joints_[K4ABT_JOINT_FOOT_RIGHT]},
    {vec_joints_[K4ABT_JOINT_ANKLE_LEFT], vec_joints_[K4ABT_JOINT_ANKLE_RIGHT]}
  };
#else
  // Define unit foot orientation vectors v0[]
  bos_t::value_type v0[LEFT_RIGHT] = {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};

  // Visual markers for feet and ankles
  bos_t::value_type vec[FOOT_ANKLE][LEFT_RIGHT];

  const double FOOT_LENGTH = 0.2;
  
  for (size_t lr : {LEFT, RIGHT})
  {
    const auto & q_msg = pose_estimates_[lr].orientation;
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    v0[lr] = tf2::quatRotate(q, v0[lr]);
    const auto & p_msg = pose_estimates_[lr].position;
    vec[ANKLE][lr].setValue(p_msg.x, p_msg.y, p_msg.z);
    vec[FOOT][lr] = vec[ANKLE][lr] + v0[lr] * FOOT_LENGTH;
  }

#endif

  for (size_t lr : {LEFT, RIGHT})
  {
    v0[lr].setZ(0);
    v0[lr] = v0[lr].normalize();
    vec[FOOT][lr].setZ(z_ground_);
    vec[ANKLE][lr].setZ(z_ground_);
    footprints_[lr].clear();
  }

  /*!
    addToBoS: Adds two points to the BoS polygon, one left and one right. Each point is
    specified by a base vector (ankle_or_foot) plus a vector in polar coordinates (rho, phi) in local foot frame. 
    \param ankle_or_foot: whether foot or ankle marker is used as the base vector 
    \param rho: the radial coordinate
    \param phi: the angular coordinate
  */
  auto addToBoS = [&, this](ankle_foot_t ankle_or_foot, double rho, double phi){
    tf2::Vector3 pt[LEFT_RIGHT] = {
      vec[ankle_or_foot][LEFT] + (v0[LEFT] * rho).rotate({0, 0, 1}, phi),
      vec[ankle_or_foot][RIGHT] + (v0[RIGHT] * rho).rotate({0, 0, 1}, -phi)};
    
    for (size_t lr : {LEFT, RIGHT}) 
    {
      footprints_[lr].push_back(pt[lr]);
      if (touches_ground_[ankle_or_foot][lr])
        bos_points.push_back(pt[lr]);
    }
  };


#if 0
  // Oxford Foot Model 
  // https://www.c-motion.com/v3dwiki/index.php/Tutorial:_Oxford_Foot_Model

  addToBoS(FOOT, 0.05, -M_PI_4);      // L/RD1M: Head of the first metatarsal
  addToBoS(FOOT, 0.03, 0.0);        // L/RTOE: Center of the toe
  addToBoS(FOOT, 0.03, M_PI_2);       // L/RD5M: Head of the fifth metatarsal
  addToBoS(FOOT, 0.03, 6 * M_PI_4);

  addToBoS(ANKLE, 0.01, 5.5 * M_PI_4);   // L/RMMA: Medial malleolus
  addToBoS(ANKLE, 0.01, 2.5 * M_PI_4);   // L/RLMA: Lateral malleolus
  addToBoS(ANKLE, 0.02, 3 * M_PI_4);   // L/RLMA: Lateral malleolus
  addToBoS(ANKLE, 0.02, 5 * M_PI_4);   // L/RMMA: Medial malleolus
#else
  addToBoS(FOOT, 0.0647,  2.1218);
  addToBoS(FOOT, 0.0482, -0.1728); 
  addToBoS(FOOT, 0.0488, -1.8540);

  addToBoS(ANKLE, 0.0316, -1.8014);
  addToBoS(ANKLE, 0.0159,  3.1652);
  addToBoS(ANKLE, 0.0393,  2.0510);
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

void GaitAnalyzer::updateMoS(const com_t & xcom)
{
  mos_t & res = mos_;
  const bos_t & bos_points = bos_points_;
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
  auto v0_ml = ( vec_joints_[K4ABT_JOINT_CLAVICLE_LEFT] - vec_joints_[K4ABT_JOINT_CLAVICLE_RIGHT]
    + vec_joints_[K4ABT_JOINT_HIP_LEFT] - vec_joints_[K4ABT_JOINT_HIP_RIGHT] ).normalize();

  // The unit vector in the sagitall plane pointing from the back to the front
  auto v0_ap = v0_ml.rotate({0, 0, 1}, -M_PI_2);

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

geometry_msgs::PointStamped GaitAnalyzer::constructPointStampedMessage(const ros::Time & stamp, const com_t & vec) 
{
  geometry_msgs::PointStamped msg;
  msg.header.frame_id = ga_params_.global_frame;
  msg.header.stamp = stamp;
  msg.point.x = vec.getX();
  msg.point.y = vec.getY();
  msg.point.z = z_ground_;
  return msg;
}

geometry_msgs::Vector3Stamped GaitAnalyzer::constructVector3StampedMessage(const ros::Time & stamp, const comv_t & vec) 
{
  geometry_msgs::Vector3Stamped msg;
  msg.header.frame_id = ga_params_.global_frame;
  msg.header.stamp = stamp;
  msg.vector.x = vec.getX();
  msg.vector.y = vec.getY();
  return msg;
}

geometry_msgs::PolygonStamped GaitAnalyzer::constructBosPolygonMessage(const ros::Time & stamp)
{
  geometry_msgs::PolygonStamped msg_bos;
  msg_bos.header.stamp = stamp;
  msg_bos.header.frame_id = ga_params_.global_frame;
  for (const auto & point : bos_points_) {
    msg_bos.polygon.points.push_back(vector3ToPoint32(point));
    msg_bos.polygon.points.back().z = z_ground_;
  }
  return msg_bos;
}

visualization_msgs::MarkerArrayPtr GaitAnalyzer::constructMosMarkerArrayMessage(const ros::Time & stamp, const com_t & xcom)
{
  MarkerArrayPtr markerArrayPtr(new MarkerArray);
  const auto & mos = mos_;
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