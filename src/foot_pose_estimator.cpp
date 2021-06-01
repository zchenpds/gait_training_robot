#include "gait_training_robot/foot_pose_estimator.h"
#include "gait_training_robot/kalman_filter_common.h"
#include <fstream>

// Log
#include <stdlib.h>
std::string path_to_home = getenv("HOME");
std::ofstream ofs_log[LEFT_RIGHT] = {
  std::ofstream(path_to_home + "/.ros/gait_training_robot/fpel.log"), 
  std::ofstream(path_to_home + "/.ros/gait_training_robot/fper.log")};
using namespace sport_sole;

void FootPoseEstimatorParams::print()
{
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)     \
    ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

    ROS_PARAM_LIST
  #undef LIST_ENTRY
}

FootPoseEstimator::FootPoseEstimator(const ros::NodeHandle& n, const ros::NodeHandle& p):
  sub_id_(-1),
  nh_(n),
  private_nh_(p),
  sub_skeletons_(nh_.subscribe("/body_tracking_data", 200, &FootPoseEstimator::skeletonsCB, this)),
  sub_sport_sole_(nh_.subscribe("/sport_sole_publisher/sport_sole", 200, &FootPoseEstimator::sportSoleCB, this)),
  cache_sport_sole_(800),
  tf_listener_(tf_buffer_),
  is_initialized_tf_global_to_publish_(false)
{
  // Collect ROS parameters from the param server or from the command line
  #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
    private_nh_.param(#param_variable, params_.param_variable, param_default_val);
    
    ROS_PARAM_LIST
  #undef LIST_ENTRY

  // Print all parameters
  ROS_INFO("FootPoseEstimator Parameters:");
  params_.print();

  for (auto lr : {LEFT, RIGHT})
    cache_kinect_measurements_[lr].setCacheSize(100);
  
  // std::cout << "Waiting for /body_tracking_data" << std::flush;

  // Load parameters
  const auto& var_q  = pow(params_.system_noise_q,      2);
  const auto& var_w  = pow(params_.system_noise_w,      2);
  const auto& var_p  = pow(params_.system_noise_p,      2);
  const auto& var_v  = pow(params_.system_noise_v,      2);
  const auto& var_a  = pow(params_.system_noise_a,      2);
  const auto& var_ab = pow(params_.system_noise_ab,     2);
  const auto& var_wb = pow(params_.system_noise_wb,     2);

  for (auto lr : {LEFT, RIGHT})
  {
    auto& ekf = ekf_[lr];
    setModelCovariance(ekf, pow(1e-2, 2)); 
    setModelCovariance(ekf.sys, 
        {var_q, var_q, var_q, var_q, var_w, var_w, var_w,                   // q, w
         var_p, var_p, var_p, var_v, var_v, var_v, var_a, var_a, var_a, // p, v, a
         var_wb, var_wb, var_wb
#if SSEKF_ENABLE_ACC_BIAS()
         ,var_ab, var_ab, var_ab
#endif
         });                    // ab wb
    // auto var_zp = pow(params_.measurement_noise_p,  2);
    // setModelCovariance(ekf.pmm,  {var_zp, var_zp, 5 * var_zp});
    setModelCovariance(ekf.pmm,  pow(params_.measurement_noise_p,  2));
    setModelCovariance(ekf.vmm,  pow(params_.measurement_noise_v,  2));
    setModelCovariance(ekf.amm,  pow(params_.measurement_noise_a,  2));
    setModelCovariance(ekf.gmm,  pow(params_.measurement_noise_g,  2));
    setModelCovariance(ekf.qmm,  pow(params_.measurement_noise_q,  2));
    setModelCovariance(ekf.ymm,  pow(params_.measurement_noise_y,  2));
    setModelCovariance(ekf.vamm, pow(params_.measurement_noise_va, 2));
  }

  // Initialize publishers
  pub_fused_pose_marker_array_ = private_nh_.advertise<visualization_msgs::MarkerArray>("fused_pose_marker_array", 1);
  fused_pose_marker_array_msg_.markers.resize(2);
  for (auto lr : {LEFT, RIGHT})
  {
    auto& marker_msg = fused_pose_marker_array_msg_.markers[lr];
    marker_msg.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.lifetime = ros::Duration(1000);
    marker_msg.mesh_resource = "package://gait_training_robot/meshes/shoe.stl";
    std::string str_lr = (lr == LEFT ? "l" : "r");
    pub_fused_poses_[lr] = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("fused_pose_" + str_lr, 600);
    pub_raw_poses_[lr] = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("raw_pose_" + str_lr, 5);
    pub_ekf_state_[lr] = private_nh_.advertise<gait_training_robot::SportSoleEkfState>("state_" + str_lr, 5);
    pub_ekf_sport_sole_measurement_[lr] = private_nh_.advertise<gait_training_robot::SportSoleEkfSportSoleMeasurement>("sport_sole_measurement_" + str_lr, 20);
    pub_ekf_kinect_measurement_[lr] = private_nh_.advertise<gait_training_robot::SportSoleEkfKinectMeasurement>("kinect_measurement_" + str_lr, 5);
  }

  msg_sport_sole_measurement_[LEFT].header.frame_id = "sport_sole_left";
  msg_sport_sole_measurement_[RIGHT].header.frame_id = "sport_sole_right";
}

void FootPoseEstimator::skeletonsCB(const visualization_msgs::MarkerArray& msg)
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

  static int tf_cnt;

  // Process the closest body
  if (it_pelvis_closest != msg.markers.end())
  {
    ++tf_cnt;
    // Get the current timestamp of the skeleton message
    auto stamp_skeleton_curr = it_pelvis_closest->header.stamp;

    // Find the transform to global frame
    tf2::Transform tf_depth_to_global;
    try
    {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg = tf_buffer_.lookupTransform(params_.global_frame, "depth_camera_link", stamp_skeleton_curr, ros::Duration(0.2));
      tf2::fromMsg(tf_msg.transform, tf_depth_to_global);
      // std::cout << "[" << stamp_skeleton_curr << "]: " << params_.global_frame << ": " << tf_msg.transform.rotation << std::endl;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    
    // Cache and broadcast tfs.
    for (auto lr: {LEFT, RIGHT})
    {
      const auto & it_ankle = it_pelvis_closest + (lr == LEFT ? K4ABT_JOINT_ANKLE_LEFT : K4ABT_JOINT_ANKLE_RIGHT);

      geometry_msgs::TransformStampedPtr tf_joint_ptr(new geometry_msgs::TransformStamped);
      tf_joint_ptr->header.stamp = stamp_skeleton_curr;
      tf_joint_ptr->header.frame_id = params_.global_frame;
      
      // Orientation
      tf2::Quaternion quat;
      tf2::fromMsg(it_ankle->pose.orientation, quat);
      switch (lr)
      {
        case LEFT: 
          tf_joint_ptr->child_frame_id = "sport_sole_left";
          quat *= tf2::Quaternion({1.0, 1.0, 1.0}, M_PI / 3 * 2) ;
          break;
        case RIGHT: 
          tf_joint_ptr->child_frame_id = "sport_sole_right";
          quat *= tf2::Quaternion({-1.0, 1.0, 1.0}, -M_PI / 3 * 2) ;
          break;
      }
      quat = tf_depth_to_global * quat;
      tf_joint_ptr->transform.rotation = tf2::toMsg(quat);

      // Position
      tf2::Vector3 position;
      tf2::fromMsg(it_ankle->pose.position, position);
      position = tf_depth_to_global * position;
      // The displacement of the sport sole relative to the ankle joint expressed in the ankle joint frame
      auto disp_imu = tf2::quatRotate(quat, {0.1, 0.0, -0.1});
      position += disp_imu;
      tf_joint_ptr->transform.translation = tf2::toMsg(position);
      cache_kinect_measurements_[lr].add(tf_joint_ptr);

      // Initialization of ekf
      auto& ekf = ekf_[lr];
      if (!ekf.initialized && !ts_sport_sole_last_.isZero())
      {
        ekf.x.px() = tf_joint_ptr->transform.translation.x;
        ekf.x.py() = tf_joint_ptr->transform.translation.y;
        ekf.x.pz() = tf_joint_ptr->transform.translation.z;
        ekf.x.q0() = tf_joint_ptr->transform.rotation.w;
        ekf.x.q1() = tf_joint_ptr->transform.rotation.x;
        ekf.x.q2() = tf_joint_ptr->transform.rotation.y;
        ekf.x.q3() = tf_joint_ptr->transform.rotation.z;
        ekf.repairQuaternion();

        ekf.initialized = true;

        ts_sport_sole_last_ = stamp_skeleton_curr;
        ts_predict_last_ = ts_kinect_last_ = stamp_skeleton_curr;
        ts_init_ = stamp_skeleton_curr;
      }

      // Publish tf message
      if (params_.relay_k4a_measurement)
      {
        tf_broadcaster_.sendTransform(*tf_joint_ptr);
      }

      // Publish pose_measured message
      if (params_.global_frame == params_.publish_frame || is_initialized_tf_global_to_publish_)
      {
        auto pose_measured = constructPoseWithCovarianceStamped(position, quat);
        pose_measured->header.stamp = stamp_skeleton_curr;
        pub_raw_poses_[lr].publish(pose_measured);
      }

    } // End of lr
    
  } // End of if body count > 0

  // std::cout << "\rBody id: " << sub_id_ << ". " << "Num of tf published: " << tf_cnt << std::flush;
}


void FootPoseEstimator::sportSoleCB(const sport_sole::SportSole& msg)
{
  sport_sole::SportSolePtr msg_new(new sport_sole::SportSole(msg));
  ros::Time ts_sport_sole_curr = msg.header.stamp + ros::Duration(params_.sport_sole_time_offset);
  msg_new->header.stamp = ts_sport_sole_curr;
  cache_sport_sole_.add(msg_new);

  // Proceed only if the state is initialized with an skeletons message.
  if (ts_kinect_last_.isZero()) 
  {
    ts_sport_sole_last_ = ts_sport_sole_curr;

    return;
  }

  static int cnt_predict, cnt_predict_prev, cnt_update;
  static int num_of_predictions_between_updates;

  // Update:
  geometry_msgs::TransformStampedConstPtr msg_kinect_ptrs[LEFT_RIGHT] = {
    cache_kinect_measurements_[LEFT].getElemAfterTime(ts_kinect_last_),
    cache_kinect_measurements_[RIGHT].getElemAfterTime(ts_kinect_last_)
  };

  if (// Update only if kinect measurement is available
      msg_kinect_ptrs[LEFT] && msg_kinect_ptrs[RIGHT]) 
  {
    // Update using sport sole measurements
    ros::Time ts_predict_until = msg_kinect_ptrs[LEFT]->header.stamp;
    if (ts_sport_sole_last_ < ts_predict_until)
    {
      int cnt_predict_local = 0;
      auto msg_ptrs = cache_sport_sole_.getInterval(ts_sport_sole_last_, ts_predict_until);
      for (auto msg_ptr : msg_ptrs)
      {
        if (msg_ptr->header.stamp <= ts_sport_sole_last_) continue;

        ++cnt_predict_local;
        sportSoleUpdate(msg_ptr);
      }
      cnt_predict += cnt_predict_local;
      // if (cnt_predict_local) std::cout << "Predict " << (ts_sport_sole_last_ - ts_init_).toSec() << " " << cnt_predict_local << " steps" << std::endl;

    }

    // Get previous Kinect message
    geometry_msgs::TransformStampedConstPtr prev_msg_kinect_ptrs[LEFT_RIGHT] = {
      cache_kinect_measurements_[LEFT].getElemBeforeTime(msg_kinect_ptrs[LEFT]->header.stamp),
      cache_kinect_measurements_[RIGHT].getElemBeforeTime(msg_kinect_ptrs[RIGHT]->header.stamp)
    };

    kinectUpdate(msg_kinect_ptrs, prev_msg_kinect_ptrs);
    // std::cout << "Update  " << (msg_kinect_ptrs[LEFT]->header.stamp - ts_init_).toSec() << std::endl;
    num_of_predictions_between_updates = cnt_predict - cnt_predict_prev;
    cnt_predict_prev = cnt_predict;
    ++cnt_update;
  }

  // printf("\rPredicts: %8d, %8d. Updates: %8d.", cnt_predict, num_of_predictions_between_updates, cnt_update);
  // fflush(stdout);
}

void FootPoseEstimator::updateTf(const ros::Time& stamp)
{
  if (params_.global_frame == params_.publish_frame) return;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = tf_buffer_.lookupTransform(params_.publish_frame, params_.global_frame, stamp, ros::Duration(0.3));
    tf2::fromMsg(tf_msg.transform, tf_global_to_publish_);
    is_initialized_tf_global_to_publish_ = true;
    ROS_INFO_STREAM_ONCE("Tf to global frame [" << params_.global_frame << "] has been received." );
  }
  catch (tf2::TransformException& ex)
  {
    ROS_DEBUG_THROTTLE(5.0, "Global frame [%s] cannot be found. %s", params_.global_frame.c_str(), ex.what());
    return;
  }
}

void FootPoseEstimator::sportSoleUpdate(sport_sole::SportSoleConstPtr msg_ptr)
{ 
  double dt = (msg_ptr->header.stamp - ts_predict_last_).toSec();

  gait_phase_fsm_.update(*msg_ptr);
  uint8_t gait_state = gait_phase_fsm_.getGaitState();

  for (auto lr: {LEFT, RIGHT})
  {
    auto& msg_measurement = msg_sport_sole_measurement_[lr];
    auto& msg_state = msg_state_[lr];
    msg_measurement.header.stamp = msg_state.header.stamp = msg_ptr->header.stamp;
    msg_state.header.frame_id = params_.global_frame;

    ekf_t& ekf = ekf_[lr];
    if (dt > 1e-3)
    {
      printDebugMessage("Predicting",ts_predict_last_, lr);
      ekf.predict(dt);
      printDebugMessage("Predicted", msg_ptr->header.stamp, lr);
    }

    ekf_t::ZG zg;
    zg.x() = msg_measurement.w.x = msg_ptr->angular_velocity[lr].x;
    zg.y() = msg_measurement.w.y = -msg_ptr->angular_velocity[lr].y;
    zg.z() = msg_measurement.w.z = -msg_ptr->angular_velocity[lr].z;

    ekf_t::ZA za;
    za.x() = msg_measurement.a.x = msg_ptr->raw_acceleration[lr].linear.x;
    za.y() = msg_measurement.a.y = -msg_ptr->raw_acceleration[lr].linear.y;
    za.z() = msg_measurement.a.z = -msg_ptr->raw_acceleration[lr].linear.z;

    // Convert quaternion to euler angles
    double yaw, pitch, roll;
    tf2::Matrix3x3(tf2::Quaternion(ekf.x.q1(), ekf.x.q2(), ekf.x.q3(), ekf.x.q0())).getEulerYPR(yaw, pitch, roll);

    // Update gait phase
    previous_gait_phase_[lr] = current_gait_phase_[lr];
    current_gait_phase_[lr] = getGaitPhaseLR(gait_state, lr);

    // Check sticky pressure sensor issue
    if (current_gait_phase_[lr] != GaitPhase::Swing)
    {
      bool stickyHindfoot = false;
      bool stickyForefoot = false;
      if (pitch > 0.5 && zg.y() > 1.0 && isHindfootTouchingGround(current_gait_phase_[lr]) )
      {
        stickyHindfoot = true;
        current_gait_phase_[lr] = GaitPhase::Stance3;
      }
      else if (pitch < -0.3 && zg.y() > 1.0 && isForefootTouchingGround(current_gait_phase_[lr]))
      {
        stickyForefoot = true;
        current_gait_phase_[lr] = GaitPhase::Stance1;
      }
      else if (zg.y() < -1.0)
      {
        stickyForefoot = stickyHindfoot = true;
        current_gait_phase_[lr] = GaitPhase::Swing;
      }

      std::string str_lr = (lr == LEFT ? "L" : "R");
      if (stickyHindfoot)
      {
        ++incident_counter_[lr].StickyHindfootPressureSensor;
        ROS_WARN_STREAM_THROTTLE(5, "Sticky " << str_lr << " Hindfoot pressure sensor incidents: "
          << incident_counter_[lr].StickyHindfootPressureSensor);
      }

      if (stickyForefoot)
      {
        ++incident_counter_[lr].StickyForefootPressureSensor;
        ROS_WARN_STREAM_THROTTLE(5, "Sticky " << str_lr << " Forefoot pressure sensor incidents: " 
          << incident_counter_[lr].StickyForefootPressureSensor);
      }
    }

    // Accel update
    if (current_gait_phase_[lr] == GaitPhase::Stance2)
      setModelCovariance(ekf.amm,  pow(params_.measurement_noise_a / 10,  2));
    else
      setModelCovariance(ekf.amm,  pow(params_.measurement_noise_a,  2));
    ekf.update(za);
    printDebugMessage("Accel update", msg_ptr->header.stamp, lr);

    // Gyro update
    ekf.update(zg);
    printDebugMessage("Gyro update", msg_ptr->header.stamp, lr);

    // Zero velocity update
    ekf_t::ZV zv;
    zv << NAN, NAN, NAN;
    if (current_gait_phase_[lr] != GaitPhase::Swing)
    // if (current_gait_phase_[lr] == GaitPhase::Stance2)
    {
      if (previous_gait_phase_[lr] == GaitPhase::Swing) 
      {
        ekf.reducePVA();
      }

      if (current_gait_phase_[lr] == GaitPhase::Stance2)
      {
        // foot-flat phase
        confidence_pos_a_priori_[lr] += params_.confidence_pos_a_priori_alpha * (1.0 - confidence_pos_a_priori_[lr]);
        ekf_t::ZVA zva;
        zva.setZero();
        zv.setZero();
        ekf.update(zva);
        printDebugMessage("Zero VA update", msg_ptr->header.stamp, lr);
        zg_y_max_[lr] = 0.;
      }
      else if (current_gait_phase_[lr] == GaitPhase::Stance1)
      {
        // First contact
        tf2::Quaternion quat(ekf.x.q1(), ekf.x.q2(), ekf.x.q3(), ekf.x.q0());
        tf2::Vector3 r{0.1, 0., 0.};
        tf2::Vector3 omega{ekf.x.wx(), ekf.x.wy(), ekf.x.wz()};
        tf2::Vector3 v = tf2::quatRotate(quat, omega.cross(r));
        zv << v.x(), v.y(), v.z();
        ekf.update(zv);
        printDebugMessage("First contact velocity update", msg_ptr->header.stamp, lr);
      }
      else if (current_gait_phase_[lr] == GaitPhase::Stance3 && zg.y() > zg_y_max_[lr])
      {
        // Last contact
        zg_y_max_[lr] = zg.y();
        tf2::Quaternion quat(ekf.x.q1(), ekf.x.q2(), ekf.x.q3(), ekf.x.q0());
        tf2::Vector3 r{-0.1, 0., 0.};
        tf2::Vector3 omega{ekf.x.wx(), ekf.x.wy(), ekf.x.wz()};
        tf2::Vector3 v = tf2::quatRotate(quat, omega.cross(r));
        zv << v.x(), v.y(), v.z();
        ekf.update(zv);
        printDebugMessage("Last contact velocity updatee", msg_ptr->header.stamp, lr);
      }
      ekf.reducePVA();
    }
    else
    {
      confidence_pos_a_priori_[lr] = params_.robustness_factor_swing;
    }

    // Publish sport_sole measurement message
    if (pub_ekf_sport_sole_measurement_[lr].getNumSubscribers())
    {
      assignVector3(msg_measurement.v, zv);
      pub_ekf_sport_sole_measurement_[lr].publish(msg_measurement);
    }

    // Publish a priori state estimate
    if (pub_ekf_state_[lr].getNumSubscribers())
    {
      msg_state.q.w = ekf.x.q0();
      msg_state.q.x = ekf.x.q1();
      msg_state.q.y = ekf.x.q2();
      msg_state.q.z = ekf.x.q3();
      msg_state.rpy.x = roll;
      msg_state.rpy.y = pitch;
      msg_state.rpy.z = yaw;
      assignVector3(msg_state.w, ekf.x.w());
      assignVector3(msg_state.p, ekf.x.p());
      assignVector3(msg_state.v, ekf.x.v());
      assignVector3(msg_state.a, ekf.x.a());
      assignVector3(msg_state.ab, ekf.x.ab());
      assignVector3(msg_state.wb, ekf.x.wb());
      pub_ekf_state_[lr].publish(msg_state);
    }
  }
  ts_predict_last_ = ts_sport_sole_last_ = msg_ptr->header.stamp;

  publishFusedPoses(ts_sport_sole_last_);
}

void FootPoseEstimator::kinectUpdate(geometry_msgs::TransformStampedConstPtr msg_ptrs[LEFT_RIGHT],
                                     geometry_msgs::TransformStampedConstPtr prev_msg_ptrs[LEFT_RIGHT])
{
  double dt = (msg_ptrs[LEFT]->header.stamp - ts_predict_last_).toSec();
  for (auto lr : {LEFT, RIGHT})
  {
    auto& msg_measurement = msg_kinect_measurement_[lr];
    auto& msg_state = msg_state_[lr];
    msg_measurement.header.frame_id = msg_state.header.frame_id = params_.global_frame;

    ekf_t& ekf = ekf_[lr];
    auto msg_kinect_ptr = msg_ptrs[lr];
    msg_measurement.header.stamp = msg_state.header.stamp = msg_kinect_ptr->header.stamp;
    printDebugMessage("Updating", msg_kinect_ptr->header.stamp, lr);

    auto prev_msg_kinect_ptr = prev_msg_ptrs[lr];
    ekf_t::ZV zv;
    bool zv_ready = false;
    if (prev_msg_kinect_ptr) //(prev_msg_kinect_ptr)
    {
      double dt = (msg_kinect_ptr->header.stamp - prev_msg_kinect_ptr->header.stamp).toSec();
      if (dt > 1e-3)
      {
        zv_ready = true;
        zv.x() = msg_measurement.v.x = (msg_kinect_ptr->transform.translation.x - prev_msg_kinect_ptr->transform.translation.x) / dt;
        zv.y() = msg_measurement.v.y = (msg_kinect_ptr->transform.translation.y - prev_msg_kinect_ptr->transform.translation.y) / dt;
        zv.z() = msg_measurement.v.z = (msg_kinect_ptr->transform.translation.z - prev_msg_kinect_ptr->transform.translation.z) / dt;
        
        if (0 && current_gait_phase_[lr] == GaitPhase::Swing)
        {
          ekf.update(zv);
          printDebugMessage("Velocity update", msg_kinect_ptr->header.stamp, lr);
        }
      }
    }

    if (dt > 1e-3)
    {
      printDebugMessage("Predicting",ts_predict_last_, lr);
      ekf.predict(dt);
      printDebugMessage("Predicted", msg_state.header.stamp, lr);
    }

    // Position update
    ekf_t::ZP zp;
    zp.x() = msg_kinect_ptr->transform.translation.x;
    zp.y() = msg_kinect_ptr->transform.translation.y;
    zp.z() = msg_kinect_ptr->transform.translation.z;
    if(current_gait_phase_[lr] == GaitPhase::Stance2 && zv.norm() > 0.2 && (zp - ekf.x.p()).norm() > 0.2)
    {
      ROS_DEBUG("Kinect measurement outlier detected. Not updating!");
    }
    else
    {
      if (params_.enable_huber_update)
      {
        if (current_gait_phase_[lr] != GaitPhase::Stance2)
        {
          FloatType gamma = confidence_pos_a_priori_[lr];
          FloatType outlier_threshold = params_.system_noise_p * params_.outlier_threashold;
          auto psi = [gamma, outlier_threshold](FloatType zeta){
            zeta = fabs(zeta);
            if (zeta < outlier_threshold) return FloatType(0);
            else return gamma * (zeta - outlier_threshold);
          };
          ekf.update(zp, psi);
          spme_[lr].clear();
        }
        else
        {
          // Initialize spme with prior estimate
          if (spme_[lr].empty()) spme_[lr].put(ekf.x.p());
          spme_[lr].put(zp);
          ekf_t::ZP zp_new = spme_[lr].getAverage();
          ekf.update(zp_new);
        }
      }
      else 
      {
        FloatType gamma = params_.outlier_threashold;
        auto psi = [gamma](FloatType zeta){
          if (fabs(zeta) < gamma)
          {
            return FloatType(1.0);
          }
          else
          {
            return gamma / fabs(zeta);
          }
        };
        setModelCovariance(ekf.pmm,  pow(params_.measurement_noise_p,  2));
        if (params_.enable_debug_log)
          ekf.updateHuber(zp, psi, &ofs_log[lr]);
        else
          ekf.updateHuber(zp, psi);
      }
      printDebugMessage("Position update", msg_kinect_ptr->header.stamp, lr);
    }
    
    // Special update
#if 1
    double MU = 0.04;
    auto zy_correction = MU * (zp - ekf.x.p()).cross(ekf.x.v());
#else
    double MU = 6.0;
    auto zy_correction = MU * (zv - ekf.x.v()).cross(ekf.x.v());
#endif
    refUnitY[lr] = (refUnitY[lr] - refUnitY[lr].cross({zy_correction.x(), zy_correction.y(), zy_correction.z()})).normalize();

    // Quaternion update (relative)
    if (ekf.x.v().hypotNorm() > 0.3)
    {
      double MU2 = 2.0;
#if 1
      tf2::Quaternion delta_quat({0, 0, 1}, zy_correction.z() * MU2);
#else
      auto dir_corr = zy_correction.normalized();
      tf2::Quaternion delta_quat({dir_corr.x(), dir_corr.y(), dir_corr.z()}, zy_correction.norm() * MU2);
#endif
      tf2::Quaternion quat_estimate(ekf.x.q1(), ekf.x.q2(), ekf.x.q3(), ekf.x.q0());
      quat_estimate = quat_estimate * delta_quat;
      ekf_t::ZQ zq;
      zq << quat_estimate.w(), quat_estimate.x(), quat_estimate.y(), quat_estimate.z();
      ekf.update(zq);
      ts_last_quaternion_update_ = msg_kinect_ptr->header.stamp;
      printDebugMessage("Quaternion update", msg_kinect_ptr->header.stamp, lr);
    }

    tf2::Quaternion quat;
    tf2::fromMsg(msg_kinect_ptr->transform.rotation, quat);
    tf2::Vector3 zy_ros = tf2::quatRotate(quat.inverse(), refUnitY[lr]);
    
    // Yaw update
    if ((msg_kinect_ptr->header.stamp - ts_last_quaternion_update_).toSec() > 2.0)
    {
      ekf_t::ZY zy;
      zy << zy_ros.x(), zy_ros.y(), zy_ros.z();
      if (current_gait_phase_[lr] == GaitPhase::Stance2)
        ekf.update(zy);
      printDebugMessage("Yaw update", msg_kinect_ptr->header.stamp, lr);
    }
    
    // Publish kinect_measurement message
    if (pub_ekf_kinect_measurement_[lr].getNumSubscribers())
    {
      msg_measurement.p = msg_kinect_ptr->transform.translation;
      msg_measurement.y = tf2::toMsg(zy_ros);
      msg_measurement.refUnitY = tf2::toMsg(refUnitY[lr]);
      pub_ekf_kinect_measurement_[lr].publish(msg_measurement);
    }

    // Publishe posteriori state estimate
    if (pub_ekf_state_[lr].getNumSubscribers())
    {
      msg_state.q.w = ekf.x.q0();
      msg_state.q.x = ekf.x.q1();
      msg_state.q.y = ekf.x.q2();
      msg_state.q.z = ekf.x.q3();

      // Convert quaternion to euler angles
      double yaw, pitch, roll;
      tf2::Matrix3x3(tf2::Quaternion(ekf.x.q1(), ekf.x.q2(), ekf.x.q3(), ekf.x.q0())).getEulerYPR(yaw, pitch, roll);
      msg_state.rpy.x = roll;
      msg_state.rpy.y = pitch;
      msg_state.rpy.z = yaw;
      assignVector3(msg_state.w, ekf.x.w());
      assignVector3(msg_state.p, ekf.x.p());
      assignVector3(msg_state.v, ekf.x.v());
      assignVector3(msg_state.a, ekf.x.a());
      assignVector3(msg_state.ab, ekf.x.ab());
      assignVector3(msg_state.wb, ekf.x.wb());
      pub_ekf_state_[lr].publish(msg_state);
    }
  }

  // Update for next iteration
  ts_predict_last_ = ts_kinect_last_ = msg_ptrs[LEFT]->header.stamp;

  // Publish pose_fused
  publishFusedPoses(ts_kinect_last_);
}

void FootPoseEstimator::publishFusedPoses(const ros::Time& stamp)
{
  updateTf(stamp);
  if (!(params_.global_frame == params_.publish_frame || is_initialized_tf_global_to_publish_))
  {
    ROS_WARN_STREAM_ONCE("Will not publish fused poses until the transform from global frame to publish frame becomes available.");
    return;
  }
  for (auto lr : {LEFT, RIGHT})
  {
    auto & ekf = ekf_[lr];
    tf2::Vector3 position(ekf.x.px(), ekf.x.py(), ekf.x.pz());
    tf2::Quaternion quat(ekf.x.q1(), ekf.x.q2(), ekf.x.q3(), ekf.x.q0());
    auto pose_fused = constructPoseWithCovarianceStamped(position, quat); 
    pose_fused->header.stamp = stamp;
    // pose_msg.pose.covariance
    for (size_t i0 = decltype(ekf.x)::PX, i = 0; i < 3; ++i)
    {
      for (size_t j0 = decltype(ekf.x)::PX, j = 0; j < 3; ++j)
      {
        pose_fused->pose.covariance[i * 6 + j] = ekf.P(i0 + i, j0 + j);
      }
    }
    pub_fused_poses_[lr].publish(pose_fused);

    // Update marker array
    auto& marker_msg = fused_pose_marker_array_msg_.markers[lr];
    marker_msg.header.stamp = stamp;
    marker_msg.header.frame_id = params_.publish_frame;
    marker_msg.id = lr;
    marker_msg.pose = pose_fused->pose.pose;
    marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = 0.00013;
    correctShoeMarkerOrientation(&marker_msg, lr);
  }
  pub_fused_pose_marker_array_.publish(fused_pose_marker_array_msg_);
}

void FootPoseEstimator::printDebugMessage(const char* message, const ros::Time& stamp, left_right_t lr) const
{
  if (!params_.enable_debug_log) return;
  if (std::strcmp("Updating", message)) return;
  const ekf_t& ekf = ekf_[lr];
  ofs_log[lr].precision(15);
  ofs_log[lr] << stamp.toSec() << " " << message << " ";
  ofs_log[lr].precision(3);
  ofs_log[lr] << "x:\n" << ekf.x.transpose() << "\n";
  ofs_log[lr] << "P:\n" << ekf.P << "\n\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_pose_estimator");
  FootPoseEstimator fpe;

  ros::spin();
  std::cout << std::endl;
  return 0;
}


geometry_msgs::PoseWithCovarianceStampedPtr FootPoseEstimator::constructPoseWithCovarianceStamped(
  tf2::Vector3 position, tf2::Quaternion quat) const
{
  geometry_msgs::PoseWithCovarianceStampedPtr msg(new geometry_msgs::PoseWithCovarianceStamped);
  if (is_initialized_tf_global_to_publish_)
  {
    msg->header.frame_id = params_.publish_frame;
    position = tf_global_to_publish_ * position;
    quat = tf_global_to_publish_ * quat;
  }
  else
  {
    msg->header.frame_id = params_.global_frame;
  }
  tf2::toMsg(position, msg->pose.pose.position);
  msg->pose.pose.orientation = tf2::toMsg(quat);

  return msg;
}