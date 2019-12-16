//Purpose: bring together data from  1) accel, quaternion, pressures of left & right sport soles  2) kinect IMU  3) kinect skeleton info

#include "gait_training_robot/sport_sole_kalman_filter.h"
#include <k4abttypes.h>

using visualization_msgs::MarkerArray;
using visualization_msgs::MarkerArrayPtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerPtr;

// Class SportSoleKalmanFilterParams
void SportSoleKalmanFilterParams::print()
{
    #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)         \
        ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

        ROS_PARAM_LIST
    #undef LIST_ENTRY
}



// Class SportSoleEKF
SportSoleEKF::SportSoleEKF()
{
    // Set initial state
    x.setZero();
}

const State& SportSoleEKF::predict(const Control & u, T dt)
{
    if (dt > T(0.0))
        sys.setSamplingPeriod(dt);
    return ExtendedKalmanFilter::predict(sys, u);
}

const State& SportSoleEKF::update(const PositionMeasurement& z)
{
    return ExtendedKalmanFilter::update(pm, z);
}

const State& SportSoleEKF::update(const OrientationMeasurement& z)
{
    return ExtendedKalmanFilter::update(om, z);
}

const State& SportSoleEKF::update(const VelocityMeasurement& z)
{
    return ExtendedKalmanFilter::update(vm, z);
}

SystemModel SportSoleEKF::sys;
PositionModel SportSoleEKF::pm;
OrientationModel SportSoleEKF::om;
VelocityModel SportSoleEKF::vm;



// Class SportSoleKalmanFilterNode
SportSoleKalmanFilterNode::SportSoleKalmanFilterNode(const ros::NodeHandle& n, const ros::NodeHandle& p):
    nh_(n),
    private_nh_(p),
    sub_sport_sole_(nh_, "/sport_sole_publisher/sport_sole", 1),
    cache_sport_sole_(sub_sport_sole_, 100),
    tf_listener_(tf_buffer_)
{
    // Collect ROS parameters from the param server or from the command line
    #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
        private_nh_.param(#param_variable, params_.param_variable, param_default_val);

        ROS_PARAM_LIST
    #undef LIST_ENTRY

    // Print all parameters
    ROS_INFO("SportSoleKalmanFilter Parameters:");
    params_.print();

    // Initialize the the static members of SportSoleEKF
    SportSoleEKF::sys.setSamplingPeriod(params_.sampling_period);
    const auto & xpstd = params_.system_noise_p;
    const auto & xvstd = params_.system_noise_v;
    const auto & xthstd = params_.system_noise_th;
    setModelCovariance(SportSoleEKF::sys, 
        {xpstd, xpstd, xpstd, xvstd, xvstd, xvstd, xthstd, xthstd, xthstd});

    const auto & zpstd = params_.measurement_noise_p;
    setModelCovariance(SportSoleEKF::pm, {zpstd, zpstd, zpstd});
    const float zvstd = params_.measurement_noise_v;
    setModelCovariance(SportSoleEKF::vm, {zvstd, zvstd, zvstd});
    const auto & zthstd = params_.measurement_noise_th;
    setModelCovariance(SportSoleEKF::om, {zthstd, zthstd, zthstd});
    
    // Initialize the subscribers
    //sub_sport_sole_ = nh_.subscribe("sport_sole", 1000, &SportSoleKalmanFilterNode::sportSoleCB, this);
    sub_skeletons_ = nh_.subscribe("/body_tracking_data", 5, &SportSoleKalmanFilterNode::skeletonsCB, this );
    
    // Initialize the publishers
    pub_estimates_ = private_nh_.advertise<MarkerArray>("estimates", 1);

    // Open the files for data logging
    std::string strDate;// = generateDateString();
    const char * home_path = getenv("HOME");
    if (home_path)
    {
        for (auto lr : {LEFT, RIGHT})
        {
            auto str_lr = std::string(!lr?"l_": "r_");
            auto str_prefix = home_path + std::string("/log/");
            auto str_suffix = std::string(".csv");

            auto str_full_path = str_prefix + "data_process_" + str_lr + strDate + str_suffix;
            ofs_process_[lr].open(str_full_path);
            if (ofs_process_[lr].is_open() == false) ROS_ERROR_STREAM("Failed to open " << str_full_path);

            str_full_path = str_prefix + "data_measurement_" + str_lr + strDate + str_suffix;
            ofs_measurement_[lr].open(str_full_path);
            if (ofs_measurement_[lr].is_open() == false) ROS_ERROR_STREAM("Failed to open " << str_full_path);
        }
    }
    else
    {
        ROS_ERROR("Cannot get enviroment variable $HOME.");
    }
}

void SportSoleKalmanFilterNode::sportSoleCB(const sport_sole::SportSole& msg)
{
    Control u;


    for (auto lr : {LEFT, RIGHT})
    {
        const size_t offset = (lr == LEFT) ? 2 : 0;
        const uint8_t mask = 3 << offset;
        bool is_foot_flat = (msg.gait_state & mask) == mask;
        
        // Function to log process data
        auto logProcessData = [&u, &msg, this, &lr](const State & x, const State & x_pred){
            // Data logging
            ofs_process_[lr] << msg.header.stamp << " "                     // 1; timestamp
                << x.transpose().format(csv_format_) << " "                 // 9; state
                << x.q.vec().transpose().format(csv_format_) << " "         // 3; quaternion vec
                << u.transpose().format(csv_format_) << " "                 // 3; system input
                << x_pred.transpose().format(csv_format_)  << " "           // 9; predicted state
                << x_pred.q.vec().transpose().format(csv_format_) << "\n";   // 3; predicted quaternion vector
        };

        // Calculate system input
        const auto& acc = msg.acceleration[lr].linear;
        u(0) = acc.x;
        u(1) = acc.y;
        u(2) = acc.z;

        // Estimate or eliminate the bias
        if ((msg.header.stamp - stamp_base_).toSec() < 2.0)
        {
            acc_bias_ave_[lr].put(u);
        }
        else
        {
            u = u - acc_bias_ave_[lr].getAverage();
        }

        // Check if we are in foot flat phase
        if (is_foot_flat)
        {
            u.setZero();
            // To-do: velocity should also be set to zero.
            // Pending feature support by kalman filter
            VelocityMeasurement zv = Kalman::Vector<T, 3>::Zero();
            predictor_[lr].update(zv);
        }

        // Find time delta
        T dt;
        if(!stamp_sport_sole_prev_[lr].isZero())
            dt = (msg.header.stamp - stamp_sport_sole_prev_[lr]).toSec();
        else
        {
            dt = T(0.01);
            const auto & x = ekf_[lr].getState();
            const auto & x_pred = predictor_[lr].getState();
            logProcessData(x, x_pred);
        }
        if (dt > 0.1 || dt < 0.001)
        {
            ROS_WARN_STREAM("Weird dt: " << dt);
            dt = T(0.01);
        }
        stamp_sport_sole_prev_[lr] = msg.header.stamp;

        // Predict
        const auto & x = ekf_[lr].predict(u, dt);
        const auto & x_pred = predictor_[lr].predict(u, dt);
        logProcessData(x, x_pred);

        // Broadcast a transform for the IMU measurement of the ankle joint frames.
        geometry_msgs::TransformStamped tf_ankle;
        tf_ankle.header.stamp = msg.header.stamp;
        tf_ankle.header.frame_id = "map"; // "map"
        tf_ankle.child_frame_id = std::string("ankle_i_") + (!lr?"l": "r");
        assignVector3(tf_ankle.transform.translation, x.p());
        auto q_imu = getImuQuaternion<T>(msg.quaternion[lr]);
        assignQuaternion(tf_ankle.transform.rotation, q_imu); // * x.q
        tf_boradcaster_.sendTransform(tf_ankle);
    }
    
}

void SportSoleKalmanFilterNode::skeletonsCB(const MarkerArray& msg)
{
    double dist_min_pelvis = 100.0;
    //double idx = -1; // body id with the min dist of pelvis from camera center
    auto it_pelvis_closest = msg.markers.end();// iterator of the pelvis marker of the closest body

    // Find the closest body, K4ABT_JOINT_PELVIS = 0
    for (auto it = msg.markers.begin(); it < msg.markers.end(); it += K4ABT_JOINT_COUNT)
    {
        // The coordinates are in expressed in /depth_camera_link
        double dist_pelvis = hypot(it->pose.position.x, it->pose.position.z);
        if (dist_pelvis < dist_min_pelvis)
        {
            dist_pelvis = dist_min_pelvis;
            it_pelvis_closest = it;
            //idx = it->id / 100;
        }
    }

    // Process the closest body
    if (it_pelvis_closest != msg.markers.end())
    {
        // Get the current timestamp of the skeleton message
        const auto & stamp_skeleton_curr = it_pelvis_closest->header.stamp;

        try
        {
            // Find the tf from map frame to depth_camera_link frame
            geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform("map", "depth_camera_link", stamp_skeleton_curr);
            fromMsg(tf_msg.transform, tf_depth_to_map_);
            
        }
        catch (tf2::TransformException & ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
            //ros::Duration(1.0).sleep();
        }

        // Broadcast some random tfs.
        for (auto joint_id: {K4ABT_JOINT_PELVIS})
        {
            const auto & it_joint = it_pelvis_closest + joint_id;

            geometry_msgs::TransformStamped tf_joint;
            tf_joint.header.stamp = stamp_skeleton_curr;
            tf_joint.header.frame_id = "depth_camera_link"; // "map", "depth_camera_link"
            tf_joint.child_frame_id = std::string("joint_") + std::to_string(joint_id);
            tf_joint.transform.translation.x = it_joint->pose.position.x;
            tf_joint.transform.translation.y = it_joint->pose.position.y;
            tf_joint.transform.translation.z = it_joint->pose.position.z;
            tf_joint.transform.rotation.w = it_joint->pose.orientation.w;
            tf_joint.transform.rotation.x = it_joint->pose.orientation.x;
            tf_joint.transform.rotation.y = it_joint->pose.orientation.y;
            tf_joint.transform.rotation.z = it_joint->pose.orientation.z;
            tf_boradcaster_.sendTransform(tf_joint);
        }

        // // Find the distance between the ankle and foot joints.
        // auto getDistance = [&it_pelvis_closest](k4abt_joint_id_t joint1, k4abt_joint_id_t joint2)->double{
        //     auto & p1 = (it_pelvis_closest + joint1) -> pose.position;
        //     auto & p2 = (it_pelvis_closest + joint2) -> pose.position;
        //     return pow(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0) + pow(p1.z - p2.z, 2.0), 0.5);
        // };
        // ROS_INFO_STREAM("Foot-ankle distance: " << getDistance(K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_FOOT_LEFT)
        //     << ", " << getDistance(K4ABT_JOINT_ANKLE_RIGHT, K4ABT_JOINT_FOOT_RIGHT));

        // If at least one skeleton message has been received after the receipt of the first sport_sole message, do an EKF prediction
        if (!stamp_skeleton_prev_.isZero())
        {
            auto sport_sole_ptrs = cache_sport_sole_.getInterval(
                stamp_skeleton_prev_, stamp_skeleton_curr);
            for (const auto & ss_ptr : sport_sole_ptrs)
            {
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
        auto sport_sole_ptr = cache_sport_sole_.getElemBeforeTime(stamp_skeleton_curr);

        // At least one sport_sole message should have been received before we can initialize the Kalman filter
        if (!sport_sole_ptr)
        {
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
            // Get ankle measurements and do an EKF initialization/update
            MarkerArrayPtr markerArrayPtr(new MarkerArray);
            const size_t ankle_indices[] = {K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_ANKLE_RIGHT};
            for (size_t lr : {LEFT, RIGHT})
            {
                const auto & it_ankle = it_pelvis_closest + ankle_indices[lr];

                // Get position measurement
                tf2::Vector3 p_ankle;
                fromMsg(it_ankle->pose.position, p_ankle);
                p_ankle = tf_depth_to_map_ * p_ankle;
                PositionMeasurement zp;
                zp << p_ankle.getX(), p_ankle.getY(), p_ankle.getZ();
                
                // Prepare to get orientation measurement
                tf2::Quaternion q_ankle_tf;
                fromMsg(it_ankle->pose.orientation, q_ankle_tf);
                q_ankle_tf = tf_depth_to_map_ * q_ankle_tf;
                if (lr == RIGHT) 
                    q_ankle_tf = q_ankle_tf * tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), M_PI) * 
                        tf2::Quaternion(tf2::Vector3(0.577, 0.577, 0.577), 2 * M_PI / 3);
                if (lr == LEFT)
                    q_ankle_tf = q_ankle_tf * tf2::Quaternion(tf2::Vector3(0.577, 0.577, 0.577), 2 * M_PI / 3);
                
                Eigen::Quaternion<T> q_ankle(q_ankle_tf.w(), q_ankle_tf.x(), q_ankle_tf.y(), q_ankle_tf.z());

                // Broadcast a transform for the Kinect measurement of the ankle joint frames.
                geometry_msgs::TransformStamped tf_ankle;
                tf_ankle.header.stamp = stamp_skeleton_curr;
                tf_ankle.header.frame_id = "map"; // "map", "depth_camera_link"
                tf_ankle.child_frame_id = std::string("ankle_k_") + (!lr?"l": "r");
                tf_ankle.transform.translation = toMsg(p_ankle);
                tf_ankle.transform.rotation = toMsg(q_ankle_tf);
                //tf_boradcaster_.sendTransform(tf_ankle);
                
                // Obtain the orientation of the ankles as measured by the IMU
                Eigen::Quaternion<T> q_imu = getImuQuaternion<T>(sport_sole_ptr->quaternion[lr]);
                //ROS_INFO_STREAM_THROTTLE(1, (!lr?"Left  ": "Right ") << "q_imu is: " << q_imu.vec().transpose() << "; " << q_imu.w());
                
                // Calculate the quaternion state measurement
                auto q_state = q_imu.inverse() * q_ankle;
                
                // Is this the first skeleton message received after the receipt of the first sport_sole message?
                if (stamp_skeleton_prev_.isZero())
                {
                    // This is the first time skeleton message received
                    // Use the position measurement to initialize the state
                    State x0;
                    x0.setZero();
                    x0.template segment<3>(0) = zp; 
                    x0.q = q_state;
                    ekf_[lr].init(x0);
                    predictor_[lr].init(x0);
                    ROS_INFO_STREAM((!lr?"Left  ": "Right ") << "EKF state initialized to " 
                        << x0.template segment<6>(0).transpose() << ";    " << x0.q.vec().transpose());
                    stamp_sport_sole_prev_[lr] = stamp_skeleton_curr;
                }
                else
                {
                    // Update state with position measurement
                    const auto & x = ekf_[lr].update(zp);
                    
                    // Update state with orientation measurement
                    OrientationMeasurement zth; // Define the measurement of the error quaternion state.
                    zth = (q_state * x.q.inverse()).vec();
                    //ekf_[lr].update(zth);
                    ROS_INFO_STREAM_THROTTLE(1, (!lr?"Left  ": "Right ") << "EKF state updated to " 
                        << x.template segment<6>(0).transpose() << ";    " << x.q.vec().transpose());

                    // // Update when foot is in contact with the ground
                    // auto gs = sport_sole_ptr->gait_state;
                    // if ( lr == LEFT && ((gs & (1<<3)) || (gs & (1<<2))))
                    // {
                    //     ekf_[lr].update
                    // }


                    // Construct the visualization message 
                    auto marker_ptr = constructMarkerMsg(x, q_imu, it_ankle->header.stamp);
                    marker_ptr->id = lr;
                    markerArrayPtr->markers.push_back(*marker_ptr);

                }

            // Data logging
            ofs_measurement_[lr] << stamp_sport_sole_prev_[lr] << " "   // 1; timestamp
                << zp.transpose().format(csv_format_) << "\n";          // 3; position measurement
                

            }
            pub_estimates_.publish(markerArrayPtr);
            stamp_skeleton_prev_ = it_pelvis_closest->header.stamp;
        }
    }
}

MarkerPtr SportSoleKalmanFilterNode::constructMarkerMsg(const State& x,const Eigen::Quaternion<T>& q_imu, const ros::Time& stamp)
{
    MarkerPtr marker_ptr(new Marker);
    marker_ptr->header.stamp = stamp;
    marker_ptr->header.frame_id = "map";
    marker_ptr->lifetime = ros::Duration(0.13);
    marker_ptr->ns = nh_.getNamespace();
    marker_ptr->type = Marker::CUBE;

    assignVector3(marker_ptr->pose.position, x.p());

    // 
    auto q = x.q * q_imu;
    assignQuaternion(marker_ptr->pose.orientation, q);

    marker_ptr->scale.x = 0.3;
    marker_ptr->scale.y = 0.1;
    marker_ptr->scale.z = 0.05;

    marker_ptr->color.a = 1.0;
    marker_ptr->color.r = 1.0;
    marker_ptr->color.g = 1.0;
    marker_ptr->color.b = 0.0;

    return marker_ptr;
}







int main(int argc, char **argv)
{
    ros::init(argc, argv, "sport_sole_kalman_filter");
    SportSoleKalmanFilterNode node;
 
    //create subscriber to collect data from sport soles, IMU, skeleton
    
    /*    ros::Subscriber kinect_IMU_sub = node_handle.subscribe<sensor_msgs::Imu>("kinect_azure_imu", 1000, listenerCallback);  
 */
    
    ros::spin();

    ROS_INFO("ROS Exit Started");
    
    ros::shutdown();


    //create publishers here

}