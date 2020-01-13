//Purpose: bring together data from  1) accel, quaternion, pressures of left & right sport soles  2) kinect IMU  3) kinect skeleton info

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "gait_training_robot/gait_analyzer.h"

using namespace visualization_msgs;

/* #include "sport_sole/SportSole.h"

void listenerCallback (const sport_sole::SportSole& msg)
{
    //TODO: Kalman filter??
    //process the subcribed data here
}

void sportsole_callback(const sport_sole::SportSole::ConstPtr &msg, const left_right_t left_right) {
    ; // Do stuff
}
*/

GaitAnalyzer::GaitAnalyzer():
    com_model_(COM_MODEL_14_SEGMENT),
    gait_phase_{GAIT_PHASE_UNKNOWN, GAIT_PHASE_UNKNOWN},
    touches_ground_{
        {true, true}, // Left ankle, right ankle
        {true, true} // Left foot, right foot
        },
    z_ground_(0.0),
    nh_("~"),
    sub_sport_sole_(nh_, "/sport_sole_publisher/sport_sole", 1),
    cache_sport_sole_(sub_sport_sole_, 100),
    tf_listener_(tf_buffer_)
{
    // Collect ROS parameters from the param server or from the command line
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
    nh_.param(#param_variable, param_variable, param_default_val);\
    ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

    ROS_PARAM_LIST
#undef LIST_ENTRY

    sub_skeletons_ = nh_.subscribe("/body_tracking_data", 5, &GaitAnalyzer::skeletonsCB, this );
    // Do not register call back for now.
    //cache_gait_state_.registerCallback(&GaitAnalyzer::gaitStateCB, this );

    pub_pcom_ = nh_.advertise<geometry_msgs::PointStamped>("pcom", 1);
    pub_xcom_ = nh_.advertise<geometry_msgs::PointStamped>("xcom", 1);
    pub_bos_ = nh_.advertise<geometry_msgs::PolygonStamped>("bos", 1);
    pub_mos_ = nh_.advertise<visualization_msgs::MarkerArray>("mos", 1);
    for (int i = 0; i < mos_t::mos_count; ++i)
        pub_mos_values_[i] = nh_.advertise<std_msgs::Float64>("mos_values" + std::to_string(i), 1);

    pub_ground_clearance_left_ = nh_.advertise<std_msgs::Float64>("ground_clearance_left", 1);
    pub_ground_clearance_right_ = nh_.advertise<std_msgs::Float64>("ground_clearance_right", 1);
}

void GaitAnalyzer::skeletonsCB(const visualization_msgs::MarkerArray& msg)
{
    if (ros::Time::now() - cache_sport_sole_.getLatestTime() < ros::Duration(1.0))
    {
        auto msg_sport_sole_ptr = cache_sport_sole_.getElemBeforeTime(msg.markers[0].header.stamp);
        updateGaitState(msg_sport_sole_ptr->gait_state);
    }
    
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
        try
        {
            // Find the tf from global frame to depth_camera_link frame
            geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(global_frame, "depth_camera_link", ros::Time(0));
            fromMsg(tf_msg.transform, tf_depth_to_global_);
            
        }
        catch (tf2::TransformException & ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
            //ros::Duration(1.0).sleep();
        }
        
        for (int i = 0; i < K4ABT_JOINT_COUNT; i++)
        {
            // Convert messages to tf2::vectors for easier calculation
            fromMsg((it_pelvis_closest + i)->pose.position, vec_joints_[i]);
            vec_joints_[i] = tf_depth_to_global_ * vec_joints_[i];                
        }
        double z_min = std::min(vec_joints_[K4ABT_JOINT_FOOT_LEFT].getZ(), vec_joints_[K4ABT_JOINT_FOOT_RIGHT].getZ());
        z_ground_ = 0.95 * std::min(z_ground_, z_min) + 0.05 * z_min;

        // Update gait phase
        //updateGaitPhase();

        // Calculate pcom (projected center of mass)
        com_t com = getCoM();

        geometry_msgs::PointStamped msg_pcom;
        msg_pcom.header.frame_id = global_frame;
        msg_pcom.header.stamp = it_pelvis_closest->header.stamp;
        msg_pcom.point.x = com.getX();
        msg_pcom.point.y = com.getY();
        msg_pcom.point.z = z_ground_;
        pub_pcom_.publish(msg_pcom);

        // Calculate CoMv (projected)
        comv_t comv = getCoMv(com, it_pelvis_closest->header.stamp);
        static double omega0 = sqrt(9.8 / (com.getZ() - z_ground_));
        com_t xcom = com + comv / omega0;

        geometry_msgs::PointStamped msg_xcom;
        msg_xcom.header.frame_id = global_frame;
        msg_xcom.header.stamp = it_pelvis_closest->header.stamp;
        msg_xcom.point.x = xcom.getX();
        msg_xcom.point.y = xcom.getY();
        msg_xcom.point.z = z_ground_;
        pub_xcom_.publish(msg_xcom);

        // Calculate the base of support polygon
        bos_t bos_points = getBoS();

        geometry_msgs::PolygonStamped msg_bos;
        msg_bos.header.stamp = it_pelvis_closest->header.stamp;
        msg_bos.header.frame_id = global_frame;
        for (const auto & point : bos_points)
            msg_bos.polygon.points.push_back(vector3ToPoint32(point));
        
        pub_bos_.publish(msg_bos);

        
        // Calculate margin of stability
        auto mos = getMoS(bos_points, xcom);       

        MarkerArrayPtr markerArrayPtr(new MarkerArray);
        for (size_t i = 0; i < mos_t::mos_count; ++i)
        //for (size_t i = 0; i < 1; ++i)        
        {
            MarkerPtr markerPtr(new Marker);
            markerPtr->header.stamp = it_pelvis_closest->header.stamp;
            markerPtr->header.frame_id = global_frame;
            markerPtr->lifetime = ros::Duration(0.13);
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
        pub_mos_.publish(markerArrayPtr);

        
        for (size_t i = 0; i < mos_t::mos_count; i++)
        {
            std_msgs::Float64 msg;
            msg.data = mos.values[i].dist;
            pub_mos_values_[i].publish(msg);
        }

    }
}

void GaitAnalyzer::sportSoleCB(const sport_sole::SportSole& msg)
{
    updateGaitState(msg.gait_state);
}

void GaitAnalyzer::updateGaitState(const uint8_t& gait_state)
{
    touches_ground_[ANKLE][LEFT] = gait_state & (1<<3);
    touches_ground_[FOOT][LEFT] = gait_state & (1<<2);
    touches_ground_[ANKLE][RIGHT] = gait_state & (1<<1);
    touches_ground_[FOOT][RIGHT] = gait_state & (1<<0);
}


com_t GaitAnalyzer::getCoM()
{
    
    if (com_model_ == COM_MODEL_14_SEGMENT)
    {
        com_t res;
        res.setX(vec_joints_[K4ABT_JOINT_PELVIS].getX());
        res.setY(vec_joints_[K4ABT_JOINT_PELVIS].getY());
        res.setZ(vec_joints_[K4ABT_JOINT_PELVIS].getZ());
        return res;
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
        return sum / sum_mass;
    }
    else
    {
        ROS_ERROR("Unkown CoM model!");
        ros::shutdown();
        return com_t{};
    }
    
}


comv_t GaitAnalyzer::getCoMv(const com_t & com, ros::Time ts)
{
    //belt_speed;
    static com_t com_1 = com;
    static ros::Time ts_1 = ts;
    
    comv_t res;

    res = (com - com_1) / ((ts - ts_1).toSec());
    res.setX(res.getX() - belt_speed);
    //ROS_INFO_STREAM("comv: " << res);
    
    ts_1 = ts;
    com_1 = com;

#if 0
    comv_t res;
    double t = ts.toSec();
    res.setX(comv_differentiator_x_.getDerivative(t, com.getX()) - belt_speed);
    res.setY(comv_differentiator_y_.getDerivative(t, com.getY()));
#endif

    return res;
    
}

bos_t GaitAnalyzer::getBoS()
{
    bos_t bos_points;
    bos_t res;
    

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

    for (int i = 0; i < LEFT_RIGHT; i++)
    {
        v0[i].setZ(0);
        v0[i] = v0[i].normalize();
        vec[FOOT][i].setZ(z_ground_);
        vec[ANKLE][i].setZ(z_ground_);
    }

    /*!
        addToBoS: Adds two points to the BoS polygon, one left and one right. Each point is
        specified by a base vector (ankle_or_foot) plus a vector in polar coordinates (rho, phi) in local foot frame. 
        \param ankle_or_foot: whether foot or ankle marker is used as the base vector 
        \param rho: the radial coordinate
        \param phi: the angular coordinate
    */
    auto addToBoS = [&, this](ankle_foot_t ankle_or_foot, double rho, double phi){
        if (touches_ground_[ankle_or_foot][LEFT])
            bos_points.push_back(vec[ankle_or_foot][LEFT] + (v0[LEFT] * rho).rotate({0, 0, 1}, phi));
        if (touches_ground_[ankle_or_foot][RIGHT])
            bos_points.push_back(vec[ankle_or_foot][RIGHT] + (v0[RIGHT] * rho).rotate({0, 0, 1}, -phi));
    };


    // Oxford Foot Model 
    // https://www.c-motion.com/v3dwiki/index.php/Tutorial:_Oxford_Foot_Model

    addToBoS(FOOT, 0.05, -M_PI_4);          // L/RD1M: Head of the first metatarsal
    addToBoS(FOOT, 0.03, 0.0);              // L/RTOE: Center of the toe
    addToBoS(FOOT, 0.03, M_PI_2);           // L/RD5M: Head of the fifth metatarsal
    addToBoS(FOOT, 0.03, 6 * M_PI_4);

    addToBoS(ANKLE, 0.01, 5.5 * M_PI_4);     // L/RMMA: Medial malleolus
    addToBoS(ANKLE, 0.01, 2.5 * M_PI_4);     // L/RLMA: Lateral malleolus
    addToBoS(ANKLE, 0.02, 3 * M_PI_4);     // L/RLMA: Lateral malleolus
    addToBoS(ANKLE, 0.02, 5 * M_PI_4);     // L/RMMA: Medial malleolus

    if (bos_points.size() < 3)
        return res;

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

    return res;
}

mos_t GaitAnalyzer::getMoS(const bos_t & bos_points, const com_t & xcom)
{
    mos_t res;
    com_t pxcom = xcom;
    pxcom.setZ(z_ground_);

    double mos_sign = 1.0;
    double dist_min = 2.0;
    tf2::Vector3 pt_min;

    if (bos_points.size() < 3)
    {
        ROS_WARN_THROTTLE(10, "Ill-formed BoS polygon.");
        return {};
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

    return res;
}

void GaitAnalyzer::updateGaitPhase()
{
    double ground_clearance[2] = 
    {
        vec_joints_[K4ABT_JOINT_FOOT_LEFT].getZ() - z_ground_,
        vec_joints_[K4ABT_JOINT_FOOT_RIGHT].getZ() - z_ground_
    };

    for (int i = 0; i < 2; i++)
    {
        const double GROUND_CLEARANCE_THREASHOLD = .03;
        if (ground_clearance[i] < GROUND_CLEARANCE_THREASHOLD)
        {
            gait_phase_[i] = GAIT_PHASE_FOOT_FLAT;
            touches_ground_[FOOT][i] = true;
            touches_ground_[ANKLE][i] = true;
        }
        else
        {
            gait_phase_[i] = GAIT_PHASE_SWING;
            touches_ground_[FOOT][i] = false;
            touches_ground_[ANKLE][i] = false;
        }
    }
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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "gait_analyzer");
    GaitAnalyzer ga;
 
    //create subscriber to collect data from sport soles, IMU, skeleton
    //ros::Subscriber sportsole_left_sub = node_handle.subscribe<sport_sole::SportSole>("sportsole_left", 1000,  boost::bind(sportsole_callback, _1, LR_left));
    //ros::Subscriber sportsole_right_sub = node_handle.subscribe<sport_sole::SportSole>("sportsole_right", 1000, boost::bind(sportsole_callback, _1, LR_right));

    
    
    ros::spin();
    
    


    //create publishers here

}