#ifndef DISTANCE_CONTROLLER_H
#define DISTANCE_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <angles/angles.h>

struct HumanState {
    ros::Time stamp;
    double distance;
    double bearing;
    //double speed;
    HumanState operator-(HumanState & rhs)
    {
      HumanState ret;
      ret.stamp = (stamp > rhs.stamp) ? stamp : rhs.stamp;
      ret.distance = distance - rhs.distance;
      ret.bearing = angles::normalize_angle(bearing - rhs.bearing);
      return ret;
    }
};
std::ostream & operator<<(std::ostream & os, const HumanState & state)
{
  return os << state.distance << " m, " << state.bearing << " rad";
}

class DistanceController {
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_cmd_vel_in_;
  ros::Publisher pub_cmd_vel_out_;
  
  tf::TransformListener tf_listener_;
  HumanState estimated_state_, desired_state_;
  tf::StampedTransform tf_base_to_pelvis_;

public:
  DistanceController();
  ~DistanceController();
  void cmdVelCB(const geometry_msgs::Twist & cmd_vel_in);
};

template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi)
{
    return assert( !(hi < lo) ),
        (v < lo) ? lo : ((hi < v) ? hi : v);
}

#endif