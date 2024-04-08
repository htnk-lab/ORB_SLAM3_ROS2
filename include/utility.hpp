#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = stamp.sec + (stamp.nanosec * pow(10, -9));
    return seconds;
  }

  static geometry_msgs::msg::Point eigenToPointMsg(Eigen::Vector3f &e)
  {
    geometry_msgs::msg::Point p;
    p.x = e.x();
    p.y = e.y();
    p.z = e.z();
    return p;
  }
  static geometry_msgs::msg::Quaternion eigenToQuaternionMsg(Eigen::Quaternionf &e)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = e.w();
    q.x = e.x();
    q.y = e.y();
    q.z = e.z();
    return q;
  }

  static geometry_msgs::msg::Pose sophusToPoseMsg(const Sophus::SE3f &s)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = s.translation().x();
    pose.position.y = s.translation().y();
    pose.position.z = s.translation().z();
    pose.orientation.w = s.unit_quaternion().coeffs().w();
    pose.orientation.x = s.unit_quaternion().coeffs().x();
    pose.orientation.y = s.unit_quaternion().coeffs().y();
    pose.orientation.z = s.unit_quaternion().coeffs().z();
    return pose;
  }
};

#endif
