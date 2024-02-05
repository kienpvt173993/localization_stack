#ifndef UTILS_COMMON__HPP__
#define UTILS_COMMON__HPP__
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point32.hpp"
namespace robot_localization{
namespace utils{
inline geometry_msgs::msg::Pose2D createPose2D(double x, double y, double theta){
    geometry_msgs::msg::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    return pose;
}
inline geometry_msgs::msg::Point32 createPoint32(double x, double y){
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    return point;
}
}}
#endif