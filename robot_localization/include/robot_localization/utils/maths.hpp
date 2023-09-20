#ifndef UTILS_MATH__HPP__
#define UTILS_MATH__HPP__
#include <bits/stdint-intn.h>
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>
namespace robot_localization{
namespace utils{
inline int roundToInt(const float x){return std::lround(x);}
inline int roundToInt(const double x){return std::lround(x);}
inline int64_t roundToInt64(const float x){return std::lround(x);}
inline int64_t roundToInt64(const double x){return std::lround(x);}
inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw){
    geometry_msgs::msg::Quaternion q;
    q.w = cos(yaw/2);
    q.z = sin(yaw/2);
    return q;
}
template <typename T>
T clamp(const T value, const T min, const T max) {
    assert(max > min);
    if(value > max){
        return max;
    }
    if(value < min){
        return min;
    }
    return value;
}
template <typename T>
T norminalAngle(T angle){
    while (angle > M_PI || angle < -M_PI){
        if(angle > M_PI) angle = angle - 2*M_PI;
        else if(angle > M_PI) angle = angle + 2*M_PI;
    }
    return angle;    
}
template <typename T>
T shortestDiffAngle(T source, T des){
    T angle = norminalAngle(des - source);
    return angle;    
}
}
}
#endif