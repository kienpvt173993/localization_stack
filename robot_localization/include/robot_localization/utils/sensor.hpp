#ifndef UTILS_SENSOR__HPP__
#define UTILS_SENSOR__HPP__
#include <cmath>
#include <bits/stdint-intn.h>
#include <vector>
namespace robot_localization{
namespace utils{
template <typename T>
std::vector<T> rangeToPoint(T range, T angle){
    return {range*cos(angle), range*sin(angle)};
}
}
}
#endif