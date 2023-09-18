#ifndef UTILS_MATH__HPP__
#define UTILS_MATH__HPP__
#include <cmath>
#include <bits/stdint-intn.h>
namespace robot_localization{
namespace utils{
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
inline int roundToInt(const float x){return std::lround(x);}
inline int roundToInt(const double x){return std::lround(x);}
inline int64_t roundToInt64(const float x){return std::lround(x);}
inline int64_t roundToInt64(const double x){return std::lround(x);}
}
}
#endif