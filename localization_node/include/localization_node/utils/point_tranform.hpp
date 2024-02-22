#ifndef LOCALIZATION_STACK_POINT_TRANSFORM__HPP__
#define LOCALIZATION_STACK_POINT_TRANSFORM__HPP__

#include "localization_node/sensor/point_cloud_interface.hpp"

namespace localization_node{
namespace utils{

inline void transformPoint(sensor::Point& point,
    const transform::Rigid3f& tf){
    point = tf.rotation()*point;
    point += tf.translation();
}

inline void transformPoints(sensor::Points& points,
    const transform::Rigid3f& tf){
    for(auto& point: points){
        transformPoint(point, tf);
    }
}

}
}

#endif