#ifndef LOCALIZATION_STACK_MAP_LIMIT__HPP__
#define LOCALIZATION_STACK_MAP_LIMIT__HPP__

#include "nav_msgs/msg/map_meta_data.hpp"
#include "localization_node/transform/rigid_transform.hpp"

namespace localization_node{
namespace utils{

inline bool inRosMap2D(const nav_msgs::msg::MapMetaData& meta_data,
    const Eigen::Vector3i& position){
    return meta_data.height >= static_cast<uint32_t>(position.y()) 
        && meta_data.width >= static_cast<uint32_t>(position.x());
}

inline size_t getValueIndex(const nav_msgs::msg::MapMetaData& meta_data,
    const Eigen::Vector3i& position){
    return position.x() + position.y()*meta_data.width;
}

inline int getXi(const nav_msgs::msg::MapMetaData& meta_data, const float& x){
    return (floor((x - meta_data.origin.position.x) 
        / meta_data.resolution + 0.5) + meta_data.width / 2);
}

inline int getYi(const nav_msgs::msg::MapMetaData& meta_data, const float& y){
    return (floor((y - meta_data.origin.position.y) 
        / meta_data.resolution + 0.5) + meta_data.height / 2);
}

}}

#endif