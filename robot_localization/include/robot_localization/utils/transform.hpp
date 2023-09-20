#ifndef UTILS_TRANSFORM__HPP__
#define UTILS_TRANSFORM__HPP__
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "robot_localization/mapping/sensor/point_cloud.hpp"
#include "eigen3/Eigen/Dense"
#include "robot_localization/utils/maths.hpp"
namespace robot_localization{
namespace utils{
inline geometry_msgs::msg::Pose2D transformPose2D(geometry_msgs::msg::Pose2D source, 
    geometry_msgs::msg::Pose2D tf){
    Eigen::Vector3d source_v, tf_v, des_v;
    Eigen::Matrix3d tf_matrix;
    source_v << source.x, source.y, source.theta;
    tf_v << tf.x, tf.y, tf.theta;
    tf_matrix << cos(tf.theta), -sin(tf.theta), 0.,
                sin(tf.theta), cos(tf.theta), 0.,
                0.,0.,1.;
    des_v = tf_matrix*source_v + tf_v;
    geometry_msgs::msg::Pose2D des;
    des.x = des_v[0];
    des.y = des_v[1];
    des.theta = norminalAngle(des_v[2]);
    return des;
}
inline geometry_msgs::msg::Point32 transformPoin2D(geometry_msgs::msg::Point32 source,
    geometry_msgs::msg::Pose2D tf){
    Eigen::Vector2d source_v, tf_v, des_v;
    Eigen::Matrix2d tf_matrix;
    source_v << source.x, source.y;
    tf_v << tf.x, tf.y;
    tf_matrix << cos(tf.theta), -sin(tf.theta),
                sin(tf.theta), cos(tf.theta);
    des_v = tf_matrix*source_v + tf_v;
    geometry_msgs::msg::Point32 des;
    des.x = des_v[0];
    des.y = des_v[1];
    return des;
}
inline std::vector<geometry_msgs::msg::Point32> transformPointArray(std::vector<geometry_msgs::msg::Point32> source,
    geometry_msgs::msg::Pose2D tf){
    std::vector<geometry_msgs::msg::Point32> des;
    for(auto point: source){
        auto des_point = transformPoin2D(point, tf);
        des.push_back(des_point);
    }
    return des;
}
inline mapping::sensor::PointCloud transformPointCloud(mapping::sensor::PointCloud source,
    geometry_msgs::msg::Pose2D tf){
    mapping::sensor::PointCloud des = source;
    des.clear();
    for(size_t i = 0; i < source.size(); i++){
        auto des_point = transformPoin2D(source.points[i], tf);
        des.points.push_back(des_point);
        des.intensities.push_back(des.intensities[i]);
    }
    des.missing_point = transformPointArray(source.missing_point, tf);
    return des;
}
}
}
#endif