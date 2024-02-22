#ifndef LOCALIZATION_STACK_POINT_CLOUD_INTERFACE__HPP__
#define LOCALIZATION_STACK_POINT_CLOUD_INTERFACE__HPP__

#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "eigen3/Eigen/Dense"

namespace localization_node{
namespace sensor{

class PointCloudInterface
{
public:
    virtual ~PointCloudInterface(){};
    virtual void clear() = 0;
    virtual bool empty() const = 0;
    virtual void pushBack(Eigen::Vector3f point ,float intensity) = 0;
    virtual void pushBack(geometry_msgs::msg::Point32, float intensity) = 0;
    virtual sensor_msgs::msg::PointCloud getRosPointCloud() const = 0;
    virtual std::vector<Eigen::Vector3f> getPointCloud() const = 0;
};
}}

#endif