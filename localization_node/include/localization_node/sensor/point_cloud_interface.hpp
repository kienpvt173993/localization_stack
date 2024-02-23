#ifndef LOCALIZATION_STACK_POINT_CLOUD_INTERFACE__HPP__
#define LOCALIZATION_STACK_POINT_CLOUD_INTERFACE__HPP__

#include "sensor_msgs/msg/point_cloud.hpp"
#include "eigen3/Eigen/Dense"
#include "localization_node/transform/rigid_transform.hpp"

namespace localization_node{
namespace sensor{

using Point = Eigen::Vector3f;
using Points = std::vector<Point>;

enum class PointCloudType { PointCloud2D, PointCloud3D};

class PointCloudInterface
{
public:
    virtual ~PointCloudInterface(){};
    virtual PointCloudType getPointCloudType() const = 0;
    virtual void clear() = 0;
    virtual bool empty() const = 0;
    virtual void pushBack(Point point ,float intensity) = 0;
    virtual void pushBack(geometry_msgs::msg::Point32, float intensity) = 0;
    virtual sensor_msgs::msg::PointCloud getRosPointCloud() const = 0;
    virtual Points getPointCloud() const = 0;
    virtual Points getMissing() const = 0;
    virtual Points getReturn() const = 0;
    virtual void transform(const transform::Rigid3f& tf) = 0;
};
}}

#endif