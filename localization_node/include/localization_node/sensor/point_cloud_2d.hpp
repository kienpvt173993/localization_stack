#ifndef LOCALIZATION_STACK_POINT_CLOUD_2D__HPP__
#define LOCALIZATION_STACK_POINT_CLOUD_2D__HPP__

#include "localization_node/sensor/point_cloud_interface.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "localization_node/transform/rigid_transform.hpp"

namespace localization_node{
namespace sensor{

class PointCloud2D : PointCloudInterface
{
public:
    explicit PointCloud2D(const sensor_msgs::msg::LaserScan& sensor,
        const transform::Rigid3f& origin);

    PointCloud2D(const std::vector<Eigen::Vector3f>& points);

    PointCloud2D(const sensor_msgs::msg::LaserScan& sensor,
        const transform::Rigid3f& origin, const float& min, const float& max);

    ~PointCloud2D() override;
    void clear() override final;
    bool empty() const override final;
    void pushBack(Eigen::Vector3f point ,float intensity) override;
    void pushBack(geometry_msgs::msg::Point32, float intensity) override;
    sensor_msgs::msg::PointCloud getRosPointCloud() const override final;
    std::vector<Eigen::Vector3f> getPointCloud() const override final;
};
}}

#endif