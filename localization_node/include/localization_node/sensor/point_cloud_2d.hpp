#ifndef LOCALIZATION_STACK_POINT_CLOUD_2D__HPP__
#define LOCALIZATION_STACK_POINT_CLOUD_2D__HPP__

#include "localization_node/sensor/point_cloud_interface.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace localization_node{
namespace sensor{

class PointCloud2D : PointCloudInterface
{

public:    
    explicit PointCloud2D(const sensor_msgs::msg::LaserScan& sensor,
        const transform::Rigid3f& origin);

    PointCloud2D(const Points& points);

    PointCloud2D();

    PointCloud2D(const sensor_msgs::msg::LaserScan& sensor,
        const transform::Rigid3f& origin, const float& min, const float& max);

    PointCloudType getPointCloudType() const override final{
        return PointCloudType::PointCloud2D;
    };

    ~PointCloud2D() override;
    void clear() override final;
    bool empty() const override final;
    void pushBack(Point point ,float intensity) override;
    void pushBack(geometry_msgs::msg::Point32, float intensity) override;
    sensor_msgs::msg::PointCloud getRosPointCloud() const override final;
    Points getPointCloud() const override final;
    Points getMissing() const override final;
    Points getReturn() const override final;
    void transform(const transform::Rigid3f& tf) override final;
protected:
    std::vector<float> intensity_;
    Points origin_data_, return_data_, miss_data_;
};
}}

#endif