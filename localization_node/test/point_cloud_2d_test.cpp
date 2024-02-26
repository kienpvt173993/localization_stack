#include "gtest/gtest.h"
#include "localization_node/sensor/point_cloud_2d.hpp"

namespace localization_node{
namespace sensor{

sensor_msgs::msg::LaserScan createLaserScan(){
    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = -0.5;
    scan.angle_max = 0.5;
    scan.angle_increment = 0.1;
    std::vector<float> ranges = {1.f, 10.f, 5.f, 
        20.f, 1.f, 1.5f, 7.f, 15.6f, 6.f, 8.f, 11.6f};
    scan.ranges.insert(scan.ranges.end(),
        ranges.begin(),
        ranges.end());
    return scan;
}

PointCloud2D createPointCloud(){
    auto scan = createLaserScan();
    transform::Rigid3f origin({1.f, 2.f, 0.f},
        {1.f, 0.f, 0.f, 0.f});
    return PointCloud2D(scan, origin);
}

TEST(PointCloud2DTest, cloud_size){
    auto scan = createLaserScan();
    auto pcl_1 = PointCloud2D(scan);
    auto length_1 = pcl_1.size();
    EXPECT_EQ(static_cast<int>(length_1), 11);
}

}}