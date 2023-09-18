#include "robot_localization/mapping/sensor/point_cloud.hpp"
#include "gtest/gtest.h"
using namespace robot_localization::mapping;
using namespace sensor_msgs::msg;
std::vector<float> lidar_scan_array = {10.,15.,0,100, //0.5,0.6,0.7,0.8
                                        11.,17,11.2,12.}; //0.9,1.,1.1,1.2

sensor::PointCloud createPointCloud(){
    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = 0.5;
    scan.angle_increment = 0.1;
    scan.angle_max = scan.angle_min + scan.angle_increment*lidar_scan_array.size();
    scan.ranges = lidar_scan_array;
    return sensor::PointCloud(&scan, 30., 1.);
}

TEST(PointCloudTest, size){
    auto points = createPointCloud();
    EXPECT_EQ(points.size(), (size_t)6);
}

TEST(PointCloudTest, points){
    auto points = createPointCloud();
    EXPECT_NEAR(points.points[0].x, 8.776, 0.1);
    EXPECT_NEAR(points.points[0].y, 4.794, 0.1);

    EXPECT_NEAR(points.points[4].x, 5.080, 0.1);
    EXPECT_NEAR(points.points[4].y, 9.982, 0.1);

    EXPECT_NEAR(points.points.back().x, 4.348, 0.1);
    EXPECT_NEAR(points.points.back().y, 11.184, 0.1);
}

TEST(PointCloudTest, push_back){
    auto points = createPointCloud();
    points.pushBack(2.,5.,10.);
    EXPECT_EQ(points.size(), (size_t)7);
    EXPECT_NEAR(points.points.back().x, 2., 0.1);
    EXPECT_NEAR(points.points.back().y, 5., 0.1);
    EXPECT_NEAR(points.intensities.back(), 10., 0.1);
}