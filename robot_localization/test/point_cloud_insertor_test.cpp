#include "robot_localization/mapping/grid/point_cloud_insertor.hpp"
#include "gtest/gtest.h"
using namespace robot_localization::mapping;
using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;

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

TEST(PointCloudInsertorTest, insert_point_cloud){
    OccupancyGrid map;
    for(int i = 0; i < 10000; i++){
        map.data.push_back(0);
    }
    map.info.width = 100;
    map.info.height = 100;
    map.info.resolution = 0.05;
    grid::Grid grid(&map);
    auto laser = createPointCloud();
}