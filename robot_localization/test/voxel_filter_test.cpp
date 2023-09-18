#include "robot_localization/mapping/sensor/voxel_filter.hpp"
#include "gtest/gtest.h"

using namespace robot_localization::mapping;
using namespace sensor_msgs::msg;
sensor::VoxelFilterOptions createOptions(){
    sensor::VoxelFilterOptions options;
    options.max_length = 0.5;
    options.min_num_points = 100;
    options.resolution = 0.025;
    return options;
}

TEST(VoxelFilterTest, returns_one_point_in_each_voxel){
    sensor::PointCloud point_cloud;
    point_cloud.pushBack(0.,0.,0.);
    point_cloud.pushBack(.1,-.1,0.);
    point_cloud.pushBack(.3,-.1,0.);
    point_cloud.pushBack(.4,-.2,0.);
    sensor::PointCloud result = sensor::voxelFilter(point_cloud, 0.3);
    EXPECT_EQ(result.size(), (size_t)3);
}

TEST(VoxelFilterTest, handles_large_coordinates){
    sensor::PointCloud point_cloud;
    point_cloud.pushBack(100000.f,0.1f,0.f);
    point_cloud.pushBack(100000.001,-.0001f,0.f);
    point_cloud.pushBack(100000.003,-.0001f,0.f);
    point_cloud.pushBack(-200000.f,-.0,0.);
    sensor::PointCloud result = sensor::voxelFilter(point_cloud, 0.01);
    EXPECT_EQ(result.size(), (size_t)3);
}