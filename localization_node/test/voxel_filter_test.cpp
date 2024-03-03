#include "localization_node/sensor/voxel_filter.hpp"
#include "gtest/gtest.h"

using namespace localization_node;
sensor::VoxelFilterOptions createOptions(){
    sensor::VoxelFilterOptions options;
    options.max_length = 0.5;
    options.min_num_points = 100;
    options.resolution = 0.025;
    return options;
}

TEST(VoxelFilterTest, returns_one_point_in_each_voxel){
    sensor::Points points;
    points.push_back({0.05,0.07,0.1});
    points.push_back({0.05,0.07,0.2});
    sensor::voxelFilter(&points, 0.05);
}

// TEST(VoxelFilterTest, handles_large_coordinates){
//     sensor::PointCloudType point_cloud;
//     point_cloud.pushBack(100000.f,0.1f,0.f);
//     point_cloud.pushBack(100000.001,-.0001f,0.f);
//     point_cloud.pushBack(100000.003,-.0001f,0.f);
//     point_cloud.pushBack(-200000.f,-.0,0.);
//     sensor::PointCloudType result = sensor::voxelFilter(point_cloud, 0.01);
//     EXPECT_EQ(result.size(), (size_t)3);
// }