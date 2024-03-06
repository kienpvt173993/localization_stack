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
    points.push_back({0.07,0.07,0.21});
    points.push_back({0.07,0.07,0.22});
    points.push_back({0.07,0.07,0.23});
    points.push_back({0.07,0.07,0.24});
    points.push_back({0.07,0.07,0.25});
    points.push_back({0.04,0.05,0.23});
    points.push_back({0.04,0.05,0.34});
    points.push_back({0.04,0.05,0.45});
    points.push_back({0.04,0.05,0.43});
    points.push_back({0.04,0.05,0.50});
    points.push_back({0.04,0.05,0.12});
    points.push_back({0.04,0.05,0.36});
    points.push_back({0.04,0.05,0.30});
    points.push_back({0.04,0.05,0.23});
    points.push_back({0.04,0.05,0.42});
    points.push_back({0.04,0.05,0.48});
    points.push_back({0.04,0.05,0.41});

    auto result = sensor::voxelFilter(&points, 0.05);
    EXPECT_EQ(result.size(),1 );
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