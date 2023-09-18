#ifndef VOXEL_FILTER__HPP__
#define VOXEL_FILTER__HPP__
#include "robot_localization/mapping/sensor/point_cloud.hpp"
namespace robot_localization{
namespace mapping{
namespace sensor{
struct VoxelFilterOptions{
    int min_num_points = 100;
    float resolution = 0.025;
    float max_length = 0.5;
};
PointCloud adaptiveVoxelFilter(const VoxelFilterOptions& options, 
    PointCloud& point_cloud);
PointCloud voxelFilter(PointCloud& point_cloud, const float resolution);
}
}
}
#endif