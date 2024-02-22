#ifndef LOCALIZATION_STACK_VOXEL_FILTER__HPP__
#define LOCALIZATION_STACK_VOXEL_FILTER__HPP__
#include "eigen3/Eigen/Dense"
#include <vector>
#include "localization_node/sensor/point_cloud_interface.hpp"

namespace localization_node{
namespace sensor{

struct VoxelFilterOptions{
    int min_num_points = 100;
    float resolution = 0.025;
    float max_length = 0.5;
};

Points adaptiveVoxelFilter(const VoxelFilterOptions& options, 
    const Points* points);

Points voxelFilter(
    const Points* points, const float resolution);

}}

#endif