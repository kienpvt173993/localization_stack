#ifndef LOCALIZATION_STACK_VOXEL_FILTER__HPP__
#define LOCALIZATION_STACK_VOXEL_FILTER__HPP__
#include "eigen3/Eigen/Dense"
#include <vector>

namespace localization_node{
namespace sensor{

struct VoxelFilterOptions{
    int min_num_points = 100;
    float resolution = 0.025;
    float max_length = 0.5;
};

std::vector<Eigen::Vector3f> adaptiveVoxelFilter(const VoxelFilterOptions& options, 
    const std::vector<Eigen::Vector3f>* points);
std::vector<Eigen::Vector3f> voxelFilter(
    const std::vector<Eigen::Vector3f>* points, const float resolution);

}}

#endif