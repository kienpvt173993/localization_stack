#include "robot_localization/mapping/sensor/voxel_filter.hpp"
#include "robot_localization/utils/maths.hpp"
#include "absl/container/flat_hash_map.h"
#include <random>
using namespace geometry_msgs::msg;
namespace robot_localization{
namespace mapping{
namespace sensor{

using VoxelKeyType = uint64_t;
VoxelKeyType getVoxelCellIndex(const Point32& point, const float resolution) {
  Eigen::Array3f index = {point.x, point.y, point.z};
  index = index.array() / resolution;
  const uint64_t x = utils::roundToInt64(index.x());
  const uint64_t y = utils::roundToInt64(index.y());
  const uint64_t z = utils::roundToInt64(index.z());
  return (x << 42) + (y << 21) + z;
}
std::vector<bool> randomizedVoxelFilterIndices(
        PointCloud& point_cloud, const float resolution) {
    std::minstd_rand0 generator;
    absl::flat_hash_map<VoxelKeyType, std::pair<int, int>>
        voxel_count_and_point_index;
    for(size_t i = 0; i < point_cloud.size(); i++) {
        auto& voxel = voxel_count_and_point_index[getVoxelCellIndex(
            point_cloud.points[i], resolution)];
        voxel.first++;
        if (voxel.first == 1) {
            voxel.second = i;
        } else {
            std::uniform_int_distribution<> distribution(1, voxel.first);
            if (distribution(generator) == voxel.first) {
                voxel.second = i;
            }
        }
    }
    std::vector<bool> points_used(point_cloud.size(), false);
    for(const auto& voxel_and_index : voxel_count_and_point_index) {
        points_used[voxel_and_index.second.second] = true;
    }
    return points_used;
}
PointCloud randomizedVoxelFilter(PointCloud& point_cloud, const float resolution){
    auto point_used = randomizedVoxelFilterIndices(point_cloud, resolution);

    auto results = point_cloud;
    results.intensities.clear();
    results.points.clear();
    for(size_t i = 0; i < point_cloud.size(); i++){
        if(point_used[i]){
            results.pushBack(point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.intensities[i]);
        }
    }
    return results;
}
PointCloud voxelFilter(PointCloud& point_cloud, const float resolution){
    return randomizedVoxelFilter(point_cloud, resolution);
}
PointCloud adaptiveVoxelFilter(const VoxelFilterOptions& options, 
    PointCloud& point_cloud){
    if(point_cloud.size() <= (size_t) options.min_num_points){
        return point_cloud;
    }
    PointCloud result = randomizedVoxelFilter(point_cloud, options.max_length);
    if(result.size() <= (size_t) options.min_num_points){
        return result;
    }
    for (float high_length = options.max_length;
        high_length > 1e-2f * options.max_length; high_length /= 2.f){
        float low_length = high_length / 2.f;
        result = randomizedVoxelFilter(point_cloud, low_length);
        if (result.size() >= (size_t) options.min_num_points){
            while ((high_length - low_length) / low_length > 1e-1f){
                const float mid_length = (low_length + high_length) / 2.f;
                PointCloud candidate = randomizedVoxelFilter(point_cloud, mid_length);
                if (candidate.size() >= (size_t) options.min_num_points){
                    low_length = mid_length;
                    result = candidate;
                }
                else{
                    high_length = mid_length;
                }
            }
            return result;
        }
    }
    return result;
}
}}}