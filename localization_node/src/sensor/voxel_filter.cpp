#include "localization_node/sensor/voxel_filter.hpp"
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/voxel_grid.h>


Points voxelFilter(const Points* points, const float resolution){
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

    std::cerr << points->width * points->height << " data points " 
        <<pcl::getFieldsList (*points) << std::endl;
    
    pcl::VoxelGrid<pcl::PCLPointCloud2> convert;
    convert.setInputcloud(points);
    convert.setLeafSize(resolution,resolution,resolution);
    convert.filter(*cloud_filtered);
    std::cerr << cloud_filtered->height *cloud_filtered->height << " data poit"
        << pcl::getFiedsList (*cloud_filtered) << std::endl:
    
    Points result;
    for (const auto& count : cloud_filtered){
        result.push_back(count);
    }
    return *result;
}