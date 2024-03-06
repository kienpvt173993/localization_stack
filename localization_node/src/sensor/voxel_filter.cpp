#include "localization_node/sensor/voxel_filter.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "glog/logging.h"

namespace localization_node{
namespace sensor{
Points voxelFilter( const Points* points, const float resolution){

    CHECK_GT(resolution, 0.);
    // Convert Points to pcl::PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    cloudXYZ->resize(points->size());
    cloudXYZ->width = points->size();
    cloudXYZ->height = 1;
    for( const auto &count : *points){
        pcl::PointXYZ point;
        point.x = count.x();
        point.y = count.y();
        point.z = count.z();
        cloudXYZ->push_back(point);
    }

    // Convert pcl::Point to Cloud2
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloudXYZ,*cloud);

    // Voxel filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> convert;
    convert.setInputCloud(cloud);
    convert.setLeafSize(resolution,resolution,resolution);
    convert.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ_result (new pcl::PointCloud<pcl::PointXYZ>);
    sensor::Points point_result;
    pcl::fromPCLPointCloud2(*cloud_filtered,*cloudXYZ_result);

    localization_node::sensor::Points points_result;
    for(const auto &count : *cloudXYZ_result){
        point_result.push_back({count.x,count.y,count.z});
    }
    return point_result;
}
}
}