#ifndef POINT_CLOUD_INSERTOR__HPP__
#define POINT_CLOUD_INSERTOR__HPP__
#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "robot_localization/mapping/sensor/point_cloud.hpp"
namespace robot_localization{
namespace mapping{
namespace grid{
struct PointCloudInsertOption{
    double hit_probability = 0.8;
    double miss_probability = 0.3;
};
class PointCloudInsertor{
public:
PointCloudInsertor(PointCloudInsertOption* option);
~PointCloudInsertor();
void Insert(const sensor::PointCloud& range_data, geometry_msgs::msg::Pose2D pose,
                      Grid* grid) const;
protected:
std::shared_ptr<PointCloudInsertOption> options_;
void updateAPointCloud(geometry_msgs::msg::Point32 point, geometry_msgs::msg::Pose2D pose, 
    Grid* grid, bool hit) const;
std::vector<Eigen::Vector2i> getCellsInRangeData(geometry_msgs::msg::Point32 point, 
    geometry_msgs::msg::Pose2D pose, Grid* grid) const;
void updateCell(Eigen::Vector2i cell, bool hit ,Grid* grid) const;
double odds(double p) const;
};
}
}
}
#endif