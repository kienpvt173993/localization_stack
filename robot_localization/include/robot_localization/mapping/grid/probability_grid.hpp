#ifndef PROBABILITY_GRID__HPP__
#define PROBABILITY_GRID__HPP__
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/pose2_d.hpp"
namespace robot_localization{
namespace mapping{
namespace grid{
/**
 * @brief probability grid:
 * 0.9 is ostacle
 * 0.1 is free
 * range from 0.1 to 0.9: inflation
 * 
 */
class Grid{
public:
explicit Grid(nav_msgs::msg::OccupancyGrid* ros_map);
~Grid();

Eigen::Vector2i getCell(Eigen::Vector2d pose) const;
Eigen::Vector2i getCell(double x, double y) const;
Eigen::Vector2i getCell(geometry_msgs::msg::Pose2D pose) const;

double getProbability(Eigen::Vector2i pose_i) const;
void setProbability(Eigen::Vector2i pose_i, float probability);

void reset();
void reset(nav_msgs::msg::OccupancyGrid* ros_map);

nav_msgs::msg::OccupancyGrid getRawMap() const;
nav_msgs::msg::OccupancyGrid getCurrentMap() const;
nav_msgs::msg::MapMetaData getMapMeta() const;

protected:
void updateMapMetaToProbability();
std::shared_ptr<nav_msgs::msg::MapMetaData> map_meta_;
std::vector<int8_t> raw_data_;
std::vector<int8_t> current_data_;
std::vector<double> probability_data_;
};
}
}
}

#endif