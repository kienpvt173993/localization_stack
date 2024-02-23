#ifndef LOCALIZATION_STACK_GRID_INTERFACE__HPP__
#define LOCALIZATION_STACK_GRID_INTERFACE__HPP__

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "eigen3/Eigen/Dense"
#include "localization_node/transform/rigid_transform.hpp"
#include "localization_node/sensor/point_cloud_interface.hpp"

namespace localization_node{
namespace grid{

enum class GridType { Grid2DMap, Grid3DMap};

class GridInterface
{
public:
    virtual ~GridInterface(){};
    virtual void setProbability(const Eigen::Array3i & cell, 
        const float & p) = 0;

    virtual float getProbability(const Eigen::Array3i & cell) const = 0;

    virtual bool inLimit(const Eigen::Array3i & cell) const = 0;

    virtual GridType getGridType() const = 0;

    virtual nav_msgs::msg::OccupancyGrid* getRosMap() const = 0;

    virtual Eigen::Array3i getCellIndex(const sensor::Point& point) const = 0;

    virtual transform::Rigid3f getPositionInImage(const transform::Rigid3f & pose) const = 0;
};

}
}

#endif