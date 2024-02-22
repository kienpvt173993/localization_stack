#ifndef LOCALIZATION_STACK_GRID_INTERFACE__HPP__
#define LOCALIZATION_STACK_GRID_INTERFACE__HPP__

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "eigen3/Eigen/Dense"
#include "localization_node/transform/rigid_transform.hpp"

namespace localization_node{
namespace grid{

enum class GridType { Grid2DMap, Grid3DMap};

class GridInterface
{
public:
    virtual ~GridInterface() = 0;
    virtual void setProbabilityGrid(const Eigen::Array3i & cell, 
        const float & p) = 0;

    virtual float getProbabilityGrid(const Eigen::Array3i & cell) const = 0;

    virtual const GridType getGridType() const = 0;

    virtual nav_msgs::msg::OccupancyGrid* getRosMap() const = 0;

    virtual Eigen::Array3i getCellIndex(const Eigen::Vector3f& point) const = 0;

    Eigen::Array3i getCellIndex(const transform::Rigid3f::Vector& point) const {
        return this->getCellIndex({point.x(), point.y(), point.z()});
    }

    Eigen::Array3i getCellIndex(const transform::Rigid3f& pose) const {
        return this->getCellIndex(pose.translation());
    }

    virtual transform::Rigid3f::Vector getPoseTranslation(const transform::Rigid3f & pose) const = 0;
};

}
}

#endif