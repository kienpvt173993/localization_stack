#ifndef LOCALIZATION_STACK_GRID_2D__HPP__
#define LOCALIZATION_STACK_GRID_2D__HPP__

#include "localization_node/grid/grid_interface.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace localization_node{
namespace grid{

class Grid2D: public GridInterface{

public:

    Grid2D(const nav_msgs::msg::OccupancyGrid & map);

    ~Grid2D() override;

    void setProbabilityGrid(const Eigen::Array3i & cell, 
        const float & p) override final;

    float getProbabilityGrid(const Eigen::Array3i & cell) const override final;

    const GridType getGridType() const override final{
        return GridType::Grid2DMap;
    }

    nav_msgs::msg::OccupancyGrid* getRosMap() const override final;

    Eigen::Array3i getCellIndex(const transform::Rigid3f& point) const;

    transform::Rigid3f::Vector getPoseTranslation(const transform::Rigid3f & pose) const override final;

protected:
    std::vector<int8_t> origin_;
    std::vector<int8_t> current_;
    std::vector<float> probability_grid_;
    nav_msgs::msg::MapMetaData meta_data_;
};

}
}

#endif