#include "localization_node/grid/grid_2d.hpp"
#include <bits/stl_algo.h>

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace localization_node::transform;

namespace localization_node{
namespace grid{

Grid2D::Grid2D(const OccupancyGrid & map){
    origin_ = map.data;
    current_ = map.data;
    meta_data_ = map.info;
    for(const auto& value: origin_){
        if(value == 100){
            probability_grid_.push_back(0.9);
        }
        else{
            probability_grid_.push_back(0.1);
        }
    }
}

Grid2D::~Grid2D(){

}

void Grid2D::setProbabilityGrid(const Eigen::Array3i & cell, 
    const float & p){
    std::clamp(p, 0.1f, 0.2f);
}

float Grid2D::getProbabilityGrid(const Eigen::Array3i & cell) const {
    return 0.f;
}

OccupancyGrid* Grid2D::getRosMap() const {
    return new OccupancyGrid();
}

Eigen::Array3i Grid2D::getCellIndex(const transform::Rigid3f& point) const {

}

Rigid3f::Vector Grid2D::getPoseTranslation(const Rigid3f & pose) const {
    
}

}
}