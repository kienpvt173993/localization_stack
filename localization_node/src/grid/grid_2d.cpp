#include "localization_node/grid/grid_2d.hpp"
#include <bits/stl_algo.h>
#include "localization_node/utils/map_limit.hpp"

using namespace nav_msgs::msg;
using namespace localization_node::transform;

namespace localization_node{
namespace grid{

Grid2D::Grid2D(const OccupancyGrid & map){
    current_ = map.data;
    meta_data_ = map.info;
    for(const auto& value: current_){
        if(value == 100){
            probability_grid_.push_back(0.9);
        }
        else{
            probability_grid_.push_back(0.1);
        }
    }
}

Grid2D::~Grid2D(){
    current_.clear();
    probability_grid_.clear();
}

void Grid2D::setProbability(const Eigen::Array3i & cell, 
    const float & p){
    if (this->inLimit(cell)){
        auto new_p = (p < 0.1f) ? 0.1f : (0.9f < p) ? 0.9f : p;
        auto ind = utils::getValueIndex(meta_data_, cell);
        auto ros_cell = current_[ind];
        probability_grid_[ind] = new_p;
        if(ros_cell == 0 && new_p > 0.7f){
            current_[ind] = 100;
        }
        else if(ros_cell == 100 && new_p < 0.3f){
            current_[ind] = 0;
        }
        else if(ros_cell == -1 && new_p < 0.5f){
            current_[ind] = 0;
        }
        else if(ros_cell == -1 && new_p > 0.5f){
            current_[ind] = 100;
        }
    }
}

float Grid2D::getProbability(const Eigen::Array3i & cell) const {
    if(this->inLimit(cell)){
        auto ind = utils::getValueIndex(meta_data_, cell);
        return probability_grid_[ind];
    }
    else{
        return 0.1f;
    }
}

bool Grid2D::inLimit(const Eigen::Array3i & cell) const{
    return utils::inRosMap2D(meta_data_, cell);
}

OccupancyGrid* Grid2D::getRosMap() const {
    auto map = new OccupancyGrid();
    map->data = current_;
    map->info = meta_data_;
    return map;
}

Eigen::Array3i Grid2D::getCellIndex(const sensor::Point& point) const {
    return {
        utils::getXi(meta_data_, point.x()),
        utils::getXi(meta_data_, point.y()),
        0
    };
}

Rigid3f Grid2D::getPositionInImage(const Rigid3f & pose) const {
    Rigid3d origin({meta_data_.origin.position.x,
            meta_data_.origin.position.y, 
            meta_data_.origin.position.z},
        {meta_data_.origin.orientation.w,
            meta_data_.origin.orientation.x,
            meta_data_.origin.orientation.y,
            meta_data_.origin.orientation.z});
    return pose*origin.inverse().cast<float>();
}

}
}