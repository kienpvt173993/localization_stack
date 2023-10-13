#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "robot_localization/utils/map_tools.hpp"
#include "robot_localization/utils/maths.hpp"
#include <algorithm>
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;
namespace robot_localization{
namespace mapping{
namespace grid{
Grid::Grid(OccupancyGrid* ros_map){
    reset(ros_map);
}
Grid::~Grid(){}
Eigen::Vector2i Grid::getCell(double x, double y) const{
    return {MAP_GXWX(map_meta_, x), MAP_GYWY(map_meta_, y)};
}
Eigen::Vector2i Grid::getCell(Eigen::Vector2d pose) const{
    return getCell(pose[0], pose[1]);
}
Eigen::Vector2i Grid::getCell(Pose2D pose) const{
    return getCell(pose.x, pose.y);
}
double Grid::getProbability(Eigen::Vector2i pose_i) const{
    if(MAP_VALID(map_meta_, pose_i[0], pose_i[1])){
        int index = MAP_INDEX(map_meta_, pose_i[0], pose_i[1]);
        return probability_data_[index];
    }
    else{
        return 0.1;
    }
}
int8_t Grid::getValueAtRaw(Eigen::Vector2i pose_i) const{
    if(MAP_VALID(map_meta_, pose_i[0], pose_i[1])){
        int index = MAP_INDEX(map_meta_, pose_i[0], pose_i[1]);
        return raw_data_[index];
    }
    else{
        return -1;
    }
}
int8_t Grid::getValueAtCurrent(Eigen::Vector2i pose_i) const{
    if(MAP_VALID(map_meta_, pose_i[0], pose_i[1])){
        int index = MAP_INDEX(map_meta_, pose_i[0], pose_i[1]);
        return current_data_[index];
    }
    else{
        return -1;
    }
}
void Grid::setProbability(Eigen::Vector2i pose_i, float probability){
    if(MAP_VALID(map_meta_, pose_i[0], pose_i[1])){
        int index = MAP_INDEX(map_meta_, pose_i[0], pose_i[1]);
        probability = utils::clamp(probability, 0.1f, 0.9f);
        probability_data_[index] = probability;
        if(probability > 0.8) current_data_[index] = 100;
        else if (probability < 0.2) current_data_[index] = 0;
        else if(current_data_[index] == -1) current_data_[index] = 0;
    }
}
void Grid::reset(OccupancyGrid* ros_map){
    if (ros_map == nullptr){
        reset();
    }
    else{
        map_meta_ = std::make_shared<MapMetaData>();
        *map_meta_ = ros_map->info;
        raw_data_ = ros_map->data;
        current_data_ = ros_map->data;
        updateMapMetaToProbability();
    }
}
void Grid::reset(){
    current_data_ = raw_data_;
    updateMapMetaToProbability();
}
OccupancyGrid Grid::getRawMap() const{
    OccupancyGrid map;
    map.info = *map_meta_;
    map.data = raw_data_;
    return map;
}
OccupancyGrid Grid::getCurrentMap() const{
    OccupancyGrid map;
    map.info = *map_meta_;
    map.data = current_data_;
    return map;
}
void Grid::updateMapMetaToProbability(){
    probability_data_.clear();
    for(const auto &data : raw_data_){
        if(data == 100){
            probability_data_.push_back(0.9);
        }
        else{
            probability_data_.push_back(0.1);
        }
    }
}
nav_msgs::msg::MapMetaData Grid::getMapMeta() const{
    return *map_meta_.get();
}
}}}