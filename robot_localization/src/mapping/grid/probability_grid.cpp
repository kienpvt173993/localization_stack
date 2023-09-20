#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "robot_localization/utils/map_tools.hpp"
#include "robot_localization/utils/maths.hpp"
#include <algorithm>
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;
namespace robot_localization{
namespace mapping{
namespace grid{
ProbabilityGrid::ProbabilityGrid(OccupancyGrid* ros_map){
    reset(ros_map);
}
ProbabilityGrid::~ProbabilityGrid(){}
Eigen::Vector2i ProbabilityGrid::getCell(double x, double y){
    return {MAP_GXWX(map_meta_, x), MAP_GYWY(map_meta_, y)};
}
Eigen::Vector2i ProbabilityGrid::getCell(Eigen::Vector2d pose){
    return getCell(pose[0], pose[1]);
}
Eigen::Vector2i ProbabilityGrid::getCell(Pose2D pose){
    return getCell(pose.x, pose.y);
}
double ProbabilityGrid::getProbability(Eigen::Vector2i pose_i){
    if(MAP_VALID(map_meta_, pose_i[0], pose_i[1])){
        int index = MAP_INDEX(map_meta_, pose_i[0], pose_i[1]);
        return probability_data_[index];
    }
    else{
        return 0.1;
    }
}
void ProbabilityGrid::setProbability(Eigen::Vector2i pose_i, float probability){
    if(MAP_VALID(map_meta_, pose_i[0], pose_i[1])){
        int index = MAP_INDEX(map_meta_, pose_i[0], pose_i[1]);
        probability = utils::clamp(probability, 0.1f, 0.9f);
        probability_data_[index] = probability;
        if(probability > 0.8) current_data_[index] = 100;
        else if (probability < 0.2) current_data_[index] = 0;
        else if(current_data_[index] == -1) current_data_[index] = 0;
    }
}
void ProbabilityGrid::reset(OccupancyGrid* ros_map){
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
void ProbabilityGrid::reset(){
    current_data_ = raw_data_;
    updateMapMetaToProbability();
}
OccupancyGrid ProbabilityGrid::getRawMap(){
    OccupancyGrid map;
    map.info = *map_meta_;
    map.data = raw_data_;
    return map;
}
OccupancyGrid ProbabilityGrid::getCurrentMap(){
    OccupancyGrid map;
    map.info = *map_meta_;
    map.data = current_data_;
    return map;
}
void ProbabilityGrid::updateMapMetaToProbability(){
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
nav_msgs::msg::MapMetaData ProbabilityGrid::getMapMeta(){
    return *map_meta_.get();
}
}}}