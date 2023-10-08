#include "robot_localization/mapping/grid/point_cloud_insertor.hpp"
#include "robot_localization/utils/transform.hpp"
#include "robot_localization/utils/maths.hpp"
#include "tf2/utils.h"
#include <unordered_set>
using namespace geometry_msgs::msg;
namespace robot_localization{
namespace mapping{
namespace grid{
PointCloudInsertor::PointCloudInsertor(PointCloudInsertOption* options){
    assert(options!=nullptr);
    options_.reset(options);
    assert(options->miss_probability > 0. && options->miss_probability < 0.5);
    assert(options->hit_probability < 1.0 && options->hit_probability > 0.5);
}
PointCloudInsertor::~PointCloudInsertor(){}
void PointCloudInsertor::Insert(const sensor::PointCloud& point_cloud, 
    Pose2D pose, ProbabilityGrid* grid) const{
    Pose2D tf = pose;
    sensor::PointCloud transfromed_point_cloud = utils::transformPointCloud(point_cloud, tf);
    for(auto point: transfromed_point_cloud.points){
        updateAPointCloud(point, pose, grid, true);
    }
    for(auto point: transfromed_point_cloud.missing_point){
        updateAPointCloud(point, pose, grid, false);
    }
}
void PointCloudInsertor::updateCell(Eigen::Vector2i cell, bool hit ,ProbabilityGrid* grid) const{
    auto cell_p = grid->getProbability(cell);
    auto p = (hit)? options_->hit_probability: options_->miss_probability;
    auto new_cell_p = 1./odds(odds(p)*odds(cell_p));
    new_cell_p = utils::clamp(new_cell_p, 0.1, 0.9);
    grid->setProbability(cell, new_cell_p);
}
double PointCloudInsertor::odds(double p) const{
    assert(p > 0.);
    return abs(p/(1-p));
}
void PointCloudInsertor::updateAPointCloud(Point32 point, 
    Pose2D pose, ProbabilityGrid* grid, bool hit)const{
    auto cell_list = getCellsInRangeData(point, pose, grid);
    if(!cell_list.empty()){
        for(size_t i = 0; i < cell_list.size() - 1; i++){
            updateCell(cell_list[i], false, grid);
        }
        updateCell(cell_list.back(), hit, grid);
    }
}
std::vector<Eigen::Vector2i> PointCloudInsertor::getCellsInRangeData(
    Point32 point, Pose2D pose, ProbabilityGrid* grid) const{
    std::unordered_set<int64_t> cells_set;
    std::vector<Eigen::Vector2i> cells_list;
    auto delta_x = point.x - pose.x;
    auto delta_y = point.y - pose.y;
    auto length = sqrt(delta_x*delta_x + delta_y*delta_y);
    auto sin_value = delta_y/length;
    auto cos_value = delta_x/length;
    int cnt = (int) (length/grid->getMapMeta().resolution);
    double resolution = grid->getMapMeta().resolution;
    for(int i = 0; i < cnt; i++){
        double x = cos_value*resolution*i + pose.x;
        double y = sin_value*resolution*i + pose.y;
        auto cell = grid->getCell(x,y);
        int64_t x_i = (int64_t)cell[0];
        int64_t y_i = (int64_t)cell[1];
        int64_t key = (x_i << 32) + y_i;
        if (!(cells_set.count(key) > 0)){
            cells_list.push_back(cell);
            cells_set.insert(key);
        }
    }
    return cells_list;
}
}
}
}