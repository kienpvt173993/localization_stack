#include "localization_node/sensor/point_cloud_2d.hpp"
#include "localization_node/utils/point_tranform.hpp"

namespace localization_node{
namespace sensor{

Point convertToPoint(const float& range, const float& angle){
    return Point({range*cos(angle), range*sin(angle), 0.f});
}

PointCloud2D::PointCloud2D(const sensor_msgs::msg::LaserScan& sensor,
    const transform::Rigid3f& origin){
    if (sensor.intensities.size() != sensor.ranges.size()){
        intensity_.resize(sensor.ranges.size(), 0.f);
    }
    else{
        intensity_ = sensor.intensities;
    }
    for(size_t i = 0; i < sensor.ranges.size(); i++){
        float angle = sensor.angle_increment*i + sensor.angle_min;
        float range = sensor.ranges[i];
        auto point = convertToPoint(range, angle);
        origin_data_.push_back(point);
        return_data_.push_back(point);
    }
    this->transform(origin);
}

PointCloud2D::PointCloud2D(const Points& points){
    return_data_ = points;
    origin_data_ = points;
    intensity_.resize(return_data_.size(), 0.f);
}

PointCloud2D::PointCloud2D(const sensor_msgs::msg::LaserScan& sensor,
    const transform::Rigid3f& origin, const float& min, const float& max){
    if (sensor.intensities.size() != sensor.ranges.size()){
        intensity_.resize(sensor.ranges.size(), 0.f);
    }
    else{
        intensity_ = sensor.intensities;
    }
    for(size_t i = 0; i < sensor.ranges.size(); i++){
        float angle = sensor.angle_increment*i + sensor.angle_min;
        float range = sensor.ranges[i];
        auto point = convertToPoint(range, angle);
        if ((range >= min) && (range <= max)){
            return_data_.push_back(point);
        }
        else if(range > max){
            auto miss = convertToPoint(max, angle);
            miss_data_.push_back(miss);
        }
        origin_data_.push_back(point);
    }
}

PointCloud2D::~PointCloud2D(){

}

void PointCloud2D::clear(){
    origin_data_.clear();
    return_data_.clear();
    miss_data_.clear();
}

bool PointCloud2D::empty() const{
    return origin_data_.empty();
}

void PointCloud2D::pushBack(Point point ,float intensity){
    origin_data_.push_back(point);
    return_data_.push_back(point);
    intensity_.push_back(intensity);
}

void PointCloud2D::pushBack(geometry_msgs::msg::Point32 point , float intensity){
    origin_data_.push_back({point.x, point.y, point.z});
    return_data_.push_back({point.x, point.y, point.z});
    intensity_.push_back(intensity);
}

sensor_msgs::msg::PointCloud PointCloud2D::getRosPointCloud() const{
    sensor_msgs::msg::PointCloud pcl;
    sensor_msgs::msg::ChannelFloat32 intensity_channel;
    intensity_channel.name = "intensity";
    for(size_t i = 0; i < origin_data_.size(); i++){
        auto point = origin_data_[i];
        auto intensity = intensity_[i];
        auto new_point = geometry_msgs::msg::Point32();
        new_point.x = point.x();
        new_point.y = point.y();
        new_point.z = point.z();
        pcl.points.push_back(new_point);
        intensity_channel.values.push_back(intensity);
    }
    pcl.channels.push_back(intensity_channel);
}

Points PointCloud2D::getPointCloud() const{
    return origin_data_;
}

Points PointCloud2D::getMissing() const{
    return miss_data_;
}

Points PointCloud2D::getReturn() const{
    return return_data_;
}

void PointCloud2D::transform(const transform::Rigid3f& tf){
    utils::transformPoints(this->origin_data_, tf);
    utils::transformPoints(this->return_data_, tf);
    utils::transformPoints(this->miss_data_, tf);
}

}}