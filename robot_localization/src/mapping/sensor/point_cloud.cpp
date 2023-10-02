#include "robot_localization/mapping/sensor/point_cloud.hpp"
using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
namespace robot_localization{
namespace mapping{
namespace sensor{
PointCloud::PointCloud(LaserScan* scan, double max_range, double min_range){
    assert(max_range>min_range);
    assert(min_range>=0);
    convertLaserScanToPointCloud(scan, max_range, min_range);
}
PointCloud::PointCloud(){}
PointCloud::~PointCloud(){}
size_t PointCloud::size() const{
    return this->intensities.size();
}
void PointCloud::convertLaserScanToPointCloud(LaserScan* scan,
    double max_range, double min_range){
    auto min = scan->angle_min;
    auto incr = scan->angle_increment;
    this->header = scan->header;
    for(size_t i = 0; i < scan->ranges.size(); i++){
        if(scan->ranges[i] <= max_range && scan->ranges[i] >= min_range){
            double angle = min + incr*i;
            double x = scan->ranges[i]*cos(angle);
            double y = scan->ranges[i]*sin(angle);
            if(i < scan->intensities.size())
                pushBack(x, y, scan->intensities[i]);
            else
                pushBack(x, y, 0.);
        }
        else if(scan->ranges[i] > max_range){
            double angle = min + incr*i;
            Point32 point;
            point.x = max_range*cos(angle);
            point.y = max_range*sin(angle);
            missing_point.push_back(point);
        }
    }
}
void PointCloud::clear(){
    this->points.clear();
    this->intensities.clear();
}
bool PointCloud::empty() const{
    return this->intensities.empty();
}
void PointCloud::pushBack(double x, double y, float intensity){
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    this->points.push_back(point);
    this->intensities.push_back(intensity);
}
}}}