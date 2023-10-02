#ifndef POINT_CLOUD__HPP__
#define POINT_CLOUD__HPP__
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace robot_localization{
namespace mapping{
namespace sensor{
class PointCloud: public sensor_msgs::msg::PointCloud{
public:
explicit PointCloud(sensor_msgs::msg::LaserScan* scan, 
    double max_range, double min_range);
PointCloud();
~PointCloud();
size_t size() const;
void clear();
bool empty() const;
std::vector<float> intensities;
void pushBack(double x, double y, float intensity);
std::vector<geometry_msgs::msg::Point32> missing_point;
protected:
void convertLaserScanToPointCloud(
    sensor_msgs::msg::LaserScan* scan,
    double max_range, double min_range);
};
}}}
#endif