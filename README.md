# Robot Localization Package
A simple version of cartographer for localization with static map: 
    [+]ceres scan matching
    [+]real-time scan matching
    [+]fast scan matching
    [+]long-life mapping
# API
The following are the services/topics that are exposed for use.
## Subscribed topics
| /scan  | `sensor_msgs/LaserScan` | the input scan from your laser to utilize | 
|-----|----|----|
| /map   | `nav_msgs/OccupancyGrid` | static map to localization | 
|-----|----|----|
| **tf** | N/A | a valid transform from your configured odom_frame to base_frame |
## Published topics
| Topic       | Type | Description | 
|-----|----|----|
| update_map  | `nav_msgs/OccupancyGrid` | occupancy grid representation at `map_update_interval` frequency | 
| pose        | `geometry_msgs/PoseWithCovarianceStamped` | pose of the base_frame in the configured map_frame along with the covariance (fitting percent) calculated from the scan match |
| score       | `std_msgs/Float64` | Fitting score (percent) with static map|
| tf          | N/A | TF from global frame to odom frame|
# In future:
    [+]Localization with landmarker: Reflector marker, V marker, V-Line Marker, etc
    [+]3D lidar
# contact me:
gmail: phantrungkien240999@gmail.com