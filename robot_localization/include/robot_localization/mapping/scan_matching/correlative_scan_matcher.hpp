#ifndef CORRELATIVE_SCAN_MATCHER__HPP__
#define CORRELATIVE_SCAN_MATCHER__HPP__
#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "robot_localization/mapping/sensor/point_cloud.hpp"
namespace robot_localization{
namespace mapping{
namespace scan_matching{
typedef std::vector<Eigen::Array2i> DiscreteScan;

// Describes the search space.
struct SearchParameters {
struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
};
SearchParameters(double linear_search_window, double angular_search_window,
                const sensor::PointCloud& point_cloud, double resolution);
SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                double angular_perturbation_step_size, double resolution);
void shrinkToFit(const std::vector<DiscreteScan>& scans,
                const nav_msgs::msg::MapMetaData& cell_limits);
int num_angular_perturbations;
double angular_perturbation_step_size;
double resolution;
int num_scans;
std::vector<LinearBounds> linear_bounds;
};
std::vector<sensor::PointCloud> generateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);
std::vector<DiscreteScan> discretizeScans(
    const nav_msgs::msg::MapMetaData& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);
struct Candidate {
Candidate(const int init_scan_index, const int init_x_index_offset,
        const int init_y_index_offset,
        const SearchParameters& search_parameters)
    : scan_index(init_scan_index),
    x_index_offset(init_x_index_offset),
    y_index_offset(init_y_index_offset),
    x(-y_index_offset * search_parameters.resolution),
    y(-x_index_offset * search_parameters.resolution),
    orientation((scan_index - search_parameters.num_angular_perturbations) *
                search_parameters.angular_perturbation_step_size) {}
int scan_index = 0;
int x_index_offset = 0;
int y_index_offset = 0;
double x = 0.;
double y = 0.;
double orientation = 0.;
float score = 0.f;
bool operator<(const Candidate& other) const { return score < other.score; }
bool operator>(const Candidate& other) const { return score > other.score; }
};
}}}
#endif