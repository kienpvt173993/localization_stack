#include "robot_localization/mapping/scan_matching/correlative_scan_matcher.hpp"
#include "robot_localization/utils/transform.hpp"
#include <bits/stl_algobase.h>
using namespace geometry_msgs::msg;
namespace robot_localization{
namespace mapping{
namespace scan_matching{
SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const sensor::PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
    float max_scan_range = 3.f * resolution;
    for (const auto point_scan : point_cloud.points) {
        const float range = sqrt(pow(point_scan.x, 2.) + pow(point_scan.y, 2.));
        max_scan_range = std::max(range, max_scan_range);
    }
    const double kSafetyMargin = 1. - 1e-3;
    angular_perturbation_step_size =
        kSafetyMargin * std::acos(1. - pow(resolution,2.) /
                                            (2. * pow(max_scan_range,2.)));
    num_angular_perturbations =
        std::ceil(angular_search_window / angular_perturbation_step_size);
    num_scans = 2 * num_angular_perturbations + 1;

    const int num_linear_perturbations =
        std::ceil(linear_search_window / resolution);
    linear_bounds.reserve(num_scans);
    for (int i = 0; i != num_scans; ++i) {
        linear_bounds.push_back(
            LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                        -num_linear_perturbations, num_linear_perturbations});
    }
}

SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
    linear_bounds.reserve(num_scans);
    for (int i = 0; i != num_scans; ++i) {
        linear_bounds.push_back(
            LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                        -num_linear_perturbations, num_linear_perturbations});
    }
}

void SearchParameters::shrinkToFit(const std::vector<DiscreteScan>& scans,
                                   const nav_msgs::msg::MapMetaData& cell_limits) {
    assert(scans.size() == num_scans);
    assert(linear_bounds.size() == num_scans);
    for (int i = 0; i != num_scans; ++i) {
        Eigen::Array2i min_bound = Eigen::Array2i::Zero();
        Eigen::Array2i max_bound = Eigen::Array2i::Zero();
        for (const Eigen::Array2i& xy_index : scans[i]) {
        min_bound = min_bound.min(-xy_index);
        max_bound = max_bound.max(Eigen::Array2i(cell_limits.width - 1,
                                                cell_limits.height - 1) -
                                    xy_index);
        }
        linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
        linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
        linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
        linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
    }
}

std::vector<sensor::PointCloud> generateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters) {
    std::vector<sensor::PointCloud> rotated_scans;
    rotated_scans.reserve(search_parameters.num_scans);

    double delta_theta = -search_parameters.num_angular_perturbations *
                        search_parameters.angular_perturbation_step_size;
    for (int scan_index = 0; scan_index < search_parameters.num_scans;
        ++scan_index,
            delta_theta += search_parameters.angular_perturbation_step_size) {
        rotated_scans.push_back(utils::rotatePointCloud(point_cloud, delta_theta));
    }
    return rotated_scans;
}

std::vector<DiscreteScan> discretizeScans(
    const grid::Grid& grid, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation) {
    std::vector<DiscreteScan> discrete_scans;
    discrete_scans.reserve(scans.size());
    for (const sensor::PointCloud& scan : scans) {
        discrete_scans.emplace_back();
        discrete_scans.back().reserve(scan.size());
        for (const auto& point_scan : scan.points) {
            Eigen::Vector3f point;
            point << point_scan.x, point_scan.y, 0.f;
          const Eigen::Vector2f translated_point =
              Eigen::Affine2f(initial_translation) * point.head<2>();
          discrete_scans.back().push_back(
              grid.getCell(translated_point));
        }
    }
    return discrete_scans;
}
}}}