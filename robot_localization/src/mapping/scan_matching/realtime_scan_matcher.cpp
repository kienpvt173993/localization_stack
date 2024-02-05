#include "robot_localization/mapping/scan_matching/realtime_scan_matcher.hpp"
#include "glog/logging.h"
#include "robot_localization/utils/transform.hpp"
namespace robot_localization{
namespace mapping{
namespace scan_matching{
double computeCandidateScore(const grid::Grid& probability_grid,
                            const DiscreteScan& discrete_scan,
                            int x_index_offset, int y_index_offset) {
    double candidate_score = 0.f;
    for (const Eigen::Array2i& xy_index : discrete_scan) {
        const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                            xy_index.y() + y_index_offset);
        const double probability =
            probability_grid.getProbability(proposed_xy_index);
        candidate_score += probability;
    }
    candidate_score /= static_cast<double>(discrete_scan.size());
    CHECK_GT(candidate_score, 0.f);
    return candidate_score;
}

RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher(
    const RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

std::vector<Candidate>
RealTimeCorrelativeScanMatcher::generateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
    int num_candidates = 0;
    for (int scan_index = 0; scan_index != search_parameters.num_scans;
        ++scan_index) {
        const int num_linear_x_candidates =
            (search_parameters.linear_bounds[scan_index].max_x -
            search_parameters.linear_bounds[scan_index].min_x + 1);
        const int num_linear_y_candidates =
            (search_parameters.linear_bounds[scan_index].max_y -
            search_parameters.linear_bounds[scan_index].min_y + 1);
        num_candidates += num_linear_x_candidates * num_linear_y_candidates;
    }
    std::vector<Candidate> candidates;
    candidates.reserve(num_candidates);
    for (int scan_index = 0; scan_index != search_parameters.num_scans;
        ++scan_index) {
        for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
            x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
            ++x_index_offset) {
            for (int y_index_offset =
                    search_parameters.linear_bounds[scan_index].min_y;
                y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
                ++y_index_offset) {
                candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                        search_parameters);
            }
        }
    }
    CHECK_EQ(candidates.size(), num_candidates);
    return candidates;
}

double RealTimeCorrelativeScanMatcher::match(
    const geometry_msgs::msg::Pose2D& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const grid::Grid& grid,
    geometry_msgs::msg::Pose2D* pose_estimate) const {
    CHECK(pose_estimate != nullptr);

    const Eigen::Rotation2Dd initial_rotation(initial_pose_estimate.theta);
    const sensor::PointCloud rotated_point_cloud = utils::rotatePointCloud(
        point_cloud, initial_rotation.angle());
    const SearchParameters search_parameters(
        options_.linear_search_window, options_.angular_search_window,
        rotated_point_cloud, grid.getMapMeta().resolution);

    const std::vector<sensor::PointCloud> rotated_scans =
        generateRotatedScans(rotated_point_cloud, search_parameters);
    const std::vector<DiscreteScan> discrete_scans = discretizeScans(
        grid.getMapMeta(), rotated_scans,
        Eigen::Translation2f(initial_pose_estimate.x,
                            initial_pose_estimate.y));
    std::vector<Candidate> candidates =
        generateExhaustiveSearchCandidates(search_parameters);
    scoreCandidates(grid, discrete_scans, search_parameters, &candidates);

    const Candidate& best_candidate =
        *std::max_element(candidates.begin(), candidates.end());
    *pose_estimate = utils::createPose2D(initial_pose_estimate.x + best_candidate.x,
        initial_pose_estimate.y + best_candidate.y, 
        (initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation)).angle());
    return best_candidate.score;
}

void RealTimeCorrelativeScanMatcher::scoreCandidates(
    const grid::Grid& grid, const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate>* const candidates) const {
    for (Candidate& candidate : *candidates) {
        candidate.score = computeCandidateScore(grid,
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
        candidate.score *=
            std::exp(pow(std::hypot(candidate.x, candidate.y) *
                                    options_.translation_delta_cost_weight +
                                std::abs(candidate.orientation) *
                                    options_.rotation_delta_cost_weight, 2.));
    }
}
}}}