#include "robot_localization/mapping/scan_matching/fast_scan_matcher.hpp"
#include <bits/stl_deque.h>
#include <glog/logging.h>
#include <cmath>
#include "robot_localization/utils/maths.hpp"
#include <stl_algobase.h>
#include "absl/memory/memory.h"
#include "robot_localization/utils/transform.hpp"
#include "robot_localization/utils/map_tools.hpp"
using namespace geometry_msgs::msg;
namespace robot_localization{
namespace mapping{
namespace scan_matching{
class SlidingWindowMaximum {
public:
    void addValue(const float value) {
        while (!non_ascending_maxima_.empty() &&
            value > non_ascending_maxima_.back()) {
        non_ascending_maxima_.pop_back();
        }
        non_ascending_maxima_.push_back(value);
    }

    void removeValue(const float value) {
        DCHECK(!non_ascending_maxima_.empty());
        DCHECK_LE(value, non_ascending_maxima_.front());
        if (value == non_ascending_maxima_.front()) {
        non_ascending_maxima_.pop_front();
        }
    }

    float getMaximum() const {
        DCHECK_GT(non_ascending_maxima_.size(), 0);
        return non_ascending_maxima_.front();
    }

    void checkIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

private:
    std::deque<float> non_ascending_maxima_;
};
PrecomputationGrid::PrecomputationGrid(const grid::Grid& grid, 
    int width, std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(grid.getMapMeta()),
      cells_(wide_limits_.width * wide_limits_.height) 
    {
    CHECK_GE(width, 1);
    CHECK_GE(wide_limits_.width, 1);
    CHECK_GE(wide_limits_.height, 1);
    const int stride = wide_limits_.width;
    std::vector<float>& intermediate = *reusable_intermediate_grid;
    auto limits = grid.getMapMeta();
    intermediate.resize(wide_limits_.width * limits.height);
    for (int y = 0; y != limits.height; ++y) {
        SlidingWindowMaximum current_values;
        current_values.addValue(
            1.f - std::abs(1.-grid.getProbability(Eigen::Array2i(0, y))));
        for (int x = -width + 1; x != 0; ++x) {
            intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
            if (x + width < limits.width) {
                current_values.addValue(1.f - std::abs(1.-grid.getProbability(
                                                Eigen::Array2i(x + width, y))));
            }
        }
        for (int x = 0; x < limits.width - width; ++x) {
            intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
            current_values.removeValue(
                1.f - std::abs(1.-grid.getProbability(Eigen::Array2i(x, y))));
            current_values.addValue(1.f - std::abs(1.-grid.getProbability(
                                                Eigen::Array2i(x + width, y))));
        }
        for (int x = (limits.width - width > 0)?limits.width:0;
            x != limits.width; ++x) {
            intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
            current_values.removeValue(
                1.f - std::abs(1.-grid.getProbability(Eigen::Array2i(x, y))));
        }
        current_values.checkIsEmpty();
    }

    for (int x = 0; x != wide_limits_.width; ++x) {
        SlidingWindowMaximum current_values;
        current_values.addValue(intermediate[x]);
        for (int y = -width + 1; y != 0; ++y) {
            cells_[x + (y + width - 1) * stride] =
                computeCellValue(current_values.getMaximum());
            if (y + width < limits.height) {
                current_values.addValue(intermediate[x + (y + width) * stride]);
            }
        }
        for (int y = 0; y < limits.height - width; ++y) {
            cells_[x + (y + width - 1) * stride] =
                computeCellValue(current_values.getMaximum());
            current_values.removeValue(intermediate[x + y * stride]);
            current_values.addValue(intermediate[x + (y + width) * stride]);
        }
        for (int y = (limits.height - width > 0)? limits.height - width:0;
            y != limits.height; ++y) {
            cells_[x + (y + width - 1) * stride] =
                computeCellValue(current_values.getMaximum());
            current_values.removeValue(intermediate[x + y * stride]);
        }
        current_values.checkIsEmpty();
    }
}
uint8_t PrecomputationGrid::computeCellValue(const float probability) const {
    const int cell_value = utils::roundToInt(
    (probability - min_score_) * (255.f / (max_score_ - min_score_)));
    CHECK_GE(cell_value, 0);
    CHECK_LE(cell_value, 255);
    return cell_value;
}

PrecomputationGridStack::PrecomputationGridStack(
    const grid::Grid& grid,
    const FastScanMatcherOptions& options) {
    CHECK_GE(options.branch_and_bound_depth, 1);
    const int max_width = 1 << (options.branch_and_bound_depth - 1);
    precomputation_grids_.reserve(options.branch_and_bound_depth);
    std::vector<float> reusable_intermediate_grid;
    auto limits = grid.getMapMeta();
    reusable_intermediate_grid.reserve((limits.width + max_width - 1) *
                                        limits.height);
    for (int i = 0; i != options.branch_and_bound_depth; ++i) {
        const int width = 1 << i;
        precomputation_grids_.emplace_back(grid, limits, width,
                                        &reusable_intermediate_grid);
    }
}

FastCorrelativeScanMatcher::FastCorrelativeScanMatcher(
    const grid::Grid& grid,
    const FastScanMatcherOptions& options)
    : options_(options),
      limits_(grid.getMapMeta()),
      precomputation_grid_stack_(
          absl::make_unique<PrecomputationGridStack>(grid, options)) {}

FastCorrelativeScanMatcher::~FastCorrelativeScanMatcher() {}

bool FastCorrelativeScanMatcher::match(const geometry_msgs::msg::Pose2D& initial_pose_estimate,
            const sensor::PointCloud& point_cloud, float min_score,
            float* score, geometry_msgs::msg::Pose2D* pose_estimate) const {
    const SearchParameters search_parameters(options_.linear_search_window,
                                            options_.angular_search_window,
                                            point_cloud, limits_.resolution);
    return matchWithSearchParameters(search_parameters, initial_pose_estimate,
                                    point_cloud, min_score, score,
                                    pose_estimate);
}

bool FastCorrelativeScanMatcher::matchFullSubmap(const sensor::PointCloud& point_cloud, float min_score,
                    float* score, geometry_msgs::msg::Pose2D* pose_estimate) const{
    const SearchParameters search_parameters(
        1e6 * limits_.resolution,  // Linear search window, 1e6 cells/direction.
        M_PI,  // Angular search window, 180 degrees in both directions.
        point_cloud, limits_.resolution);
    Pose2D center;
    auto limit_ptr = &limits_;
    center.x = MAP_WXGX(limit_ptr,limits_.width * limits_.resolution / 2.);
    center.y = MAP_WYGY(limit_ptr,limits_.height * limits_.resolution / 2.);
    center.theta = 0.;
    return matchWithSearchParameters(search_parameters, center, point_cloud,
                                    min_score, score, pose_estimate);
}

bool FastCorrelativeScanMatcher::matchWithSearchParameters(
    SearchParameters search_parameters,
    const geometry_msgs::msg::Pose2D& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    geometry_msgs::msg::Pose2D* pose_estimate) const{
    CHECK(score != nullptr);
    CHECK(pose_estimate != nullptr);
    const Eigen::Rotation2Dd initial_rotation(initial_pose_estimate.theta);
    const sensor::PointCloud rotated_point_cloud = 
        utils::rotatePointCloud(point_cloud, initial_pose_estimate.theta);
    const std::vector<sensor::PointCloud> rotated_scans =
        generateRotatedScans(rotated_point_cloud, search_parameters);
    const std::vector<DiscreteScan> discrete_scans = discretizeScans(
        limits_, rotated_scans,
        Eigen::Translation2f(initial_pose_estimate.x,
                            initial_pose_estimate.y));
    search_parameters.shrinkToFit(discrete_scans, limits_);

    const std::vector<Candidate> lowest_resolution_candidates =
        computeLowestResolutionCandidates(discrete_scans, search_parameters);
    const Candidate best_candidate = branchAndBound(
        discrete_scans, search_parameters, lowest_resolution_candidates,
        precomputation_grid_stack_->maxDepth(), min_score);
    if (best_candidate.score > min_score) {
        *score = best_candidate.score;
        pose_estimate->x = initial_pose_estimate.x + best_candidate.x;
        pose_estimate->y = initial_pose_estimate.y + best_candidate.y;
        pose_estimate->theta = (initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation)).angle();
        return true;
    }
    return false;
}

std::vector<Candidate>
FastCorrelativeScanMatcher::computeLowestResolutionCandidates(
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters) const {
    std::vector<Candidate> lowest_resolution_candidates =
        generateLowestResolutionCandidates(search_parameters);
    scoreCandidates(
        precomputation_grid_stack_->get(precomputation_grid_stack_->maxDepth()),
        discrete_scans, search_parameters, &lowest_resolution_candidates);
    return lowest_resolution_candidates;
}

std::vector<Candidate>
FastCorrelativeScanMatcher::generateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
    const int linear_step_size = 1 << precomputation_grid_stack_->maxDepth();
    int num_candidates = 0;
    for (int scan_index = 0; scan_index != search_parameters.num_scans;
        ++scan_index) {
        const int num_lowest_resolution_linear_x_candidates =
            (search_parameters.linear_bounds[scan_index].max_x -
            search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
            linear_step_size;
        const int num_lowest_resolution_linear_y_candidates =
            (search_parameters.linear_bounds[scan_index].max_y -
            search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
            linear_step_size;
        num_candidates += num_lowest_resolution_linear_x_candidates *
                        num_lowest_resolution_linear_y_candidates;
    }
    std::vector<Candidate> candidates;
    candidates.reserve(num_candidates);
    for (int scan_index = 0; scan_index != search_parameters.num_scans;
        ++scan_index) {
        for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
            x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
            x_index_offset += linear_step_size) {
            for (int y_index_offset =
                    search_parameters.linear_bounds[scan_index].min_y;
                y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
                y_index_offset += linear_step_size) {
                candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                        search_parameters);
            }
        }
    }
    CHECK_EQ(candidates.size(), num_candidates);
    return candidates;
}

void FastCorrelativeScanMatcher::scoreCandidates(
    const PrecomputationGrid& precomputation_grid,
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate>* const candidates) const {
    for (Candidate& candidate : *candidates) {
        int sum = 0;
        for (const Eigen::Array2i& xy_index :
            discrete_scans[candidate.scan_index]) {
            const Eigen::Array2i proposed_xy_index(
                xy_index.x() + candidate.x_index_offset,
                xy_index.y() + candidate.y_index_offset);
            sum += precomputation_grid.getValue(proposed_xy_index);
        }
        candidate.score = precomputation_grid.toScore(
            sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
    }
    std::sort(candidates->begin(), candidates->end(),
                std::greater<Candidate>());
}

Candidate FastCorrelativeScanMatcher::branchAndBound(
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate>& candidates, const int candidate_depth,
    float min_score) const {
    if (candidate_depth == 0) {
        // Return the best candidate.
        return *candidates.begin();
    }

    Candidate best_high_resolution_candidate(0, 0, 0, search_parameters);
    best_high_resolution_candidate.score = min_score;
    for (const Candidate& candidate : candidates) {
        if (candidate.score <= min_score) {
            break;
        }
        std::vector<Candidate> higher_resolution_candidates;
        const int half_width = 1 << (candidate_depth - 1);
        for (int x_offset : {0, half_width}) {
            if (candidate.x_index_offset + x_offset >
                search_parameters.linear_bounds[candidate.scan_index].max_x) {
                break;
            }
            for (int y_offset : {0, half_width}) {
                if (candidate.y_index_offset + y_offset >
                    search_parameters.linear_bounds[candidate.scan_index].max_y) {
                    break;
                }
                higher_resolution_candidates.emplace_back(
                    candidate.scan_index, candidate.x_index_offset + x_offset,
                    candidate.y_index_offset + y_offset, search_parameters);
            }
        }
        scoreCandidates(precomputation_grid_stack_->get(candidate_depth - 1),
                        discrete_scans, search_parameters,
                        &higher_resolution_candidates);
        best_high_resolution_candidate = std::max(
            best_high_resolution_candidate,
            branchAndBound(discrete_scans, search_parameters,
                        higher_resolution_candidates, candidate_depth - 1,
                        best_high_resolution_candidate.score));
    }
    return best_high_resolution_candidate;
}

}}}