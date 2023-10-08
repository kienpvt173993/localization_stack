#ifndef FAST_SCAN_MATCHER__HPP__
#define FAST_SCAN_MATCHER__HPP__
#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "robot_localization/mapping/sensor/point_cloud.hpp"
#include "robot_localization/mapping/scan_matching/correlative_scan_matcher.hpp"
namespace robot_localization{
namespace mapping{
namespace scan_matching{
class FastScanMatcherOptions{
    double linear_search_window = 0.1;
    double angular_search_window = 0.2;
    int branch_and_bound_depth = 6;
};

class PrecomputationGrid {
public:
PrecomputationGrid(const grid::ProbabilityGrid& grid, int width, std::vector<float>* reusable_intermediate_grid);
int GetValue(const Eigen::Array2i& xy_index) const {
    const Eigen::Array2i local_xy_index = xy_index - offset_;
    if (static_cast<unsigned>(local_xy_index.x()) >=
            static_cast<unsigned>(wide_limits_.width) ||
        static_cast<unsigned>(local_xy_index.y()) >=
            static_cast<unsigned>(wide_limits_.height)) {
        return 0;
    }
    const int stride = wide_limits_.width;
    return cells_[local_xy_index.x() + local_xy_index.y() * stride];
}
float ToScore(float value) const {
return min_score_ + value * ((max_score_ - min_score_) / 255.f);
}
private:
uint8_t ComputeCellValue(float probability) const;
const Eigen::Array2i offset_;
const nav_msgs::msg::MapMetaData wide_limits_;
const float min_score_;
const float max_score_;
std::vector<uint8_t> cells_;
};
class PrecomputationGridStack2D {
public:
PrecomputationGridStack2D(
    const grid::ProbabilityGrid& grid,
    const FastScanMatcherOptions& options);

const PrecomputationGrid& Get(int index) {
return precomputation_grids_[index];
}
int max_depth() const { return precomputation_grids_.size() - 1; }
private:
std::vector<PrecomputationGrid> precomputation_grids_;
};
class FastCorrelativeScanMatcher {
public:
FastCorrelativeScanMatcher(
    const grid::ProbabilityGrid& grid,
    const FastScanMatcherOptions& options);
~FastCorrelativeScanMatcher();

FastCorrelativeScanMatcher(const FastCorrelativeScanMatcher&) = delete;
FastCorrelativeScanMatcher& operator=(const FastCorrelativeScanMatcher&) =
    delete;
bool match(const geometry_msgs::msg::Pose2D& initial_pose_estimate,
            const sensor::PointCloud& point_cloud, float min_score,
            float* score, geometry_msgs::msg::Pose2D* pose_estimate) const;
bool matchFullSubmap(const sensor::PointCloud& point_cloud, float min_score,
                    float* score, geometry_msgs::msg::Pose2D* pose_estimate) const;

private:
bool matchWithSearchParameters(
    SearchParameters search_parameters,
    const geometry_msgs::msg::Pose2D& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    geometry_msgs::msg::Pose2D* pose_estimate) const;
std::vector<Candidate> computeLowestResolutionCandidates(
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters) const;
std::vector<Candidate> generateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const;
void scoreCandidates(const PrecomputationGrid& precomputation_grid,
                    const std::vector<DiscreteScan>& discrete_scans,
                    const SearchParameters& search_parameters,
                    std::vector<Candidate>* const candidates) const;
Candidate branchAndBound(const std::vector<DiscreteScan>& discrete_scans,
                            const SearchParameters& search_parameters,
                            const std::vector<Candidate>& candidates,
                            int candidate_depth, float min_score) const;
const FastScanMatcherOptions options_;
nav_msgs::msg::MapMetaData limits_;
std::unique_ptr<PrecomputationGridStack2D> precomputation_grid_stack_;
};
}}}
#endif