#ifndef REAL_TIME_SCAN_MATCHER__HPP__
#define REAL_TIME_SCAN_MATCHER__HPP__
#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "robot_localization/mapping/sensor/point_cloud.hpp"
#include "robot_localization/mapping/scan_matching/correlative_scan_matcher.hpp"
namespace robot_localization{
namespace mapping{
namespace scan_matching{

struct RealTimeCorrelativeScanMatcherOptions{
    double linear_search_window=0.6;
    double angular_search_window=0.16;
    double translation_delta_cost_weight=0.;
    double rotation_delta_cost_weight=0.;
};


class RealTimeCorrelativeScanMatcher{
public:
    explicit RealTimeCorrelativeScanMatcher(
        const RealTimeCorrelativeScanMatcherOptions& options);

    RealTimeCorrelativeScanMatcher(const RealTimeCorrelativeScanMatcher&) =
        delete;
    RealTimeCorrelativeScanMatcher& operator=(
        const RealTimeCorrelativeScanMatcher&) = delete;
    double match(const geometry_msgs::msg::Pose2D& initial_pose_estimate,
                const sensor::PointCloud& point_cloud, const grid::Grid& grid,
                geometry_msgs::msg::Pose2D* pose_estimate) const;

    void scoreCandidates(const grid::Grid& grid,
                        const std::vector<DiscreteScan>& discrete_scans,
                        const SearchParameters& search_parameters,
                        std::vector<Candidate>* candidates) const;

private:
    std::vector<Candidate> generateExhaustiveSearchCandidates(
        const SearchParameters& search_parameters) const;
    const RealTimeCorrelativeScanMatcherOptions options_;
};
}
}
}
#endif