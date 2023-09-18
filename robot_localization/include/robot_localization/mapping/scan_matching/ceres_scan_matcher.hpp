#ifndef CERES_SCAN_MATCHING__HPP__
#define CERES_SCAN_MATCHING__HPP__
#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "robot_localization/mapping/sensor/point_cloud.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "ceres/ceres.h"
namespace robot_localization{
namespace mapping{
namespace scan_matching{
enum LossFunc{null_func=0, cauchy=1, huber=2};

struct CeresScanMatcherOptions{
    LossFunc loss = null_func;
    double linear_search_window = 0.1;
    double angular_search_window = 0.2;
    double occupied_space_weight = 1.0;
    double translation_weight = 1.0;
    double rotation_weight = 5.0;
};

class CeresScanMatcher{
public:
explicit CeresScanMatcher(CeresScanMatcherOptions* opitons);
~CeresScanMatcher();

void match(const geometry_msgs::msg::Pose2D target_translation,
            const geometry_msgs::msg::Pose2D initial_pose_estimate,
            const sensor::PointCloud& point_cloud, const grid::ProbabilityGrid& grid,
            geometry_msgs::msg::Pose2D* pose_estimate) const;

void setMatcherOptions(CeresScanMatcherOptions* opitons);
void setCeresOptions(ceres::Solver::Options* opitons);

ceres::Solver::Options* getCeresOptions();
ceres::Solver::Summary* getSummary();
protected:
std::shared_ptr<CeresScanMatcherOptions> options_;
ceres::Solver::Options* ceres_options_;
ceres::Solver::Summary* sum_{nullptr};
};
ceres::CostFunction* createOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const grid::ProbabilityGrid& grid);
}
ceres::CostFunction* translationDeltaCostFunctor2D(const double scaling_factor,
    double target_x, double target_y);
ceres::CostFunction* rotationDeltaCostFunctor2D(const double scaling_factor,
    double target_theta);
}
}
#endif