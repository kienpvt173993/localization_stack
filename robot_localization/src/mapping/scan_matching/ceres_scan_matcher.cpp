#include "robot_localization/mapping/scan_matching/ceres_scan_matcher.hpp"
namespace robot_localization{
namespace mapping{
namespace scan_matching{
CeresScanMatcher::CeresScanMatcher(CeresScanMatcherOptions* options){
    setMatcherOptions(options);
    ceres::Solver::Options* ceres_options_;
    ceres_options_ = new ceres::Solver::Options();
    ceres_options_->use_nonmonotonic_steps = false;
    ceres_options_->max_num_iterations = 5;
    ceres_options_->num_threads = 2;
}
void CeresScanMatcher::setMatcherOptions(CeresScanMatcherOptions* options){
    assert(options != nullptr);
    options_.reset(options);
}
void CeresScanMatcher::setCeresOptions(ceres::Solver::Options* opitons){
    assert(opitons != nullptr);
    ceres_options_ = opitons;
}
ceres::Solver::Options* CeresScanMatcher::getCeresOptions(){return ceres_options_;}
ceres::Solver::Summary* CeresScanMatcher::getSummary(){return sum_;}
void CeresScanMatcher::match(const geometry_msgs::msg::Pose2D target_translation,
    const geometry_msgs::msg::Pose2D initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const grid::ProbabilityGrid& grid,
    geometry_msgs::msg::Pose2D* pose_estimate) const{
    double ceres_pose_estimate[3] = {initial_pose_estimate.x,
                                    initial_pose_estimate.y,
                                    initial_pose_estimate.theta};
    ceres::Problem problem;
    assert(options_->occupied_space_weight > 0.);
    assert(options_->translation_weight > 0.);
    assert(options_->rotation_weight > 0.);
    assert(options_->linear_search_window > 0.);
    assert(options_->angular_search_window > 0.);
    ceres::LossFunction* loss_func;
    if(options_->loss == null_func)
        loss_func = nullptr;
    else if(options_->loss == cauchy)
        loss_func = new ceres::HuberLoss(0.7);
    else if(options_->loss == huber)
        loss_func = new ceres::CauchyLoss(0.7);
}
}}}