#ifndef ROTATION_DELTA_COST__HPP__
#define ROTATION_DELTA_COST__HPP__
#include "ceres/ceres.h"
#include "robot_localization/utils/maths.hpp"
namespace robot_localization{
namespace mapping{
namespace scan_matching{
namespace ceres_cost_functor{
// Computes the cost of rotating 'pose' to 'target_angle'. Cost increases with
// the solution's distance from 'target_angle'.
class RotationDeltaCostFunctor2D {
public:
static ceres::CostFunction* createAutoDiffCostFunction(
    const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
}

template <typename T>
bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * utils::norminalAngle(pose[2] - angle_);
    return true;
}

private:
explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                    const double target_angle)
    : scaling_factor_(scaling_factor), angle_(utils::norminalAngle(target_angle)) {}

RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
    delete;

const double scaling_factor_;
const double angle_;
};
}
}
}
}
#endif