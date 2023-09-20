#ifndef TRANSLATION_DELTA_COST__HPP__
#define TRANSLATION_DELTA_COST__HPP__
#include "ceres/ceres.h"
namespace robot_localization{
namespace mapping{
namespace scan_matching{
namespace ceres_cost_functor{
// Computes the cost of translating 'pose' to 'target_translation'.
// Cost increases with the solution's distance from 'target_translation'.
class TranslationDeltaCostFunctor2D {
public:
static ceres::CostFunction* createAutoDiffCostFunction(
    const double scaling_factor, const double x, const double y) {
return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                        2 /* residuals */,
                                        3 /* pose variables */>(
    new TranslationDeltaCostFunctor2D(scaling_factor, x, y));
}

template <typename T>
bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
}

private:
// Constructs a new TranslationDeltaCostFunctor2D from the given
// 'target_translation' (x, y).
explicit TranslationDeltaCostFunctor2D(
    const double scaling_factor, const double x, const double y)
    : scaling_factor_(scaling_factor),
        x_(x),y_(y) {}

TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
TranslationDeltaCostFunctor2D& operator=(
    const TranslationDeltaCostFunctor2D&) = delete;

const double scaling_factor_;
const double x_;
const double y_;
};
}
}
}
}
#endif