#ifndef OCCUPIED_SPACE_COST__HPP__
#define OCCUPIED_SPACE_COST__HPP__
#include "ceres/cubic_interpolation.h"
#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "robot_localization/mapping/sensor/point_cloud.hpp"
#include "robot_localization/utils/maths.hpp"
#define kMaxCorrespondenceCost 0.9
using namespace nav_msgs::msg;
namespace robot_localization{
namespace mapping{
namespace scan_matching{
namespace ceres_cost_functor{
class OccupiedSpaceCostFunction {
 public:
  OccupiedSpaceCostFunction(const double scaling_factor,
                              const sensor::PointCloud& point_cloud,
                              const grid::ProbabilityGrid& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapMetaData& limits = grid_.limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_.points[i].x)),
                                         (T(point_cloud_.points[i].y)),
                                         T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(
          (limits.width - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.height - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const grid::ProbabilityGrid& grid) : grid_(grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = kMaxCorrespondenceCost;
      } else {
        *value = static_cast<double>(1.0 - grid_.getProbability(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return grid_.getMapMeta().width + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.getMapMeta().height + 2 * kPadding;
    }

   private:
    const grid::ProbabilityGrid& grid_;
  };

  OccupiedSpaceCostFunction(const OccupiedSpaceCostFunction&) = delete;
  OccupiedSpaceCostFunction& operator=(const OccupiedSpaceCostFunction&) =
      delete;

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const grid::ProbabilityGrid& grid_;
};
inline ceres::CostFunction* CreateOccupiedSpaceCostFunction(
    const double scaling_factor,const sensor::PointCloud& point_cloud,
    const grid::ProbabilityGrid& grid) {
    return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction,
                                            ceres::DYNAMIC /* residuals */,
                                            3 /* pose variables */>(
        new OccupiedSpaceCostFunction(scaling_factor, point_cloud, grid),
        point_cloud.size());
}
}}}}
#endif