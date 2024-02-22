#ifndef LOCALIZATION_STACK_FAST_SCAN_INTERFACE__HPP__
#define LOCALIZATION_STACK_FAST_SCAN_INTERFACE__HPP__

#include "localization_node/scan_matching/scan_matcher_interface.hpp"
#include "localization_node/sensor/point_cloud_interface.hpp"
#include "localization_node/transform/rigid_transform.hpp"

namespace localization_node{
namespace scan_matching{
namespace fast_scan{

class FastScanInterface: ScanMatcherInterface
{
public:
    virtual ~FastScanInterface();

    virtual bool match(
        const transform::Rigid3f& initial_pose_estimate,
        const sensor::Points& points,
        const float& min_score, float& score,
        transform::Rigid3f& pose_estimate) = 0;
};

}
}
}

#endif