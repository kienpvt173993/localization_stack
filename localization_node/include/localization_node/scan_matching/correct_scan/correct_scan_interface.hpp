#ifndef LOCALIZATION_STACK_CORRECT_SCAN_INTERFACE__HPP__
#define LOCALIZATION_STACK_CORRECT_SCAN_INTERFACE__HPP__

#include "localization_node/scan_matching/scan_matcher_interface.hpp"
#include "localization_node/sensor/point_cloud_interface.hpp"
#include "localization_node/transform/rigid_transform.hpp"

namespace localization_node{
namespace scan_matching{
namespace correct_scan{

class CorrectScanInterface: ScanMatcherInterface
{
public:
    virtual ~CorrectScanInterface();

    virtual void match(const transform::Rigid3f::Vector& target_translation,
        const transform::Rigid3f& initial_pose_estimate,
        const sensor::Points& points,
        float& score,
        transform::Rigid3f& pose_estimate) = 0;
};


}
}
}

#endif