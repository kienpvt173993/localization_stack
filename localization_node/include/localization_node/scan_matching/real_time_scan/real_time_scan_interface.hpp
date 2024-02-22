#ifndef LOCALIZATION_STACK_REAL_TIME_SCAN__HPP__
#define LOCALIZATION_STACK_REAL_TIME_SCAN__HPP__

#include "localization_node/scan_matching/scan_matcher_interface.hpp"
#include "localization_node/sensor/point_cloud_interface.hpp"
#include "localization_node/transform/rigid_transform.hpp"

namespace localization_node{
namespace scan_matching{
namespace real_time_scan{   

class RealTimeScanInterface: ScanMatcherInterface
{
public:
    virtual ~RealTimeScanInterface();

    virtual float match(
        const transform::Rigid3f& initial_pose_estimate,
        const sensor::Points& points,
        transform::Rigid3f& pose_estimate) = 0;
};

}
}
}

#endif