#ifndef LOCALIZATION_STACK_POINT_CLOUD_INSERT_INTERFACE__HPP__
#define LOCALIZATION_STACK_POINT_CLOUD_INSERT_INTERFACE__HPP__

#include "localization_node/grid/grid_interface.hpp"
#include "localization_node/transform/rigid_transform.hpp"
#include "localization_node/sensor/point_cloud_interface.hpp"

namespace localization_node{
namespace grid{

struct PointCloudInsertOption{
    double hit_probability = 0.8;
    double miss_probability = 0.3;
};

class PointCloudInsertInterface
{
public:
    virtual ~PointCloudInsertInterface();
    virtual void insert(GridInterface* grid, const transform::Rigid3f& origin, 
        const sensor::PointCloudInterface& points) const = 0;
    virtual PointCloudInsertOption& getOptions() = 0;
};

}
}

#endif