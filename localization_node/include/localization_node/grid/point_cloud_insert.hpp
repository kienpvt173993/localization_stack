#ifndef LOCALIZATION_STACK_POINT_CLOUD_INSERT__HPP__
#define LOCALIZATION_STACK_POINT_CLOUD_INSERT__HPP__

#include "localization_node/grid/point_cloud_insert_interface.hpp"

namespace localization_node{
namespace grid{

class PointCloudInsertInsert: PointCloudInsertInterface
{
public:
    PointCloudInsertInsert(const PointCloudInsertOption& option);
    ~PointCloudInsertInsert();
    void insert(GridInterface* grid, const transform::Rigid3f& origin, 
        const sensor::PointCloudInterface& points) const override final;
    PointCloudInsertOption& getOptions() override final;
protected:
    PointCloudInsertOption options_;
};

}
}

#endif