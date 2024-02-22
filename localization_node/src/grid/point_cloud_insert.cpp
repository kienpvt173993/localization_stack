#include "localization_node/grid/point_cloud_insert.hpp"

namespace localization_node{
namespace grid{

PointCloudInsertInsert::PointCloudInsertInsert(const PointCloudInsertOption& option)
:options_(option){}

PointCloudInsertInsert::~PointCloudInsertInsert(){}

void PointCloudInsertInsert::insert(GridInterface* grid, const transform::Rigid3f& origin, 
    const sensor::PointCloudInterface& points) const{
    
}

PointCloudInsertOption& PointCloudInsertInsert::getOptions(){
    return options_;
}
}
}