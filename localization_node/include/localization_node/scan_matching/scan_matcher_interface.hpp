#ifndef LOCALIZATION_STACK_SCAN_MATCHER_INTERFACE__HPP__
#define LOCALIZATION_STACK_SCAN_MATCHER_INTERFACE__HPP__

#include "localization_node/grid/grid_interface.hpp"

namespace localization_node{
namespace scan_matching{

class ScanMatcherInterface
{
public:
    virtual ~ScanMatcherInterface();

    virtual void updateGrid(grid::GridInterface* grid) = 0;
};

}
}


#endif