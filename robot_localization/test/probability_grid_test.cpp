#include "robot_localization/mapping/grid/probability_grid.hpp"
#include "gtest/gtest.h"
using namespace robot_localization::mapping::grid;
using namespace nav_msgs::msg;
std::vector<int8_t> map_data = {100,100,0,100,
                                100,0,100,0,
                                -1,100,0,100,
                                0,-1,0,100};
Grid createGrid(){
    OccupancyGrid ros_map;
    ros_map.info.width = 4;
    ros_map.info.height = 4;
    ros_map.info.resolution = 0.05;
    ros_map.info.origin.position.x = 0.05;
    ros_map.info.origin.position.y = 0.05;
    ros_map.info.origin.orientation.w = 1.;
    ros_map.data = map_data;
    Grid grid(&ros_map);
    return grid;
}

TEST(Grid2DTest, get_cell){
    auto grid = createGrid();
    auto cell = grid.getCell(0.1,0.2);
    EXPECT_EQ(cell[0], 3);
    EXPECT_EQ(cell[1], 5);
}

TEST(Grid2DTest, data){
    auto grid = createGrid();
    EXPECT_EQ(grid.getCurrentMap().data, map_data);
    EXPECT_EQ(grid.getRawMap().data, map_data);
}

TEST(Grid2DTest, probability){
    auto grid = createGrid();
    EXPECT_NEAR(grid.getProbability({0,0}), 0.9, 0.1);
    EXPECT_NEAR(grid.getProbability({0,1}), 0.9, 0.1);
    EXPECT_NEAR(grid.getProbability({0,2}), 0.1, 0.1);
    EXPECT_NEAR(grid.getProbability({0,3}), 0.1, 0.1);

    EXPECT_NEAR(grid.getProbability({1,0}), 0.9, 0.1);
    EXPECT_NEAR(grid.getProbability({1,1}), 0.1, 0.1);
    EXPECT_NEAR(grid.getProbability({1,2}), 0.9, 0.1);
    EXPECT_NEAR(grid.getProbability({1,3}), 0.1, 0.1);

    EXPECT_NEAR(grid.getProbability({2,0}), 0.1, 0.1);
    EXPECT_NEAR(grid.getProbability({2,1}), 0.9, 0.1);
    EXPECT_NEAR(grid.getProbability({2,2}), 0.1, 0.1);
    EXPECT_NEAR(grid.getProbability({2,3}), 0.1, 0.1);

    EXPECT_NEAR(grid.getProbability({3,0}), 0.9, 0.1);
    EXPECT_NEAR(grid.getProbability({3,1}), 0.1, 0.1);
    EXPECT_NEAR(grid.getProbability({3,2}), 0.9, 0.1);
    EXPECT_NEAR(grid.getProbability({3,3}), 0.9, 0.1);
}

TEST(Grid2DTest, set_value){
    auto grid = createGrid();
    grid.setProbability({2,0}, 0.9);
    EXPECT_NEAR(grid.getProbability({2,0}), 0.9, 0.1);
    grid.setProbability({3,0}, 0.1);
    EXPECT_NEAR(grid.getProbability({3,0}), 0.1, 0.1);
    grid.setProbability({1,0}, 1.0);
    EXPECT_NEAR(grid.getProbability({1,0}), 0.9, 0.1);
    grid.setProbability({3,1}, 0.);
    EXPECT_NEAR(grid.getProbability({3,1}), 0.1, 0.1);
    grid.setProbability({3,2}, 0.7);
    EXPECT_NEAR(grid.getProbability({3,2}), 0.7, 0.1);
}