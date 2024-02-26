#include "gtest/gtest.h"
#include "localization_node/grid/grid_2d.hpp"

namespace localization_node{
namespace grid{

nav_msgs::msg::OccupancyGrid createMap(){
    nav_msgs::msg::OccupancyGrid map;
    map.data = {    0,      0,      -1,     100,    0,
                    -1,     100,    0,      0,      100,
                    -1,     100,    100,    100,    -1,
                    -1,     -1,     -1,     0,      100};
    map.info.height = 4;
    map.info.width = 5;
    map.info.resolution = 0.05;
    map.info.origin.position.x = -0.2;
    map.info.origin.position.y = -0.15;
    map.info.origin.orientation.w = 1;
    map.info.origin.orientation.z = 0;
    return map;
}

Grid2D createGrid2D(){
    auto map = createMap();
    return Grid2D(map);
}

TEST(Grid2DTest, create_grid){
    auto grid = createGrid2D();
    auto map_ref = createMap();
    auto map = grid.getRosMap();
    for(size_t i = 0; i < map_ref.data.size(); i++){
        EXPECT_EQ(map_ref.data[i], map->data[i]);
    }
    EXPECT_EQ(map->info.width, map_ref.info.width);
    EXPECT_EQ(map->info.height, map_ref.info.height);

    EXPECT_NEAR(map->info.origin.position.x,
                map_ref.info.origin.position.x, 0.01);

    EXPECT_NEAR(map->info.origin.position.y,
                map_ref.info.origin.position.y, 0.01);

    EXPECT_NEAR(map->info.origin.position.z,
                map_ref.info.origin.position.z, 0.01);

    EXPECT_NEAR(map->info.origin.orientation.x,
                map_ref.info.origin.orientation.x, 0.01);

    EXPECT_NEAR(map->info.origin.orientation.y,
                map_ref.info.origin.orientation.y, 0.01);

    EXPECT_NEAR(map->info.origin.orientation.z,
                map_ref.info.origin.orientation.z, 0.01);

    EXPECT_NEAR(map->info.origin.orientation.w,
                map_ref.info.origin.orientation.w, 0.01);
}

TEST(Grid2DTest, init_probability){
    std::vector<float> ps = {   0.1f,   0.1f,   0.1f,   0.9f,   0.1f,
                                0.1f,   0.9f,   0.1f,   0.1f,   0.9f,
                                0.1f,   0.9f,   0.9f,   0.9f,   0.1f,
                                0.1f,   0.1f,   0.1f,   0.1f,   0.9f};
    auto grid = createGrid2D();
    for(size_t i = 0; i < ps.size(); i++){
        auto x = static_cast<int>(i%5);
        auto y = static_cast<int>(i/5);
        auto cell = grid.getProbability({x,y,0});
        EXPECT_NEAR(cell, ps[i], 0.01f);
    }
}

TEST(Grid2DTest, int_cell){
    auto grid = createGrid2D();
    auto cell_1 = grid.getCellIndex({0.04, 0.11, 0.0});
    EXPECT_EQ(cell_1.x(), 7);
    EXPECT_EQ(cell_1.y(), 8);

    auto cell_2 = grid.getCellIndex({-0.04, -0.11, 0.0});
    EXPECT_EQ(cell_2.x(), 5);
    EXPECT_EQ(cell_2.y(), 4);

    auto cell_3 = grid.getCellIndex({0.24, 0.5, 0.0});
    EXPECT_EQ(cell_3.x(), 11);
    EXPECT_EQ(cell_3.y(), 16);

    auto cell_4 = grid.getCellIndex({-0.24, -0.5, 0.0});
    EXPECT_EQ(cell_4.x(), 1);
    EXPECT_EQ(cell_4.y(), -4);
}

TEST(Grid2DTest, limit){
    auto grid = createGrid2D();
    auto cell_1 = grid.inLimit({1,2,0});
    EXPECT_TRUE(cell_1);
    auto cell_2 = grid.inLimit({11,5,0});
    EXPECT_FALSE(cell_2);
    auto cell_3 = grid.inLimit({1,-5,0});
    EXPECT_FALSE(cell_3);
}

TEST(Grid2DTest, probability_limit){
    auto grid = createGrid2D();
    grid.setProbability({-1,1,0}, 0.8f);
    auto value_1 = grid.getProbability({-1,1,0});
    EXPECT_NEAR(value_1, 0.1f, 0.01f);

    grid.setProbability({2,3,0}, 0.f);
    auto value_2 = grid.getProbability({2,3,0});
    EXPECT_NEAR(value_2, 0.1f, 0.01f);

    grid.setProbability({2,1,0}, 1.f);
    auto value_3 = grid.getProbability({2,1,0});
    EXPECT_NEAR(value_3, 0.9f, 0.01f);

    grid.setProbability({1,1,0}, 0.8f);
    auto value_4 = grid.getProbability({1,1,0});
    EXPECT_NEAR(value_4, 0.8f, 0.01f);
}

}
}