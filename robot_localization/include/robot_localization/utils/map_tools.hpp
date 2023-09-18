#ifndef MAP_TOOLS__HPP__
#define MAP_TOOLS__HPP__
#include "nav_msgs/msg/map_meta_data.hpp"

#define MAP_WXGX(map_infor, i) (map_infor->origin.position.x + ((i) - map_infor->width / 2) * map_infor->info.resolution)
#define MAP_WYGY(map_infor, j) (map_infor->origin.position.y + ((j) - map_infor->height / 2) * map_infor->resolution)

// Convert from world coords to map coords
#define MAP_GXWX(map_infor, x) (floor((x - map_infor->origin.position.x) / map_infor->resolution + 0.5) + map_infor->width / 2)
#define MAP_GYWY(map_infor, y) (floor((y - map_infor->origin.position.y) / map_infor->resolution + 0.5) + map_infor->height / 2)

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map_infor, i, j) ((i >= 0) && (i < (int)map_infor->width) && (j >= 0) && (j < (int)map_infor->height))


// Compute the cell index for the given map coords.
#define MAP_INDEX(map_infor, i, j) ((i) + (j) * map_infor->width)

#endif