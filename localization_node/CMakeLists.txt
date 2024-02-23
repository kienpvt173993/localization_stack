cmake_minimum_required(VERSION 3.8)
project(localization_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)

set(execute_node localization_node)
set(library_name ${execute_node}_package)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(dependencies 
  geometry_msgs
  nav_msgs
  rclcpp
  absl
  Eigen3
  Ceres
)

foreach(name IN LISTS dependencies)
  find_package(${name} REQUIRED)
endforeach()

include_directories(include)
  add_library(${library_name} SHARED
  src/grid/grid_2d.cpp
  src/sensor/point_cloud_2d.cpp
)

target_include_directories(${library_name} PUBLIC ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(${library_name}
  ${dependencies}
  Eigen3
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY
#         map
#         rviz
#         launch
#         config
#         DESTINATION share/${PROJECT_NAME}/)

target_link_libraries(${library_name}
  ${CERES_LIBRARIES}
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()