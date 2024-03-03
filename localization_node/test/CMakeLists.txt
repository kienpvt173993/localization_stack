set(TEST_NAMES
    voxel_filter
    point_cloud_2d
    grid_2d
    transform
)

foreach(name IN LISTS TEST_NAMES)
  ament_add_gtest(${name}
    ${name}_test.cpp
  )

  ament_target_dependencies(${name}
    ${dependencies}
  )

  target_link_libraries(${name}
    ${library_name}
  )

  if(${TEST_DEBUG_INFO})
    target_compile_definitions(${name} PUBLIC -DTEST_DEBUG_INFO)
  endif()

endforeach()