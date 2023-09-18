set(TEST_NAMES
    point_cloud
    probability_grid
    voxel_filter
    ceres_scan_matcher
    fast_scan_matcher
    realtime_scan_matcher
    point_cloud_insertor
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