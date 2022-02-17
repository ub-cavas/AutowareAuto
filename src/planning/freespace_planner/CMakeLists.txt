cmake_minimum_required(VERSION 3.5)
project(freespace_planner)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(FREESPACE_PLANNER_LIB freespace_planner)
ament_auto_add_library(${FREESPACE_PLANNER_LIB} SHARED
  src/freespace_planner/base_planning_algorithm.cpp
  src/freespace_planner/astar_search.cpp
  src/freespace_planner/reeds_shepp.cpp
  src/freespace_planner/reeds_shepp_impl.cpp)

autoware_set_compile_options(${FREESPACE_PLANNER_LIB})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Unit tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${FREESPACE_PLANNER_LIB}_test test/test_freespace_planner.cpp TIMEOUT 600)
  autoware_set_compile_options(${FREESPACE_PLANNER_LIB}_test)
  target_link_libraries(${FREESPACE_PLANNER_LIB}_test ${FREESPACE_PLANNER_LIB})
endif()

ament_auto_package(INSTALL_TO_SHARE)
