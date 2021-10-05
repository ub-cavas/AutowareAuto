cmake_minimum_required(VERSION 3.5)
project(astar_search)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(ASTAR_SEARCH_LIB astar_search)
ament_auto_add_library(${ASTAR_SEARCH_LIB} SHARED
        src/astar_search.cpp)
autoware_set_compile_options(${ASTAR_SEARCH_LIB})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
