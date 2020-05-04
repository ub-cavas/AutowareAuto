# Copyright 2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

project(cepton_node)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT APPLE)
  # includes
  set(CEPTON_HEADERS lib/cepton_sdk_redist/include)
  include_directories(include ${CEPTON_HEADERS})

  # Include external library
  if(WIN32)
    set(CEPTON_LIB_DIR lib/cepton_sdk_redist/bin/win64/)
    set(CEPTON_LIB cepton_sdk.imp.lib)
  else()
    # TODO more stuff: os x etc.
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
      set(CEPTON_LIB_DIR lib/cepton_sdk_redist/lib/linux-aarch64/)
    else()
      set(CEPTON_LIB_DIR lib/cepton_sdk_redist/lib/linux-x86_64/)
    endif()
    set(CEPTON_LIB libcepton_sdk.so)
  endif()
  link_directories(${CEPTON_LIB_DIR})

  if(NOT WIN32)
    set(DISABLED_WARNINGS
      -Wno-pedantic
      -Wno-old-style-cast
      -Wno-sign-conversion
      -Wno-conversion
      -Wno-unused-parameter
      -Wno-unused-variable
      -Wno-sign-compare)
  endif()

  # generate library
  # These are kept separate because PCLint just picks up junk from standard headers otherwise
  set(CEPTON_NODE_HEADERS
    include/cepton_node/cepton_block_node.hpp
    include/cepton_node/cepton_cloud_node.hpp
    include/cepton_node/cepton_common.hpp
    include/cepton_node/visibility_control.hpp)
  set(CEPTON_NODE_SOURCES
    src/cepton_block_node.cpp
    src/cepton_cloud_node.cpp
    src/cepton_common.cpp)
  set(LIB_FILES ${CEPTON_NODE_HEADERS} ${CEPTON_NODE_SOURCES})

  add_library(${PROJECT_NAME} SHARED ${LIB_FILES})
  ament_target_dependencies(${PROJECT_NAME} "udp_driver" "apex_auto_msgs" "sensor_msgs"
      "lidar_utils" "logging")
  target_link_libraries(${PROJECT_NAME} ${CEPTON_LIB})
  autoware_set_compile_options(${PROJECT_NAME})
  # Disable warnings due to dirty cepton headers
  target_compile_options(${PROJECT_NAME} PRIVATE ${DISABLED_WARNINGS})

  # generate executable for ros1-style standalone nodes
  set(BLOCK_MAIN_FILES src/cepton_block_main.cpp)
  set(CEPTON_BLOCK_NODE_EXE "cepton_block_node_exe")
  add_executable(${CEPTON_BLOCK_NODE_EXE} ${BLOCK_MAIN_FILES})
  target_link_libraries(${CEPTON_BLOCK_NODE_EXE} ${PROJECT_NAME})
  ament_target_dependencies(${CEPTON_BLOCK_NODE_EXE} ${PROJECT_NAME} apex_init)
  autoware_set_compile_options(${CEPTON_BLOCK_NODE_EXE})
  target_compile_options(${CEPTON_BLOCK_NODE_EXE} PRIVATE ${DISABLED_WARNINGS})
  # generate executable for ros1-style standalone nodes
  set(CLOUD_MAIN_FILES src/cepton_cloud_main.cpp)
  set(CEPTON_CLOUD_NODE_EXE "cepton_cloud_node_exe")
  add_executable(${CEPTON_CLOUD_NODE_EXE} ${CLOUD_MAIN_FILES})
  target_link_libraries(${CEPTON_CLOUD_NODE_EXE} ${PROJECT_NAME})
  ament_target_dependencies(${CEPTON_CLOUD_NODE_EXE} ${PROJECT_NAME} apex_init)
  autoware_set_compile_options(${CEPTON_CLOUD_NODE_EXE})
  target_compile_options(${CEPTON_CLOUD_NODE_EXE} PRIVATE ${DISABLED_WARNINGS})

  # Testing
  if(BUILD_TESTING)
    # run common linters
    find_package(ament_lint_auto)
    ament_lint_auto_find_test_dependencies()

    # Static code analyzers
    find_package(ament_cmake_pclint)
    ament_pclint(${CEPTON_NODE_SOURCES} ${BLOCK_MAIN_FILES} ${CLOUD_MAIN_FILES})
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_pclint)

    # only run uncrustify in Apex code
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    ament_uncrustify(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

    # only run cpplint in Apex code
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
    ament_cpplint(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

    # only run flake8 in Apex code
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)
    ament_flake8(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

    # only run pep257 in Apex code
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_pep257)
    ament_pep257(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

    # only run copyright in Apex code
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_copyright(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

    # only run lint_cmake in Apex code
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_lint_cmake)
    ament_lint_cmake(${CMAKE_SOURCE_DIR}/CMakeLists.txt)

    # Check for file existence: if not submodule probably hasn't been init'd
    if(NOT (EXISTS "${PROJECT_SOURCE_DIR}/lib/cepton_sdk_redist/source/capture.hpp"))
      message(FATAL_ERROR "SDK not found, run git submodule update --init")
    endif()

    set(REPLAYER cepton_replay_exe)
    add_executable(${REPLAYER}
      lib/cepton_sdk_redist/source/capture.cpp
      test/replayer.cpp
    )
    ament_target_dependencies(${REPLAYER} "apex_init" "apexcpp" "argparse" "logging")
    target_link_libraries(${REPLAYER} ${CEPTON_LIB})
    target_include_directories(${REPLAYER} PRIVATE lib/cepton_sdk_redist/source test/)
    autoware_set_compile_options(${REPLAYER})
    target_compile_options(${REPLAYER} PRIVATE ${DISABLED_WARNINGS})
    list(APPEND ${PROJECT_NAME}_EXECUTABLES "${REPLAYER}")

    # Move test pcap file into build folder
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test/lidar.pcap
      DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cepton_node/")

    # Integration test
    find_package(ros_testing REQUIRED)
    add_ros_test(test/cepton_node.test.py)
  endif()

  # Install stuff
  # Special installation for external cepton headers
  install(
    DIRECTORY ${CEPTON_HEADERS}
    DESTINATION include
  )
  ament_export_include_directories(${CEPTON_HEADERS})
  # Special install for external cepton libraries
  install(
    DIRECTORY ${CEPTON_LIB_DIR}
    DESTINATION lib
    USE_SOURCE_PERMISSIONS FILES_MATCHING PATTERN "*.so*")
  ament_export_libraries(${CEPTON_LIB_DIR}/libcepton.so)

  # Ament Exporting
  list(APPEND ${PROJECT_NAME}_LIBRARIES "${PROJECT_NAME}")
  list(APPEND ${PROJECT_NAME}_EXECUTABLES
    "${CEPTON_CLOUD_NODE_EXE}"
    "${CEPTON_BLOCK_NODE_EXE}"
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE param/
)
