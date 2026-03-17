cmake_minimum_required(VERSION 3.15)
project(slam_core VERSION 1.0.0 LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Find ZeroMQ (cppzmq header-only wrapper + zmq library)
find_package(cppzmq REQUIRED)

# Prefer local user install path for GTSAM (e.g. ~/.local/gtsam)
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local/gtsam")

# Find GTSAM (graph optimization)
find_package(GTSAM REQUIRED)

# Use the Eigen version that GTSAM was built against.
# GTSAM installs Eigen under: <prefix>/include/gtsam/3rdparty/Eigen/Eigen/...
set(EIGEN_FROM_GTSAM_INCLUDE_DIR "${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen/Eigen")

set(GTSAM_LINK_TARGET "")
if(TARGET GTSAM::gtsam)
    set(GTSAM_LINK_TARGET GTSAM::gtsam)
elseif(TARGET gtsam)
    set(GTSAM_LINK_TARGET gtsam)
else()
    message(FATAL_ERROR "GTSAM was found, but no expected link target exists (GTSAM::gtsam or gtsam).")
endif()


set(SOURCES
    src/main.cpp
    src/zmq_bridge.cpp
    src/odometry.cpp
    src/lidar_processor.cpp
    src/occupancy_grid.cpp
    src/icp_matcher.cpp
    src/feature_extractor.cpp
    src/PoseGraph.cpp
    src/pose_graph_optimizer.cpp
)


set(HEADERS
    src/zmq_bridge.h
    src/odometry.h
    src/lidar_processor.h
    src/icp_matcher.h
    src/feature_extractor.h
    src/types.h
    src/PoseGraph.h
    src/pose_graph_optimizer.h
)


add_executable(slam_subscriber ${SOURCES} ${HEADERS})


target_link_libraries(slam_subscriber 
    PRIVATE 
    cppzmq
    ${GTSAM_LINK_TARGET}
)


target_include_directories(slam_subscriber
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN_FROM_GTSAM_INCLUDE_DIR}
)


install(TARGETS slam_subscriber
    RUNTIME DESTINATION bin
)

# Test executable for ICP
add_executable(test_icp
    tests/test_icp_transform.cpp
    src/icp_matcher.cpp
)

target_link_libraries(test_icp
    PRIVATE
)

target_include_directories(test_icp
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN_FROM_GTSAM_INCLUDE_DIR}
)

add_executable(test_icp_full
    tests/test_icp_full.cpp
    src/icp_matcher.cpp
)

target_link_libraries(test_icp_full
    PRIVATE
)

target_include_directories(test_icp_full
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN_FROM_GTSAM_INCLUDE_DIR}
)

add_executable(test_scan_conversion
    tests/test_scan_conversion.cpp
    src/lidar_processor.cpp
)

target_link_libraries(test_scan_conversion
    PRIVATE
)

target_include_directories(test_scan_conversion
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN_FROM_GTSAM_INCLUDE_DIR}
)

# Print configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
