cmake_minimum_required(VERSION 3.0.2)
project(frontier_manager)
find_package(OpenMP REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    message_filters
    visualization_msgs
    lidar_map
    pointcloud_topo
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS segmentation)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES frontier_manager
    CATKIN_DEPENDS roscpp std_msgs lidar_map pointcloud_topo

)
include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_library(frontier_manager
    src/frontier_manager.cpp
    src/global_planning
    src/raycast.cpp
    src/frontier_viszer.cpp
)
target_link_libraries(frontier_manager PUBLIC
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OpenMP_CXX_FLAGS}
)

