cmake_minimum_required(VERSION 3.0.2)
project(lidar_map)
find_package(OpenMP REQUIRED)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_filters
  visualization_msgs

)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS segmentation)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES  unit_shpere  ikd_tree 
    CATKIN_DEPENDS roscpp std_msgs 
    # DEPENDS system_lib
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
add_library(ikd_tree
  src/ikd_Tree.cpp

)
add_library(unit_shpere
    src/sim_lio.cpp
    src/lidar_map_interface.cpp
)

target_link_libraries(unit_shpere
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    ikd_tree
)