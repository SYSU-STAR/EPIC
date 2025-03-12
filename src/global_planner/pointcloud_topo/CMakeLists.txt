cmake_minimum_required(VERSION 3.16)
project(pointcloud_topo)
find_package(OpenMP REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# find_package(CGAL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    cv_bridge
    message_filters
    lkh_tsp_solver
    lidar_map

    # mesh_booleans
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS )

# include_directories(/home/lin/workspace/CGAL-5.6.1/include)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pointcloud_topo parallel_bubble_astar
    CATKIN_DEPENDS roscpp std_msgs lkh_tsp_solver lidar_map
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(parallel_bubble_astar
    src/parallel_bubble_astar.cpp
)

target_link_libraries(parallel_bubble_astar
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_library(pointcloud_topo
    src/topo_skeleton_graph.cpp
    src/historical_odom_graph.cpp
    src/bubble_split.cpp
)

target_link_libraries(pointcloud_topo
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    parallel_bubble_astar
)

