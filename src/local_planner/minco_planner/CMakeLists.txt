cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++14")

# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS segmentation)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  quadrotor_msgs
  poly_traj
  pointcloud_topo
  path_searching

  traj_utils
  cv_bridge
  pcl_ros
  pcl_conversions
  pcl_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plan_manage
  CATKIN_DEPENDS pointcloud_topo path_searching traj_utils)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(fast_planner_node
  src/fast_planner_node.cpp

  # src/kino_replan_fsm.cpp
  # test/local_explore_fsm.cpp
  src/planner_manager.cpp

  # src/planner_manager_dev.cpp
)
target_link_libraries(fast_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})

# add_executable(traj_server2 src/traj_server_2.cpp)
# target_link_libraries(traj_server2 ${catkin_LIBRARIES})
# set(CMAKE_BUILD_TYPE "Debug")

add_library(plan_manage
  src/planner_manager.cpp
)
target_link_libraries(plan_manage ${catkin_LIBRARIES} )



