cmake_minimum_required(VERSION 2.8.3)
project(epic_planner)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

# find_package(CGAL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  pointcloud_topo
  path_searching
  cv_bridge

  # active_perception
  frontier_manager

  # bspline
  # bspline_opt
  traj_utils
  lkh_tsp_solver
  plan_manage
  cv_bridge
  message_filters

)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES epic_planner
  CATKIN_DEPENDS pointcloud_topo active_perception path_searching traj_utils lkh_tsp_solver plan_manage frontier_manager

  # DEPENDS system_lib
)

# include_directories(/home/lin/workspace/CGAL-5.6.1/include)
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)



add_executable(exploration_node
  src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
  src/fsm_utils.cpp
)

target_link_libraries(exploration_node
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  # remesh_isotropic_planar
  epic_planner
)

add_library(epic_planner
  src/fast_exploration_manager.cpp
)
target_link_libraries(epic_planner
  ${catkin_LIBRARIES}
)
