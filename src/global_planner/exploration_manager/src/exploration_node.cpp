/*** 
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-10-12 11:33:14
 * @LastEditTime: 2025-03-12 17:02:18
 * @Description: 
 * @
 * @Copyright (c) 2025 by ning-zelin, All Rights Reserved. 
 */


#include <epic_planner/fast_exploration_fsm.h>
#include <epic_planner/fast_exploration_manager.h>
#include <frontier_manager/frontier_manager.h>
#include <plan_manage/backward.hpp>
#include <plan_manage/planner_manager.h>
#include <ros/ros.h>

namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");
  LIOInterface::Ptr lio_interface = std::make_shared<LIOInterface>();
  ParallelBubbleAstar::Ptr parallel_path_finder =
      std::make_shared<ParallelBubbleAstar>();
  FrontierManager::Ptr frontier_manager = std::make_shared<FrontierManager>();
  TopoGraph::Ptr graph = std::make_shared<TopoGraph>();
  FastPlannerManager::Ptr planner_manager =
      std::make_shared<FastPlannerManager>();
  FastExplorationManager::Ptr explore_manager =
      std::make_shared<FastExplorationManager>();
  FastExplorationFSM expl_fsm;

  lio_interface->init(nh);
  graph->init(nh, lio_interface, parallel_path_finder);
  parallel_path_finder->init(nh, lio_interface);
  planner_manager->initPlanModules(nh, parallel_path_finder, graph);
  frontier_manager->init(nh, lio_interface, graph);
  explore_manager->initialize(nh, frontier_manager, planner_manager);
  expl_fsm.init(nh, explore_manager);

  ros::Duration(1.0).sleep();
  ros::spin();
  ros::shutdown();
  return 0;
}
