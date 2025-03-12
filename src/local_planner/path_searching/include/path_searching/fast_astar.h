/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-28 14:48:50
 * @LastEditTime: 2024-01-14 21:41:59
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */
#ifndef _BUBBLE_ASTAR_H
#define _BUBBLE_ASTAR_H

#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <pcl/common/distances.h>
#include <pointcloud_topo/graph.h>
#include <lidar_map/lidar_map.h>
#include <queue>
#include <ros/console.h>
#include <ros/ros.h>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace Eigen;

class FastSearcher {
private:
  TopoGraph::Ptr topo_graph_;

public:
  typedef std::shared_ptr<FastSearcher> Ptr;
  void init(TopoGraph::Ptr topo_graph);
  int search(const Vector3d &start_pt, const Vector3d &curr_vel, const Vector3d &end_pt, double max_time, std::vector<Eigen::Vector3d> &path);
};

#endif