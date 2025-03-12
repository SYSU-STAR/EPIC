/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-03-01 15:11:38
 * @LastEditTime: 2024-03-05 17:26:18
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#pragma once
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <pcl/common/distances.h>
#include <queue>
#include <ros/console.h>
#include <ros/ros.h>
#include <shared_mutex>
#include <string>
#include <thread>
#include <lidar_map/lidar_map.h>
#include <unordered_map>
#include <unordered_set>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <frontier_manager/frontier_manager.h>

using namespace fast_planner;

struct v3i_hash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

class ParallelBubbleAstar {
public:
  enum { REACH_END = 1, NO_PATH = 2, START_FAIL = 3, END_FAIL = 4, TIME_OUT = 5 };

  struct Node {
    typedef ::shared_ptr<Node> Ptr;
    Ptr parent;
    Eigen::Vector3f position;
    double g_score;
    double f_score;
  };

  struct NodeCompre {
    bool operator()(Node::Ptr &node1, Node::Ptr &node2) { return node1->f_score > node2->f_score; }
  };

  typedef std::shared_ptr<ParallelBubbleAstar> Ptr;

  ros::Publisher open_set_pub_;
  // FrontierManager::Ptr frontier_manager_;
  double resolution_, inv_resolution_, lambda_heu_, safe_distance_, tie_breaker_;
  int allocate_num_;
  bool debug_;
  double max_vel_, max_acc_;
  LIOInterface::Ptr lidar_map_interface_;
  Eigen::Vector3f origin_;

  unordered_set<Eigen::Vector3i, v3i_hash> safe_node;
  unordered_set<Eigen::Vector3i, v3i_hash> dangerous_node;
  std::shared_timed_mutex safe_node_mtx;
  std::shared_timed_mutex dangerous_node_mtx;

  void posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx);
  void IndexToPos(Eigen::Vector3f &pt, Eigen::Vector3i &idx);
  bool isNodeSafe(Node::Ptr node, const Eigen::Vector3f &bbox_min, const Eigen::Vector3f &bbox_max,
                  unordered_set<Eigen::Vector3i, v3i_hash> &safe_set, unordered_set<Eigen::Vector3i, v3i_hash> &danger_set);

  void init(ros::NodeHandle &nh, const LIOInterface::Ptr &lidar_map);
  void reset();
  bool collisionCheck_shortenPath(vector<Eigen::Vector3f> &path);
  // best_result = true: 启发式函数 = 1.0 * 欧氏距离
  int search(const Eigen::Vector3f &start, const Eigen::Vector3f &goal, vector<Eigen::Vector3f> &path, double timeout, bool best_result = false,
             bool only_raycast = false,
             const Eigen::Vector3f &bbox_min = Eigen::Vector3f(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                                                               std::numeric_limits<double>::lowest()),
             const Eigen::Vector3f &bbox_max = Eigen::Vector3f(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                                                               std::numeric_limits<double>::max()));
  void calculatePathCost(const vector<Eigen::Vector3f> &path, double &cost);
};