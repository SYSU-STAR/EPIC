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

namespace fast_planner {
struct matrix_hash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    // Combine the hash values of the individual components
    std::hash<int> hasher;
    size_t hash = 0;
    for (int i = 0; i < 3; ++i) {
      hash ^= hasher(v[i]) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

class Bubble {
public:
  // Bubble();
  void init(double radius, PointType position);
  double radius2;
  PointType position;
};

class GridNode {
public:
  Eigen::Vector3i index; // 假想的栅格
  PointType position;
  double g_score, f_score;
  Bubble *safe_bubble;
  GridNode *parent;
  std::vector<GridNode *> sons;
  bool is_center;

  /* -------------------- */
  GridNode() { parent = NULL, is_center = false; }

  // ~GridNode();
};

typedef GridNode *GridNodePtr;
typedef Bubble *BubblePtr;

class BubbleNodeComparator0 {
public:
  bool operator()(GridNodePtr node1, GridNodePtr node2) { return node1->f_score > node2->f_score; }
};

// class BubbleAreaComparator {
//    public:
//     bool operator()(BubblePtr bubble1, BubblePtr bubble2) {
//         return bubble1->radius > bubble2->radius;
//     }
// };
class BubbleVisualizer {
public:
  ros::NodeHandle nh_;
  ros::Publisher spherePub, wayPointsPub, spherePub_debug_;

  BubbleVisualizer() {}

  ~BubbleVisualizer() {}

  void init(ros::NodeHandle &nh);
  void visualize(const std::vector<GridNodePtr> &bubble_nodes_);
  void visualizeSingleBubble(BubblePtr &bubble, Vector3f color, bool clear_all);
};



class BubbleAstar {
public:
  typedef std::shared_ptr<BubbleAstar> Ptr;
  BubbleAstar();
  ~BubbleAstar();
  BubbleVisualizer vizer;

  enum { REACH_END = 1, NO_PATH = 2, START_FAIL = 3, END_FAIL = 4, TIME_OUT = 5 };

  void init(ros::NodeHandle &nh, const LIOInterface::Ptr &lidar_map);

  void reset(double resolution);
  std::vector<Eigen::Vector3f> getPath();

  int search(const Vector3f &start_pt, const Vector3f &end_pt, const double &max_time, bool refine_goal = true, bool topo_insert = false);

  static double pathLength(const vector<Eigen::Vector3f> &path);

  std::vector<Eigen::Vector3f> getVisited();
  double getEarlyTerminateCost();

  double lambda_heu_;
  double max_search_time_;
  bool safeCheck(GridNodePtr node);
  bool safeCheck(GridNodePtr node, const PointVector &points);
  double resolution_, inv_resolution_;
  void goal_refine(Eigen::Vector3f &goal, bool use_map_bd = false); // 调整目标位置，使之满足膨胀半径约束

private:
  void backtrack(const GridNodePtr &end_node, const Eigen::Vector3f &end, bool viz = false);
  void posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx);
  void posToIndex(const PointType &pt, Eigen::Vector3i &idx);
  // void IndexToPos(Eigen::Vector3f& pt, Eigen::Vector3i& idx);
  void IndexToPos(PointType &pt, Eigen::Vector3i &idx);
  double getDiagHeu(const Eigen::Vector3f &x1, const Eigen::Vector3f &x2);
  double getDiagHeu(const PointType &x1, const PointType &x2);
  double getDiagHeu(const Eigen::Vector3f &x1, const PointType &x2);
  bool generateBubble(GridNodePtr &node, bool is_start = false);
  bool generateBubble(GridNodePtr &node, const PointVector &points);

  // main data structure
  double safe_distance_, bubble_avg_cost_, bubble_avg_cost_2;
  vector<GridNodePtr> path_node_pool_;
  vector<BubblePtr> bubble_node_pool_;
  vector<BubblePtr> safeArea_, deadArea_;
  int use_node_num_, iter_num_, bubble_used_, bubble_cnt_, bubble_cnt_2;
  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, BubbleNodeComparator0> open_set_;
  std::unordered_map<Eigen::Vector3i, GridNodePtr, matrix_hash> open_set_map_;
  std::unordered_map<Eigen::Vector3i, BubblePtr, matrix_hash> bubble_map_;
  std::unordered_map<Eigen::Vector3i, int, matrix_hash> close_set_map_;
  std::unordered_map<Eigen::Vector3i, int, matrix_hash> dead_set_map_;
  std::vector<Eigen::Vector3f> path_nodes_;
  std::vector<Eigen::Vector4d> path_nodes_4d_;
  std::vector<GridNodePtr> bubble_nodes_;
  bool debug_;
  double early_terminate_cost_;
  double safe_check_cost_;
  double loop_cost_;
  int close_set_visit_cnt_;
  int close_set_work_cnt_;

  LIOInterface::Ptr lidar_map_interface_;

  // parameter
  int allocate_num_;
  double tie_breaker_;
  Eigen::Vector3f origin_;
};

class FastSearcher {
private:
  TopoGraph::Ptr topo_graph_;
  BubbleAstar::Ptr bubble_astar_searcher_;

public:
  typedef std::shared_ptr<FastSearcher> Ptr;
  void init(TopoGraph::Ptr topo_graph, BubbleAstar::Ptr bubble_astar_searcher);
  TopoNode::Ptr createTopoNode(const Eigen::Vector3f &pose, const Eigen::Vector3f &curr_vel, bool connect_nei_region = true, bool nei_must_known = false,
                               bool is_odom_node = false);
  void removeTopoNode(TopoNode::Ptr node);
  //   void init(ros::NodeHandle& nh, const LIOInterface::Ptr& lidar_map);
  int search(const TopoNode::Ptr &start_node, const Vector3f &curr_vel, const TopoNode::Ptr &end_node, double max_time, std::vector<Eigen::Vector3f> &path);
  int topoSearch(const TopoNode::Ptr &start_node, const TopoNode::Ptr &end_node, double max_time, std::vector<Eigen::Vector3f> &path);
};
} // namespace fast_planner

#endif