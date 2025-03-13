/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-02 16:33:15
 * @LastEditTime: 2024-03-14 11:41:02
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#pragma once
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <omp.h>
#include <pcl/common/distances.h>

#include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_topo/parallel_bubble_astar.h>
#include <random>
#include <ros/ros.h>
#include <thread>
#include <lidar_map/lidar_map.h>
#include <unordered_map>
#include <unordered_set>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std;

struct Vector3iHash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::size_t seed = 0;
    for (int i = 0; i < 3; ++i) {
      seed ^= std::hash<int>{}(v[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct Vector2iHash {
  std::size_t operator()(const Eigen::Vector2i &v) const {
    std::size_t seed = 0;
    for (int i = 0; i < 2; ++i) {
      seed ^= std::hash<int>{}(v[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct PairHash {
  std::size_t operator()(const std::pair<Eigen::Vector3i, Eigen::Vector3i> &p) const {
    std::size_t seed = 0;
    seed ^= Vector3iHash{}(p.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= Vector3iHash{}(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct PairHashSet {
  std::shared_timed_mutex hs_mtx;

  void insert(const Eigen::Vector3i &v1, const Eigen::Vector3i &v2) {
    std::unique_lock<std::shared_timed_mutex> lk(hs_mtx);
    mySet.insert(std::make_pair(v1, v2));
    mySet.insert(std::make_pair(v2, v1));
    lk.unlock();
  }

  void remove(const Eigen::Vector3i &v1, const Eigen::Vector3i &v2) {
    std::unique_lock<std::shared_timed_mutex> lk(hs_mtx);
    mySet.erase(std::make_pair(v1, v2));
    mySet.erase(std::make_pair(v2, v1));
    lk.unlock();
  }

  bool check(const Eigen::Vector3i &v1, const Eigen::Vector3i &v2) {
    std::shared_lock<std::shared_timed_mutex> lk(hs_mtx);
    bool fail = mySet.find(std::make_pair(v1, v2)) == mySet.end();
    lk.unlock();
    if (fail) {
      return false;
    } else {
      return true;
    }
  }

  void reset() { mySet.clear(); }

  // private:
  std::unordered_set<std::pair<Eigen::Vector3i, Eigen::Vector3i>, PairHash> mySet;
};

class BubbleNode {
public:
  typedef std::shared_ptr<BubbleNode> Ptr;
  BubbleNode(double radius, Eigen::Vector3f center);
  double radius_;
  int idx_;
  Eigen::Vector3f center_;
};

class TopoNode {
public:
  typedef std::shared_ptr<TopoNode> Ptr;
  bool is_viewpoint_ = false;
  bool is_history_odom_node_ = false;
  float yaw_;
  Eigen::Vector3f center_;
  vector<BubbleNode::Ptr> bubbles_; // 过程量，计算出topoNode后会清空
  unordered_set<TopoNode::Ptr> neighbors_;
  unordered_map<TopoNode::Ptr, uint8_t> unreachable_nbrs_;
  unordered_map<TopoNode::Ptr, vector<Eigen::Vector3f>> paths_;
  unordered_map<TopoNode::Ptr, float> weight_;
};

struct PairPtrHash {
  std::size_t operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
    return std::hash<TopoNode::Ptr>()(p.first) ^ std::hash<TopoNode::Ptr>()(p.second);
  }
};

struct PtrPair {
  PtrPair() { flatten_data.reserve(2000); };

  void insert(TopoNode::Ptr a, TopoNode::Ptr b) {
    if (map.count(a) && map[a].count(b))
      return;
    if (map.count(b) && map[b].count(a))
      return;
    map[a].insert(b);
  }

  unordered_map<TopoNode::Ptr, unordered_set<TopoNode::Ptr>> map;

  struct iter_elem {
    TopoNode::Ptr p1;
    TopoNode::Ptr p2;
    bool insert;
    vector<Eigen::Vector3f> path;
  };

  void flatten() {
    flatten_data.clear();
    for (auto it = map.begin(); it != map.end(); it++) {
      for (auto it2 = it->second.begin(); it2 != it->second.end(); it2++) {
        flatten_data.push_back(iter_elem{it->first, *it2, true, {}});
      }
    }
  }

  vector<iter_elem> flatten_data;
};

class RegionNode {
public:
  typedef std::shared_ptr<RegionNode> Ptr;
  RegionNode(Eigen::Vector3i region_idx);
  Eigen::Vector3i region_idx_;
  int his_odom_id_;
  unordered_set<TopoNode::Ptr> topo_nodes_;
};

class BubbleUnionSet {
public:
  BubbleUnionSet(double min_topobubble_radius) : min_topobubble_radius_(min_topobubble_radius) {};
  typedef std::shared_ptr<BubbleUnionSet> Ptr;
  void updateRegionNode(RegionNode::Ptr region_ptr, const Eigen::Vector3f &region_center_);
  void unionSetCluster(const vector<BubbleNode::Ptr> &bubbles, vector<TopoNode::Ptr> &topos, Eigen::Vector3f &center);

private:
  std::unordered_map<BubbleNode::Ptr, BubbleNode::Ptr> parent;
  std::unordered_map<BubbleNode::Ptr, int> rank;
  std::vector<BubbleNode::Ptr> clusters;
  std::vector<BubbleNode::Ptr> bubbles;
  double min_topobubble_radius_;
  std::unordered_map<BubbleNode::Ptr, TopoNode::Ptr> topo_map;
  void init(const std::vector<BubbleNode::Ptr> &bubbles_);
  BubbleNode::Ptr find(BubbleNode::Ptr b);
  void merge(BubbleNode::Ptr b1, BubbleNode::Ptr b2);
  void getClusters();
  void getTopoNodes(unordered_set<TopoNode::Ptr> &topo_nodes_, const Eigen::Vector3f &center);
};

class TopoGraph {
public:
  TopoGraph(float res = 0.1) : check_pts_octree_(res) {}

  ros::NodeHandle nh;

  typedef std::unordered_map<Eigen::Vector3i, TopoNode::Ptr, Vector3iHash> HashMap;
  TopoNode::Ptr odom_node_;
  ParallelBubbleAstar::Ptr parallel_bubble_astar_;
  std::ofstream log;
  void removeNodes(vector<TopoNode::Ptr> &nodes);
  void updateRemainedConnections(vector<TopoNode::Ptr> &nodes);
  void insertNodes(vector<TopoNode::Ptr> &nodes, bool only_raycast = false);
  void insertNode(TopoNode::Ptr &new_node, vector<TopoNode::Ptr> &nbr_nodes, vector<vector<Eigen::Vector3f>> &paths);
  // void getUnreachableLocalNodes(vector<TopoNode::Ptr> &nodes_unreachable);
  void updateSkeleton();
  void updateHistoricalOdoms();
  void updateOdomNode(Eigen::Vector3f &odom_pos, float &yaw);
  Eigen::Vector3f min_bd, max_bd, map_bd_min, map_bd_max;
  double min_x_, min_y_, min_z_; // 最小格子尺寸
  double bubble_min_radius_, frt_bubble_radius_;
  double init_region_size_x_, init_region_size_y_, init_region_size_z_; // 初始分区大小
  int x_len, y_len, z_len;                                              // 分区数量
  double max_radius, cube_discrete_size;
  bool view_graph_;
  int getBoxId(const Eigen::Vector3f &pt);
  vector<Eigen::Vector3i> update_idx_vec_; //

  vector<RegionNode::Ptr> toponodes_update_region_arr_;
  vector<RegionNode::Ptr> viewpoints_update_region_arr_;

  vector<RegionNode::Ptr> regions_arr_;
  unordered_map<Eigen::Vector3i, RegionNode::Ptr, Vector3iHash> reg_map_idx2ptr_;
  vector<Eigen::Vector3f> global_path_;
  vector<Eigen::Vector3f> global_view_points_;
  LIOInterface::Ptr lidar_map_interface_;
  typedef std::shared_ptr<TopoGraph> Ptr;
  void getIndex(const Eigen::Vector3f &point, Eigen::Vector3i &region_idx_);
  bool index2boundary(const Eigen::Vector3i &region_idx_, Eigen::Vector3f &low_bd, Eigen::Vector3f &high_bd);
  RegionNode::Ptr getRegionNode(const Eigen::Vector3i &region_idx_);
  bool graphSearch(const TopoNode::Ptr &start_node, const TopoNode::Ptr &end_node, std::vector<TopoNode::Ptr> &path, double time_out,
                   bool kino = false, std::unordered_set<pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> last_path = {});
  void init(ros::NodeHandle &nh, LIOInterface::Ptr &lidar_map, ParallelBubbleAstar::Ptr &parallel_bubble_astar);
  void cauculateMemoryConsumption();
  double getPathLength(const vector<TopoNode::Ptr> &topo_path);

  void inline posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
    idx = ((pt - lidar_map_interface_->lp_->global_box_min_boundary_) * 1000).array().floor().cast<int>();
  }

  void inline indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3f &pt) {
    pt = (idx.cast<float>() + Eigen::Vector3f(0.5, 0.5, 0.5)) / 1000.0f + lidar_map_interface_->lp_->global_box_min_boundary_;
  }

  void overlap(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &overlap);
  void setdiff(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &set_1diff2);
  void getPreNbrs(TopoNode::Ptr &node, vector<TopoNode::Ptr> &nbrs);
  void getRegionsToUpdate();
  void removeNode(TopoNode::Ptr &node);
  float estimateRoughDistance(const Eigen::Vector3f &goal, const int his_idx);
  vector<TopoNode::Ptr> history_odom_nodes_;
  vector<float> his_odom_dis_vec_;

private:
  PointVector check_pts_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> check_pts_octree_;
  int max_update_region_num_;
  bool use_prior_map_;
  double update_connection_timeout, insert_node_timeout;
  bool hasOverlapWithBox(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd);

  void generateBubble(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd, vector<BubbleNode::Ptr> &bubble_node_vec,
                      vector<bool> &check_flags);
  void splitCubeBubbleGeneration(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd, vector<BubbleNode::Ptr> &bubble_node_vec,
                                 vector<bool> &check_flags);
  void supplementCubeBubbleGeneration(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd, vector<BubbleNode::Ptr> &bubble_node_vec,
                                      vector<bool> &check_flags, const BubbleNode::Ptr &bubble_node);
  bool isCubeCoveredByBubble(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd, const vector<BubbleNode::Ptr> &bubble_node_vec);

  int searchPathWithBoundary(const Eigen::Vector3f &start, const Eigen::Vector3f &end, double &time_out, vector<Eigen::Vector3f> &path);
};