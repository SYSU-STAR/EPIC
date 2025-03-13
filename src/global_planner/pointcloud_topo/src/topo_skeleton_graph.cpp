/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-02 21:33:08
 * @LastEditTime: 2024-03-05 12:12:19
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include "pointcloud_topo/graph.h"

void debug_exit(const std::string &location) {
  std::cout << "\033[1;31m Terminating process at location: " << location << "\033[0m" << std::endl;
  exit(0);
}

void TopoGraph::init(ros::NodeHandle &nh, LIOInterface::Ptr &lidar_map, ParallelBubbleAstar::Ptr &parallel_bubble_astar) {
  lidar_map_interface_ = lidar_map;

  min_bd = lidar_map_interface_->lp_->global_box_min_boundary_;
  max_bd = lidar_map_interface_->lp_->global_box_max_boundary_;

  parallel_bubble_astar_ = parallel_bubble_astar;
  odom_node_ = make_shared<TopoNode>();
  odom_node_->is_viewpoint_ = true;
  // 分区，初始化regions_arr_
  // 10m * 10m * 2m ==> 0.315 * 0.315 * 0.5
  nh = nh;
  nh.param("bubble_topo/min_x", min_x_, 0.0);
  nh.param("bubble_topo/min_y", min_y_, 0.0);
  nh.param("bubble_topo/min_z", min_z_, 0.0);
  nh.param("bubble_topo/init_region_size_x", init_region_size_x_, 0.0);
  nh.param("bubble_topo/init_region_size_y", init_region_size_y_, 0.0);
  nh.param("bubble_topo/init_region_size_z", init_region_size_z_, 0.0);
  nh.param("bubble_topo/bubble_min_radius", bubble_min_radius_, 0.5);
  nh.param("bubble_topo/frontier_bubble_min_radius", frt_bubble_radius_, 0.5);
  nh.param("bubble_topo/cube_discrete_size", cube_discrete_size, 0.3);

  nh.getParam("parallel_astar/update_connection_timeout", update_connection_timeout);
  nh.getParam("parallel_astar/insert_node_timeout", insert_node_timeout);

  nh.getParam("max_update_region_num", max_update_region_num_);
  update_idx_vec_.reserve(100);
  global_path_.reserve(200);
  x_len = std::ceil((max_bd - min_bd).x() / init_region_size_x_);
  y_len = std::ceil((max_bd - min_bd).y() / init_region_size_y_);
  z_len = std::ceil((max_bd - min_bd).z() / init_region_size_z_);
  for (size_t i = 0; i < x_len; i++)
    for (int j = 0; j < y_len; j++)
      for (int k = 0; k < z_len; k++) {
        Eigen::Vector3i idx(i, j, k);
        RegionNode::Ptr region_node = std::make_shared<RegionNode>(idx);
        Eigen::Vector3f hb, lb;
        index2boundary(idx, lb, hb);
        if (!hasOverlapWithBox(lb, hb))
          continue;
        reg_map_idx2ptr_[idx] = region_node;
      }
  // 建立一个octree作为球形覆盖的check-point
  check_pts_.clear();
  for (float x = 0; x < init_region_size_x_; x += cube_discrete_size) {
    for (float y = 0; y < init_region_size_y_; y += cube_discrete_size) {
      for (float z = 0; z < init_region_size_z_; z += cube_discrete_size) {
        check_pts_.emplace_back(x, y, z);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr check_pts_pc(new pcl::PointCloud<pcl::PointXYZ>);
  check_pts_pc->points = check_pts_;
  check_pts_octree_.setResolution(cube_discrete_size);
  check_pts_octree_.setInputCloud(check_pts_pc);
  check_pts_octree_.addPointsFromInputCloud();
}

BubbleNode::BubbleNode(double radius, Eigen::Vector3f center) {
  radius_ = radius;
  center_ = center;
}

RegionNode::RegionNode(Eigen::Vector3i region_idx) {
  region_idx_ = region_idx;
  his_odom_id_ = -1;
}

RegionNode::Ptr TopoGraph::getRegionNode(const Eigen::Vector3i &region_idx_) {
  if (reg_map_idx2ptr_.find(region_idx_) == reg_map_idx2ptr_.end()) {
    return nullptr;
  }
  return reg_map_idx2ptr_[region_idx_];
}

void TopoGraph::getIndex(const Eigen::Vector3f &point, Eigen::Vector3i &region_idx_) {
  region_idx_.x() = int((point[0] - min_bd[0]) / init_region_size_x_);
  region_idx_.y() = int((point[1] - min_bd[1]) / init_region_size_y_);
  region_idx_.z() = int((point[2] - min_bd[2]) / init_region_size_z_);
}

bool TopoGraph::index2boundary(const Eigen::Vector3i &region_idx_, Eigen::Vector3f &low_bd, Eigen::Vector3f &high_bd) {
  low_bd = Eigen::Vector3f(min_bd[0] + region_idx_.x() * init_region_size_x_, min_bd[1] + region_idx_.y() * init_region_size_y_,
                           min_bd[2] + region_idx_.z() * init_region_size_z_);
  high_bd = low_bd + Eigen::Vector3f(init_region_size_x_, init_region_size_y_, init_region_size_z_);
  return true;
}


void BubbleUnionSet::init(const std::vector<BubbleNode::Ptr> &bubbles_) {
  parent.clear();
  rank.clear();
  clusters.clear();
  bubbles.clear();
  topo_map.clear();
  bubbles = bubbles_;
  rank.reserve(bubbles.size());
  parent.reserve(bubbles.size());
  clusters.reserve(bubbles.size());
  parent.clear();
  rank.clear();
  for (auto &b : bubbles) {
    parent[b] = b;
    rank[b] = 0;
  }
}

BubbleNode::Ptr BubbleUnionSet::find(BubbleNode::Ptr b) {
  if (parent[b] != b) {
    parent[b] = find(parent[b]);
  }
  return parent[b];
}

void BubbleUnionSet::merge(BubbleNode::Ptr b1, BubbleNode::Ptr b2) {
  b1 = find(b1);
  b2 = find(b2);
  if (rank[b1] > rank[b2]) {
    parent[b2] = b1;
  } else {
    parent[b1] = b2;
    if (rank[b1] == rank[b2]) {
      rank[b2]++;
    }
  }
}

void BubbleUnionSet::getClusters() {
  clusters.clear();
  for (auto &b : parent) {
    if (b.second == b.first) {
      clusters.push_back(b.first);
      topo_map[b.first] = TopoNode::Ptr(new TopoNode);
    }
  }
  for (auto &b : bubbles) {
    auto topo_ptr = topo_map[find(b)];
    topo_ptr->bubbles_.push_back(b);
  }
}

bool TopoGraph::graphSearch(const TopoNode::Ptr &start_node, const TopoNode::Ptr &end_node, std::vector<TopoNode::Ptr> &path, double time_out,
                            bool kino, std::unordered_set<pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> last_path) {
  path.clear();
  std::unordered_map<TopoNode::Ptr, float> g_score, f_score;
  std::unordered_map<TopoNode::Ptr, TopoNode::Ptr> parent_map;
  std::unordered_set<TopoNode::Ptr> close_set, open_set_set_;
  float tie_breaker_ = 1.0 + 1.0 / 1000;
  std::priority_queue<std::pair<float, TopoNode::Ptr>, std::vector<std::pair<float, TopoNode::Ptr>>, std::greater<std::pair<float, TopoNode::Ptr>>>
  open_set;
  auto getHeuristic = [&](const TopoNode::Ptr &n) -> float { return tie_breaker_ * (n->center_ - end_node->center_).norm(); };
  auto backtrack = [&]() {
    TopoNode::Ptr cur_node = end_node;
    path.push_back(cur_node);
    while (parent_map.find(cur_node) != parent_map.end()) {
      cur_node = parent_map[cur_node];
      path.push_back(cur_node);
    }
    std::reverse(path.begin(), path.end());
  };
  auto cur_node = start_node;
  std::unordered_map<TopoNode::Ptr, Eigen::Vector3f> node_vel;
  g_score[cur_node] = 0.0;
  f_score[cur_node] = getHeuristic(cur_node);
  open_set.push({f_score[cur_node], cur_node});
  open_set_set_.insert(cur_node);
  const auto t1 = ros::Time::now();
  while (!open_set.empty()) {
    cur_node = open_set.top().second;
    open_set_set_.erase(cur_node);
    open_set.pop();
    close_set.insert(cur_node);
    if (cur_node == end_node) {
      backtrack();
      return true;
    }
    if ((ros::Time::now() - t1).toSec() > time_out) {
      // ROS_ERROR("topo a* timeout");
      return false;
    }
    for (auto &neighbor : cur_node->neighbors_) {
      // if (!neighbor->reachable_)
      //   continue;
      if (close_set.find(neighbor) != close_set.end())
        continue;

      float tentative_g_score;
      if (kino) {
        if (last_path.find({cur_node, neighbor}) != last_path.end()) {
          // tentative_g_score = g_score[cur_node] + 1e-3 * cur_node->weight_[neighbor];
          tentative_g_score = g_score[cur_node] + 0 * cur_node->weight_[neighbor];
        } else
          tentative_g_score = g_score[cur_node] + cur_node->weight_[neighbor];
      } else {
        tentative_g_score = g_score[cur_node] + cur_node->weight_[neighbor];
      }
      if (open_set_set_.find(neighbor) == open_set_set_.end() || tentative_g_score < g_score[neighbor]) {
        parent_map[neighbor] = cur_node;
        g_score[neighbor] = tentative_g_score;
        f_score[neighbor] = g_score[neighbor] + getHeuristic(neighbor);
        open_set.push({f_score[neighbor], neighbor});
        open_set_set_.insert(neighbor);
      } else
        continue;
    }
  }
  return false;
}

void TopoGraph::cauculateMemoryConsumption() {
  size_t graph_cost = 0;
  size_t graph_cost2 = 0;

  double node_size = 0;
  double nbr_size = 0;
  double ur_nbr_size = 0;
  for (auto &[_, region] : reg_map_idx2ptr_) {
    size_t single_cost = 0;
    size_t single_cost2 = 0;
    if (region->topo_nodes_.empty())
      continue;
    for (auto &topo : region->topo_nodes_) {
      node_size++;
      nbr_size += topo->neighbors_.size();
      ur_nbr_size += topo->unreachable_nbrs_.size();
      if (topo->neighbors_.size() != topo->paths_.size()) {
        ROS_ERROR("memory error 644");
        exit(1);
      }
      if (topo->neighbors_.size() != topo->weight_.size()) {
        ROS_ERROR("memory error 648");
        exit(1);
      }

      single_cost += sizeof(bool);                                                       // is_viewpoint_
      single_cost += sizeof(float);                                                      // yaw_
      single_cost += sizeof(Eigen::Vector3f);                                            // center
      single_cost += (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();              // neighbors_
      single_cost += (sizeof(TopoNode::Ptr) + 2) * topo->unreachable_nbrs_.size();       // unreachable_nbrs_
      single_cost += (sizeof(float) + sizeof(TopoNode::Ptr) + 1) * topo->weight_.size(); // weight_
      single_cost2 = single_cost;
      single_cost2 -= (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();
      for (auto &nei : topo->neighbors_) {
        single_cost += sizeof(Eigen::Vector3f) * topo->paths_[nei].size(); // paths_
        single_cost2 += sizeof(Eigen::Vector3f) * topo->paths_[nei].size() / 2.0;
      }
    }
    single_cost += (sizeof(Eigen::Vector3i) + sizeof(RegionNode::Ptr) + 1); // region_node key-value pair
    graph_cost += single_cost;
    graph_cost2 += single_cost2;
  }
  for (auto &topo : history_odom_nodes_) {
    size_t single_cost = 0;
    size_t single_cost2 = 0;
    single_cost += sizeof(bool);                                                       // is_viewpoint_
    single_cost += sizeof(float);                                                      // yaw_
    single_cost += sizeof(Eigen::Vector3f);                                            // center
    single_cost += (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();              // neighbors_
    single_cost += (sizeof(TopoNode::Ptr) + 2) * topo->unreachable_nbrs_.size();       // unreachable_nbrs_
    single_cost += (sizeof(float) + sizeof(TopoNode::Ptr) + 1) * topo->weight_.size(); // weight_
    single_cost2 = single_cost;
    single_cost2 -= (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();
    for (auto &nei : topo->neighbors_) {
      single_cost += sizeof(Eigen::Vector3f) * topo->paths_[nei].size(); // paths_
      single_cost2 += sizeof(Eigen::Vector3f) * topo->paths_[nei].size() / 2.0;
    }

    single_cost += (sizeof(Eigen::Vector3i) + sizeof(RegionNode::Ptr) + 1); // region_node key-value pair
    graph_cost += single_cost;
    graph_cost2 += single_cost2;
  }

}

int TopoGraph::getBoxId(const Eigen::Vector3f &pt) {
  auto inbox = [&](const Eigen::Vector3f &pt, const Eigen::Vector3f &min, const Eigen::Vector3f &max) -> bool {
    for (size_t i = 0; i < 3; i++) {
      if (pt(i) < min(i) || pt(i) > max(i))
        return false;
    }
    return true;
  };

  for (size_t i = 0; i < lidar_map_interface_->lp_->box_num_; i++) {
    Eigen::Vector3f min_ = lidar_map_interface_->lp_->global_box_min_boundary_vec_[i];
    Eigen::Vector3f max_ = lidar_map_interface_->lp_->global_box_max_boundary_vec_[i];
    if (inbox(pt, min_, max_))
      return i;
  }
  return -1;
}

void BubbleUnionSet::unionSetCluster(const vector<BubbleNode::Ptr> &bubbles, vector<TopoNode::Ptr> &topos, Eigen::Vector3f &center) {
  auto is_bubble_connected = [&](BubbleNode::Ptr b1, BubbleNode::Ptr b2) -> bool {
    double center_distance = (b1->center_ - b2->center_).norm();
    return center_distance < (b1->radius_ + b2->radius_) - 0.5;
  };
  init(bubbles);
  for (size_t i = 0; i < bubbles.size(); i++)
    for (int j = i + 1; j < bubbles.size(); j++)
      if (is_bubble_connected(bubbles[i], bubbles[j]))
        merge(bubbles[i], bubbles[j]);
  getClusters();
  // getTopoNodes(topos, center);
  for (auto &tpair : topo_map) {
    auto node = tpair.second;
    if (node->bubbles_.empty())
      continue;
    BubbleNode::Ptr max_raduis_bubble = node->bubbles_[0];
    BubbleNode::Ptr center_big_bubble = node->bubbles_[0];
    double dis2center = (center_big_bubble->center_ - center).norm();
    for (auto &b : node->bubbles_) {
      if (b->radius_ > max_raduis_bubble->radius_)
        max_raduis_bubble = b; // 半径最大的bubble
      double dis2center_now = (b->center_ - center).norm();
      if (dis2center_now < dis2center && b->radius_ > min_topobubble_radius_)
        center_big_bubble = b; // 半径大于阈值，距离中心最近的bubble
    }
    if (center_big_bubble->radius_ > min_topobubble_radius_)
      node->center_ = center_big_bubble->center_;
    else
      node->center_ = max_raduis_bubble->center_;
    node->is_viewpoint_ = false;
    topos.push_back(node);
    vector<BubbleNode::Ptr>().swap(node->bubbles_);
  }
}

void TopoGraph::overlap(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &overlap) {
  HashMap map;
  for (auto &node : set1) {
    Eigen::Vector3i idx;
    posToIndex(node->center_, idx);
    map[idx] = node;
  }
  overlap.clear();
  for (auto &node : set2) {
    Eigen::Vector3i idx;
    posToIndex(node->center_, idx);
    if (map.find(idx) != map.end()) {
      overlap.push_back(node);
    }
  }
}

void TopoGraph::setdiff(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &set_1diff2) {
  HashMap map;
  for (auto &node : set2) {
    Eigen::Vector3i idx;
    posToIndex(node->center_, idx);
    map[idx] = node;
  }
  set_1diff2.clear();
  for (auto &node : set1) {
    Eigen::Vector3i idx;
    posToIndex(node->center_, idx);
    if (map.find(idx) == map.end())
      set_1diff2.push_back(node);
  }
}

void TopoGraph::removeNodes(vector<TopoNode::Ptr> &nodes) {

  // region_set
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    Eigen::Vector3i region_idx;
    getIndex(node->center_, region_idx);
    auto region_node = getRegionNode(region_idx);
    ROS_ASSERT(region_node != nullptr);
    // if (region_node == nullptr) {
    //   continue;
    //   debug_exit("TopoGraph::removeNodes :region_node == nullptr ");
    // }
    region_node->topo_nodes_.erase(node);
  }

  // nbrs
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    for (auto &nbr : node->neighbors_) {
      // if (nbr->is_history_odom_node_)
      //   continue;
      nbr->neighbors_.erase(node);
      nbr->paths_.erase(node);
      nbr->weight_.erase(node);
      nbr->unreachable_nbrs_.erase(node);
    }
    node->unreachable_nbrs_.clear();
    node->neighbors_.clear();
    node->weight_.clear();
    node->paths_.clear();
  }
}

void TopoGraph::updateRemainedConnections(vector<TopoNode::Ptr> &nodes) {

  // 处理已有的邻居：检查，如果不行就重新搜索
  auto checkNbr = [&](PtrPair::iter_elem &elem) {
    auto node = elem.p1;
    auto nbr = elem.p2;

    vector<Eigen::Vector3f> path = node->paths_[nbr];
    bool safe = parallel_bubble_astar_->collisionCheck_shortenPath(path);
    if (safe) {
      elem.insert = true;
      elem.path = path;
      return;
    }
    // 并不安全：重新搜路

    path.clear();
    // int res =
    // parallel_bubble_astar_->search(node->center_, nbr->center_, path, update_connection_timeout);
    int res = searchPathWithBoundary(node->center_, nbr->center_, update_connection_timeout, path);

    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
    } else {
      elem.insert = false;
    }
  };
  // 处理可能的邻居：搜一条路看看
  auto testPreNbr = [&](PtrPair::iter_elem &elem) {
    auto node = elem.p1;
    auto pre_nbr = elem.p2;
    // if ((node->center_ - pre_nbr->center_).norm() > 3.0) {
    //   elem.insert = false;
    //   return;
    // }
    vector<Eigen::Vector3f> path;
    int res = searchPathWithBoundary(node->center_, pre_nbr->center_, update_connection_timeout, path);
    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
    } else {
      elem.insert = false;
    }
  };
  PtrPair edge2test, edge2check;
  for (auto &node : nodes) {
    vector<TopoNode::Ptr> pre_nbrs;
    getPreNbrs(node, pre_nbrs);
    unordered_set<TopoNode::Ptr> pre_nbrs_set(pre_nbrs.begin(), pre_nbrs.end());
    for (auto &nbr : node->neighbors_) {
      if (nbr->is_history_odom_node_)
        continue;
      pre_nbrs_set.insert(nbr);
    }
    unordered_set<TopoNode::Ptr> pre_nbrs_set_tmp;
    unordered_map<TopoNode::Ptr, uint8_t> unreachable_nbrs_tmp;
    for (auto &pre_nbr : pre_nbrs_set) {
      if (node->unreachable_nbrs_.count(pre_nbr) && node->unreachable_nbrs_[pre_nbr] > 2) {
        continue;
      }
      pre_nbrs_set_tmp.insert(pre_nbr);
    }
    for (auto &pre_nbr : node->unreachable_nbrs_) {
      if (pre_nbrs_set.count(pre_nbr.first) && pre_nbr.first != odom_node_)
        unreachable_nbrs_tmp.insert(pre_nbr);
    }
    pre_nbrs_set_tmp.swap(pre_nbrs_set);
    node->unreachable_nbrs_.swap(unreachable_nbrs_tmp);
    for (auto &pre_nbr : pre_nbrs_set) {
      if (node->neighbors_.find(pre_nbr) == node->neighbors_.end()) {
        edge2test.insert(node, pre_nbr);
        // testPreNbr(node, pre_nbr);
      } else {
        // checkNbr(node, pre_nbr);
        edge2check.insert(node, pre_nbr);
      }
    }
  }
  edge2test.flatten();
  edge2check.flatten();
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : edge2test.flatten_data) {
    testPreNbr(elem);
  }
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : edge2check.flatten_data) {
    checkNbr(elem);
  }
  edge2test.flatten_data.insert(edge2test.flatten_data.end(), edge2check.flatten_data.begin(), edge2check.flatten_data.end());
  for (auto &elem : edge2test.flatten_data) {
    if (elem.insert) {
      auto node1 = elem.p1;
      auto node2 = elem.p2;
      node1->paths_[node2] = elem.path;
      std::reverse(elem.path.begin(), elem.path.end());
      node2->paths_[node1] = elem.path;
      double cost;
      parallel_bubble_astar_->calculatePathCost(elem.path, cost);
      node1->unreachable_nbrs_.erase(node2);
      node2->unreachable_nbrs_.erase(node1);
      node1->neighbors_.insert(node2);
      node2->neighbors_.insert(node1);
      node1->weight_[node2] = cost;
      node2->weight_[node1] = cost;
    } else {
      auto node1 = elem.p1;
      auto node2 = elem.p2;
      node1->neighbors_.erase(node2);
      node2->neighbors_.erase(node1);
      node1->weight_.erase(node2);
      node2->weight_.erase(node1);
      node1->paths_.erase(node2);
      node2->paths_.erase(node1);
      if (node1->unreachable_nbrs_.count(node2)) {
        node1->unreachable_nbrs_[node2]++;
      } else {
        node1->unreachable_nbrs_[node2] = 1;
      }
      if (node2->unreachable_nbrs_.count(node1)) {
        node2->unreachable_nbrs_[node1]++;
      } else {
        node2->unreachable_nbrs_[node1] = 1;
      }
    }
  }
}

void TopoGraph::getPreNbrs(TopoNode::Ptr &node, vector<TopoNode::Ptr> &nbrs) {
  Eigen::Vector3i idx, odom_idx;
  getIndex(node->center_, idx);
  nbrs.clear();
  // getIndex(odom_node_->center_, odom_idx);
  // Eigen::Vector3i diff = (idx - odom_idx).cwiseAbs();
  // if (diff.maxCoeff() <= 1)
  //   nbrs.push_back(odom_node_);
  vector<Eigen::Vector3i> steps1{Eigen::Vector3i(0, 0, 0),  Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(0, 1, 0),
                                 Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1)};
  vector<Eigen::Vector3i> steps2{Eigen::Vector3i(1, 1, 0), Eigen::Vector3i(1, -1, 0), Eigen::Vector3i(-1, 1, 0), Eigen::Vector3i(-1, -1, 0),
                                 Eigen::Vector3i(1, 0, 1), Eigen::Vector3i(1, 0, -1), Eigen::Vector3i(-1, 0, 1), Eigen::Vector3i(-1, 0, -1),
                                 Eigen::Vector3i(0, 1, 1), Eigen::Vector3i(0, 1, -1), Eigen::Vector3i(0, -1, 1), Eigen::Vector3i(0, -1, -1)};

  // for (int i = 0; i < steps1.size() + steps2.size(); i++) {
  //   if (i >= steps1.size() && nbrs.size() > 4)
  //     break;
  for (int i = 0; i < steps1.size() ; i++) {
    Eigen::Vector3i step = i < steps1.size() ? steps1[i] : steps2[i - steps1.size()];
    Eigen::Vector3i nbr_idx = idx + step;
    auto nbr_region_node = getRegionNode(nbr_idx);
    if (nbr_region_node == nullptr)
      continue;
    for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
      if (nbr_topo_node == nullptr) {
        cout << "wtf 970" << endl;
        continue;
      }
      if (nbr_topo_node == node)
        continue;
      // if (nbr_topo_node->is_viewpoint_ && node->is_viewpoint_)
      //   continue;
      nbrs.push_back(nbr_topo_node);
    }
  }
}

// void TopoGraph::getPreNbrs(TopoNode::Ptr &node, vector<TopoNode::Ptr> &nbrs) {
//   Eigen::Vector3i idx, odom_idx;
//   getIndex(node->center_, idx);
//   if (!node->is_viewpoint_) {
//     for (int i = 0; i < 3; i++) {
//       for (int j = -1; j <= 1; j++) {
//         Eigen::Vector3i idx_tmp = idx;
//         idx_tmp[i] += j;
//         auto nbr_region_node = getRegionNode(idx_tmp);
//         if (nbr_region_node == nullptr)
//           continue;
//         for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
//           if (nbr_topo_node == nullptr)
//             continue;
//           if (nbr_topo_node == node)
//             continue;
//           nbrs.push_back(nbr_topo_node);
//         }
//       }
//     }
//   } else {
//     for (int i = -1; i <= 1; i++) {
//       for (int j = -1; j <= 1; j++) {
//         for (int k = -1; k <= 1; k++) {
//           Eigen::Vector3i idx_tmp = idx;
//           idx_tmp[0] += i;
//           idx_tmp[1] += j;
//           idx_tmp[2] += k;
//           auto nbr_region_node = getRegionNode(idx_tmp);
//           if (nbr_region_node == nullptr)
//             continue;
//           for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
//             if (nbr_topo_node == nullptr)
//               continue;
//             if (nbr_topo_node == node)
//               continue;
//             nbrs.push_back(nbr_topo_node);
//           }
//         }
//       }
//     }
//   }
// }

void TopoGraph::insertNodes(vector<TopoNode::Ptr> &nodes, bool only_raycast) {
  // insert到region里
  if (nodes.empty())
    return;
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    Eigen::Vector3i region_idx;
    getIndex(node->center_, region_idx);
    // else
    auto region_node = getRegionNode(region_idx);
    // if (region_node == nullptr) {
    //   continue;
    // }
    ROS_ASSERT(region_node != nullptr);
    region_node->topo_nodes_.insert(node);
  }

  // 找到邻居region和自己region的其他节点

  std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> ptr_pair_set;
  vector<pair<TopoNode::Ptr, TopoNode::Ptr>> pair_vector; // 使用vector支持并行运算
  for (auto &node : nodes) {
    vector<TopoNode::Ptr> nbrs;
    getPreNbrs(node, nbrs);
    for (auto &nbr : nbrs) {
      if (ptr_pair_set.find({node, nbr}) == ptr_pair_set.end()) {
        pair_vector.push_back({node, nbr});
        ptr_pair_set.insert({node, nbr});
        ptr_pair_set.insert({nbr, node});
      }
    }
  }

  // 获得节点对的vector
  vector<vector<Eigen::Vector3f>> path_vec; // 初始是start和end两个点, 算完是path+一个zero/one表示成功/失败
  path_vec.resize(pair_vector.size());

  // 并行A*搜索路径并写入结果
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (size_t i = 0; i < path_vec.size(); i++) {
    Eigen::Vector3f start = pair_vector[i].first->center_;
    Eigen::Vector3f end = pair_vector[i].second->center_;
    vector<Eigen::Vector3f> path;
    int res;
    if (!only_raycast) {
      res = searchPathWithBoundary(start, end, insert_node_timeout, path);
    } else
      res = parallel_bubble_astar_->search(start, end, path, insert_node_timeout, false, true);
    if (res != ParallelBubbleAstar::REACH_END)
      path.push_back(Eigen::Vector3f::Zero());
    else if (!only_raycast) {
      bool safe = parallel_bubble_astar_->collisionCheck_shortenPath(path);
      if (safe)
        path.push_back(Eigen::Vector3f::Ones());
      else
        path.push_back(Eigen::Vector3f::Zero());
    } else {
      path.push_back(Eigen::Vector3f::Ones()); // 1表示安全，0表示危险
    }
    path_vec[i].swap(path);
  }

  // 串行更新节点
  for (size_t i = 0; i < path_vec.size(); i++) {
    if (path_vec[i].back().norm() < 0.5)
      continue;
    auto node1 = pair_vector[i].first;
    auto node2 = pair_vector[i].second;
    node1->neighbors_.insert(node2);
    node2->neighbors_.insert(node1);
    path_vec[i].pop_back();
    node1->paths_[node2] = path_vec[i];
    std::reverse(path_vec[i].begin(), path_vec[i].end());
    node2->paths_[node1] = path_vec[i];
    double cost;
    parallel_bubble_astar_->calculatePathCost(path_vec[i], cost);
    node1->weight_[node2] = cost;
    node2->weight_[node1] = cost;
  }
}

void TopoGraph::getRegionsToUpdate() {
  update_idx_vec_.clear();
  viewpoints_update_region_arr_.clear();
  toponodes_update_region_arr_.clear();
  unordered_set<RegionNode::Ptr> region_set;
  for (auto &pt : lidar_map_interface_->ld_->lidar_cloud_.points) {
    Eigen::Vector3f pt3d = pt.getArray3fMap();
    if ((pt3d - lidar_map_interface_->ld_->lidar_pose_).norm() > lidar_map_interface_->lp_->max_ray_length_)
      pt3d = lidar_map_interface_->ld_->lidar_pose_ + lidar_map_interface_->lp_->max_ray_length_ * (pt3d - lidar_map_interface_->ld_->lidar_pose_) /
                                                   (pt3d - lidar_map_interface_->ld_->lidar_pose_).norm();
    Eigen::Vector3i region_idx;
    getIndex(pt3d, region_idx);
    auto region = getRegionNode(region_idx);
    if (region != nullptr)
      region_set.insert(region);
  }
  for (auto &region : region_set) {
    toponodes_update_region_arr_.push_back(region);
  }
  auto shorten_by_distance_insert_update_arr = [&](vector<RegionNode::Ptr> &arr) {
    std::sort(arr.begin(), arr.end(), [this](const RegionNode::Ptr &region1, const RegionNode::Ptr &region2) {
      Eigen::Vector3f lb1, hb1, lb2, hb2;
      index2boundary(region1->region_idx_, lb1, hb1);
      index2boundary(region2->region_idx_, lb2, hb2);
      Eigen::Vector3f diff1 = ((hb1 + lb1) * 0.5 - lidar_map_interface_->ld_->lidar_pose_);
      Eigen::Vector3f diff2 = ((hb2 + lb2) * 0.5 - lidar_map_interface_->ld_->lidar_pose_);
      double dist1 = diff1.norm();
      double dist2 = diff2.norm();
      return dist1 < dist2;
    });
    arr.resize(std::min((int)arr.size(), max_update_region_num_));

    // 去重
    unordered_set<RegionNode::Ptr> region2update(arr.begin(), arr.end());
    arr = vector<RegionNode::Ptr>(region2update.begin(), region2update.end());
  };
  // shorten_by_distance_insert_update_arr(toponodes_update_region_arr_);

  // 向四周发射射线，超过当前单位球大概一格子的范围
  double step_size = min(init_region_size_x_, init_region_size_y_);
  step_size = min(step_size, init_region_size_z_);
  step_size /= 2.0;
  for (auto &region : toponodes_update_region_arr_) {
    Eigen::Vector3f lb, hb, goal;
    index2boundary(region->region_idx_, lb, hb);
    goal = 0.5 * (lb + hb);
    Eigen::Vector3f dir = goal - lidar_map_interface_->ld_->lidar_pose_;
    int step_num = (int)(dir.norm() / step_size) + 1;
    dir.normalize();
    Eigen::Vector3f step = dir * step_size;
    for (int i = 0; i < step_num; ++i) {
      Eigen::Vector3f pos = lidar_map_interface_->ld_->lidar_pose_ + step * i;
      Eigen::Vector3i region_idx;
      getIndex(pos, region_idx);
      auto region = getRegionNode(region_idx);
      if (region != nullptr)
        region_set.insert(region);
    }
  }

  for (auto &region : region_set) {
    toponodes_update_region_arr_.push_back(region);
  }

  shorten_by_distance_insert_update_arr(toponodes_update_region_arr_);
  for (auto &region : toponodes_update_region_arr_) {
    update_idx_vec_.push_back(region->region_idx_);
  }
}

void TopoGraph::updateSkeleton() {
  parallel_bubble_astar_->reset();
  vector<TopoNode::Ptr> nodes2insert, nodes_remained, nodes2remove, new_nodes, old_nodes;
  mutex new_nodes_mtx;
  ros::Time t0 = ros::Time::now();
  for (auto &region : toponodes_update_region_arr_) {
    for (auto &node : region->topo_nodes_) {
      if (!node->is_viewpoint_)
        old_nodes.push_back(node);
    }
  }
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &region_ptr : toponodes_update_region_arr_) {
    Eigen::Vector3f lb, hb;
    index2boundary(region_ptr->region_idx_, lb, hb);
    vector<BubbleNode::Ptr> tmp_bubbles;
    vector<bool> check_pt_flag(check_pts_.size(), false);
    for (int i = 0; i < check_pts_.size(); ++i) {
      Eigen::Vector3f pt = check_pts_[i].getArray3fMap();
      pt += lb;
      if (!lidar_map_interface_->IsInBox(pt))
        check_pt_flag[i] = true;
    }
    generateBubble(lb, hb, tmp_bubbles, check_pt_flag);
    BubbleUnionSet::Ptr union_set_ = std::make_shared<BubbleUnionSet>(bubble_min_radius_); // TODO: 这个参数是topo节点2occ的最小距离
    vector<TopoNode::Ptr> new_nodes_region;
    Eigen::Vector3f region_center = (lb + hb) * 0.5;
    union_set_->unionSetCluster(tmp_bubbles, new_nodes_region, region_center);
    new_nodes_mtx.lock();
    for (auto &node : new_nodes_region) {
      if (!lidar_map_interface_->IsInBox(node->center_))
        continue;
      new_nodes.emplace_back(node);
    }
    new_nodes_mtx.unlock();
  }

  overlap(new_nodes, old_nodes, nodes_remained);
  setdiff(old_nodes, new_nodes, nodes2remove);
  setdiff(new_nodes, old_nodes, nodes2insert);

  ros::Time t1 = ros::Time::now();
  removeNodes(nodes2remove);
  ros::Time t2 = ros::Time::now();
  updateRemainedConnections(nodes_remained);
  ros::Time t3 = ros::Time::now();
  insertNodes(nodes2insert);
  ros::Time t4 = ros::Time::now();
  vector<TopoNode::Ptr> unreachable_nodes;


}

void TopoGraph::updateOdomNode(Eigen::Vector3f &odom_pos, float &yaw) {
  struct PairPtrHash {
    std::size_t operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
      return std::hash<TopoNode::Ptr>()(p.first) ^ std::hash<TopoNode::Ptr>()(p.second);
    }
  };

  Eigen::Vector3i idx;
  getIndex(lidar_map_interface_->ld_->lidar_pose_, idx);
  vector<TopoNode::Ptr> pre_nbrs;
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i tmp_idx = idx;
        tmp_idx(0) = idx(0) + i;
        tmp_idx(1) = idx(1) + j;
        tmp_idx(2) = idx(2) + k;
        if (tmp_idx.x() == 0 && tmp_idx.y() == 0 && tmp_idx.z() != 0)
          continue;
        auto region = getRegionNode(tmp_idx);
        if (region) {
          for (auto &topo : region->topo_nodes_) {
            if (topo == odom_node_)
              continue;
            // if(topo->is_viewpoint_)
            //   continue;
            pre_nbrs.emplace_back(topo);
          }
        }
      }
  std::unordered_map<std::pair<TopoNode::Ptr, TopoNode::Ptr>, vector<Eigen::Vector3f>, PairPtrHash> edge2insert;
  mutex edge2insert_mtx;
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &nbr : pre_nbrs) {
    vector<Eigen::Vector3f> path;
    int res = parallel_bubble_astar_->search(odom_pos, nbr->center_, path, update_connection_timeout, true);
    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      edge2insert_mtx.lock();
      edge2insert.insert({std::make_pair(odom_node_, nbr), path});
      edge2insert_mtx.unlock();
    }
  }
  if (edge2insert.empty())
    return;
  // 更新odom节点
  odom_node_->center_ = odom_pos;
  odom_node_->yaw_ = yaw;
  // if (edge2insert.size() > 0) {
  for (auto &nei : odom_node_->neighbors_) {
    nei->neighbors_.erase(odom_node_);
    nei->weight_.erase(odom_node_);
    nei->paths_.erase(odom_node_);
    nei->unreachable_nbrs_.erase(odom_node_);
  }
  odom_node_->neighbors_.clear();
  odom_node_->weight_.clear();
  odom_node_->paths_.clear();
  odom_node_->unreachable_nbrs_.clear();
  for (auto &edge : edge2insert) {
    odom_node_->neighbors_.insert(edge.first.second);
    odom_node_->paths_.insert({edge.first.second, edge.second});
    double cost;
    // parallel_bubble_astar_->calculatePathCost(edge.second, cost);
    // odom_node_->weight_[edge.first.second] = cost;
    odom_node_->weight_[edge.first.second] = 0;
    // auto nbr = edge.first.second;
    // nbr->neighbors_.insert(odom_node_);
    // nbr->weight_[odom_node_] = cost;
    // vector<Eigen::Vector3f> path = edge.second;
    // std::reverse(path.begin(), path.end());
    // nbr->paths_[odom_node_] = path;
  }
  // }
}

void TopoGraph::removeNode(TopoNode::Ptr &node) {
  if (node == nullptr)
    return;
  Eigen::Vector3i region_idx;
  getIndex(node->center_, region_idx);
  auto region_node = getRegionNode(region_idx);
  if (region_node == nullptr) {
    debug_exit("TopoGraph::removeNodes :region_node == nullptr ");
  }
  region_node->topo_nodes_.erase(node);

  // nbrs
  for (auto &nbr : node->neighbors_) {
    nbr->neighbors_.erase(node);
    nbr->paths_.erase(node);
    nbr->weight_.erase(node);
    nbr->unreachable_nbrs_.erase(node);
  }
  node->unreachable_nbrs_.clear();
  node->neighbors_.clear();
  node->weight_.clear();
  node->paths_.clear();
}

void TopoGraph::insertNode(TopoNode::Ptr &new_node, vector<TopoNode::Ptr> &nbr_nodes, vector<vector<Eigen::Vector3f>> &paths) {
  Eigen::Vector3i region_idx;
  getIndex(new_node->center_, region_idx);
  auto region_node = getRegionNode(region_idx);
  if (region_node == nullptr) {
    debug_exit("TopoGraph::insertNodes :region_node == nullptr ");
  }
  region_node->topo_nodes_.insert(new_node);
  for (int i = 0; i < nbr_nodes.size(); i++) {
    new_node->neighbors_.insert(nbr_nodes[i]);
    nbr_nodes[i]->neighbors_.insert(new_node);
    auto path = paths[i];
    new_node->paths_.insert({nbr_nodes[i], path});
    std::reverse(path.begin(), path.end());
    nbr_nodes[i]->paths_.insert({new_node, path});
    double cost;
    parallel_bubble_astar_->calculatePathCost(path, cost);
    new_node->weight_[nbr_nodes[i]] = cost;
    nbr_nodes[i]->weight_[new_node] = cost;
  }
}



int TopoGraph::searchPathWithBoundary(const Eigen::Vector3f &start, const Eigen::Vector3f &end, double &time_out, vector<Eigen::Vector3f> &path) {
  Eigen::Vector3f bd_min, bd_max;
  for (int i = 0; i < 3; i++) {
    bd_min(i) = min(start(i), end(i));
    bd_max(i) = max(start(i), end(i));
  }
  bd_min -= Eigen::Vector3f(init_region_size_x_ / 2.0, init_region_size_y_ / 2.0, init_region_size_z_ / 2.0);
  bd_max += Eigen::Vector3f(init_region_size_x_ / 2.0, init_region_size_y_ / 2.0, init_region_size_z_ / 2.0);
  int res = parallel_bubble_astar_->search(start, end, path, time_out, false, false, bd_min, bd_max);
  return res;
}

double TopoGraph::getPathLength(const vector<TopoNode::Ptr> &topo_path) {
  vector<Eigen::Vector3f> path;
  for (int i = 0; i < topo_path.size() - 1; i++) {
    auto back = topo_path[i];
    auto front = topo_path[i + 1];
    for (auto &pt : back->paths_[front]) {
      path.emplace_back(pt);
    }
  }
  double length = 0.0;
  for (int i = 0; i < path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

bool TopoGraph::hasOverlapWithBox(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd) {
  const static vector<Eigen::Vector3f> tmp_vec{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0, 1, 1}, {1, 0, 1}, {1, 1, 0}, {1, 1, 1}};
  for (auto &tmp : tmp_vec) {
    Eigen::Vector3f pt;
    for (int i = 0; i < 3; i++) {
      pt(i) = tmp(i) * low_bd(i) + (1 - tmp(i)) * high_bd(i);
    }
    if (lidar_map_interface_->IsInBox(pt))
      return true;
  }
  return false;
}