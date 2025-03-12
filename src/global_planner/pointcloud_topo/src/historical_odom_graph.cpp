#include "pointcloud_topo/graph.h"

void TopoGraph::updateHistoricalOdoms() {
  // step1 更新和toponode的边
  unordered_map<TopoNode::Ptr, unordered_set<TopoNode::Ptr>> node2remove;
  unordered_map<TopoNode::Ptr, unordered_set<TopoNode::Ptr>> node2check;
  unordered_map<TopoNode::Ptr, unordered_set<TopoNode::Ptr>> node2insert;
  unordered_set<TopoNode::Ptr> updating_history_odom_nodes;
  for (auto &region : toponodes_update_region_arr_) {
    if (region->his_odom_id_ < 0)
      continue;
    auto hodom = history_odom_nodes_[region->his_odom_id_];
    updating_history_odom_nodes.insert(hodom);
    for (auto &node : region->topo_nodes_) {
      if (hodom->neighbors_.count(node)) {
        node2check[hodom].insert(node); // 过去有，现在也有
      } else {
        node2insert[hodom].insert(node); // 过去没有，现在有
      }
    }
  }
  unordered_set<Eigen::Vector3i, v3i_hash> updating_region_idx_set;
  for (auto &region : toponodes_update_region_arr_) {
    updating_region_idx_set.insert(region->region_idx_);
  }
  // 过去有，现在没有 -> node2remove
  for (auto &hodom : updating_history_odom_nodes) {
    for (auto &old_nbr : hodom->neighbors_) {
      Eigen::Vector3i idx;
      getIndex(old_nbr->center_, idx);
      auto region = getRegionNode(idx);
      // if (node2check[hodom].count(old_nbr) || node2insert[hodom].count(old_nbr) || old_nbr->is_history_odom_node_)
      //   continue;
      if (old_nbr->is_history_odom_node_ || region == nullptr)
        continue;
      if (region->topo_nodes_.count(old_nbr))
        continue;
      node2remove[hodom].insert(old_nbr);
    }
  }

  for (auto &[hodom, old_nbrs] : node2remove) {
    for (auto &old_nbr : old_nbrs) {
      old_nbr->neighbors_.erase(hodom);
      old_nbr->paths_.erase(hodom);
      old_nbr->weight_.erase(hodom);
      old_nbr->unreachable_nbrs_.erase(hodom);

      hodom->neighbors_.erase(old_nbr);
      hodom->paths_.erase(old_nbr);
      hodom->weight_.erase(old_nbr);
      hodom->unreachable_nbrs_.erase(old_nbr);
    }
  }
  PtrPair edge2insert, edge2check;
  for (auto &[hodom, new_nbrs] : node2insert) {
    for (auto &new_nbr : new_nbrs) {
      edge2insert.insert(hodom, new_nbr);
    }
  }
  for (auto &[hodom, remain_nbrs] : node2check) {
    for (auto &remain_nbr : remain_nbrs) {
      edge2check.insert(hodom, remain_nbr);
    }
  }
  auto checkEdge = [&](PtrPair::iter_elem &elem) {
    auto hodom = elem.p1;
    auto nbr = elem.p2;
    if (hodom->is_history_odom_node_ && !nbr->is_history_odom_node_) {

    } else if (!hodom->is_history_odom_node_ && nbr->is_history_odom_node_) {
      swap(hodom, nbr);
    } else {
      ROS_ERROR("err 1328");
      exit(0);
    }
    vector<Eigen::Vector3f> path = hodom->paths_[nbr];
    bool safe = parallel_bubble_astar_->collisionCheck_shortenPath(path);
    if (safe) {
      elem.insert = true;
      elem.path = path;
      return;
    }
    path.clear();
    int res = searchPathWithBoundary(hodom->center_, nbr->center_, update_connection_timeout, path);
    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
    } else {
      elem.insert = false;
    }
  };
  auto insertEdge = [&](PtrPair::iter_elem &elem) {
    auto hodom = elem.p1;
    auto nbr = elem.p2;
    if (hodom->is_history_odom_node_ && !nbr->is_history_odom_node_) {

    } else if (!hodom->is_history_odom_node_ && nbr->is_history_odom_node_) {
      swap(hodom, nbr);
    }
    vector<Eigen::Vector3f> path;
    int res = searchPathWithBoundary(hodom->center_, nbr->center_, update_connection_timeout, path);
    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
    } else if (res == ParallelBubbleAstar::REACH_END && !parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      ROS_ERROR("err 104");
      exit(1);
    } else {
      elem.insert = false;
    }
  };
  edge2check.flatten();
  edge2insert.flatten();
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : edge2check.flatten_data) {
    checkEdge(elem);
  }
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : edge2insert.flatten_data) {
    insertEdge(elem);
  }
  edge2check.flatten_data.insert(edge2check.flatten_data.end(), edge2insert.flatten_data.begin(), edge2insert.flatten_data.end());
  for (auto &elem : edge2check.flatten_data) {
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
  // step2 更新odom和上一个odom之间的边
  static vector<Eigen::Vector3f> path2next_odom_node;
  path2next_odom_node.push_back(odom_node_->center_);
  bool insert = history_odom_nodes_.empty();
  if (!insert) {
    for (int i = history_odom_nodes_.size() - 1; i >= 0; i--) {
      auto hodom = history_odom_nodes_[i];
      if ((hodom->center_ - odom_node_->center_).norm() > 5.0)
        insert = true;
      else {
        insert = false;
        break;
      }
    }
  }
  Eigen::Vector3i idx;
  getIndex(odom_node_->center_, idx);
  if (getRegionNode(idx) == nullptr) {
    insert = false;
  } else if (lidar_map_interface_->getDisToOcc(odom_node_->center_) < parallel_bubble_astar_->safe_distance_) {
    insert = false;
  }
  if (insert) {
    auto path = path2next_odom_node;
    parallel_bubble_astar_->collisionCheck_shortenPath(path);
    path2next_odom_node.clear();
    TopoNode::Ptr new_odom_node = std::make_shared<TopoNode>();
    new_odom_node->center_ = odom_node_->center_;
    new_odom_node->is_history_odom_node_ = true;
    Eigen::Vector3i idx;
    getIndex(new_odom_node->center_, idx);
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          Eigen::Vector3i nbr_idx = idx + Eigen::Vector3i(i, j, k);
          auto region = getRegionNode(nbr_idx);
          if (region == nullptr)
            continue;
          if (region->his_odom_id_ >= 0)
            continue;
          region->his_odom_id_ = history_odom_nodes_.size();
        }
      }
    }
    // TODO: path节点小于2、不安全
    if (!history_odom_nodes_.empty()) {
      // 将odom-node删除
      auto pre_node = history_odom_nodes_.back();

      odom_node_->neighbors_.erase(pre_node);
      pre_node->neighbors_.erase(odom_node_);

      pre_node->weight_.erase(odom_node_);
      odom_node_->weight_.erase(pre_node);

      pre_node->paths_.erase(odom_node_);
      odom_node_->paths_.erase(pre_node);

      pre_node->neighbors_.insert(new_odom_node);
      new_odom_node->neighbors_.insert(pre_node);
      double weight;
      parallel_bubble_astar_->calculatePathCost(path, weight);
      pre_node->weight_[new_odom_node] = weight;
      new_odom_node->weight_[pre_node] = weight;
      pre_node->paths_[new_odom_node] = path;
      reverse(path.begin(), path.end());
      new_odom_node->paths_[pre_node] = path;
      his_odom_dis_vec_.push_back(his_odom_dis_vec_.back() + weight);
    } else {
      his_odom_dis_vec_.push_back(0.0);
    }
    history_odom_nodes_.push_back(new_odom_node);
    path2next_odom_node.push_back(odom_node_->center_);
  }
  if (history_odom_nodes_.empty())
    return;
  auto path = path2next_odom_node;
  parallel_bubble_astar_->collisionCheck_shortenPath(path);
  auto pre_node = history_odom_nodes_.back();
  pre_node->neighbors_.insert(odom_node_);
  odom_node_->neighbors_.insert(pre_node);
  double weight;
  parallel_bubble_astar_->calculatePathCost(path, weight);
  pre_node->weight_[odom_node_] = weight;
  odom_node_->weight_[pre_node] = weight;
  pre_node->paths_[odom_node_] = path;
  reverse(path.begin(), path.end());
  odom_node_->paths_[pre_node] = path;
  // vector<TopoNode::Ptr> hodom2connect;
  PtrPair odomedge2insert;
  for (int i = 0; i < history_odom_nodes_.size() - 1; i++) {
    if ((history_odom_nodes_[i]->center_ - odom_node_->center_).norm() < 5.0)
      odomedge2insert.insert(odom_node_, history_odom_nodes_[i]);
  }
  odomedge2insert.flatten();
  if (odomedge2insert.flatten_data.empty())
    return;
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : odomedge2insert.flatten_data) {
    insertEdge(elem);
  }

  for (auto &elem : odomedge2insert.flatten_data) {
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

float TopoGraph::estimateRoughDistance(const Eigen::Vector3f &goal, const int his_idx) {
  // 找到距离最近的odom-node
  if (his_idx < 0 && !history_odom_nodes_.empty()) {
    return his_odom_dis_vec_.back() + (goal - history_odom_nodes_.front()->center_).norm();
  } else if (his_idx < 0) {
    return (odom_node_->center_ - goal).norm();
  } else {
    return (history_odom_nodes_[his_idx]->center_ - goal).norm() + his_odom_dis_vec_.back() - his_odom_dis_vec_[his_idx];
  }
}