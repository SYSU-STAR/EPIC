/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-03-01 15:10:53
 * @LastEditTime: 2024-03-01 17:18:07
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include "pointcloud_topo/parallel_bubble_astar.h"

void ParallelBubbleAstar::init(ros::NodeHandle &nh, const LIOInterface::Ptr &lidar_map) {
  nh.param("bubble_astar/resolution_astar", resolution_, 0.1);
  nh.param("bubble_astar/lambda_heu", lambda_heu_, 1.0);
  nh.param("bubble_astar/allocate_num", allocate_num_, -1);
  nh.param("bubble_astar/safe_distance", safe_distance_, -1.0);
  nh.param("bubble_astar/debug", debug_, false);
  tie_breaker_ = 1.0 + 1.0 / 1000;
  this->lidar_map_interface_ = lidar_map;
  origin_ = lidar_map->lp_->global_map_min_boundary_;
  this->inv_resolution_ = 1.0 / resolution_;
  // frontier_manager_ = frontier_manager_;
}

bool ParallelBubbleAstar::isNodeSafe(Node::Ptr node, const Eigen::Vector3f &bbox_min, const Eigen::Vector3f &bbox_max,
                                     unordered_set<Eigen::Vector3i, v3i_hash> &safe_set, unordered_set<Eigen::Vector3i, v3i_hash> &danger_set) {

  if (!lidar_map_interface_->IsInMap(node->position))
    return false;

  if (node->position.x() < bbox_min.x() || node->position.y() < bbox_min.y() || node->position.z() < bbox_min.z() ||
      node->position.x() > bbox_max.x() || node->position.y() > bbox_max.y() || node->position.z() > bbox_max.z())
    return false;
  Eigen::Vector3i idx;
  posToIndex(node->position, idx);
  if (safe_set.count(idx))
    return true;
  if (danger_set.count(idx))
    return false;

  double radius = lidar_map_interface_->getDisToOcc(node->position) - safe_distance_ - 1e-1;

  if (radius > 0) {
    int width = (int)((radius * 0.57737) * inv_resolution_) - 1;
    width = min(0, width);
    for (int i = -width; i <= width; i++) {
      for (int j = -width; j <= width; j++) {
        for (int k = -width; k <= width; k++) {
          Eigen::Vector3i idx_tmp = idx + Eigen::Vector3i(i, j, k);
          safe_set.insert(idx_tmp);
        }
      }
    }
    return true;
  }
  danger_set.insert(idx);
  return false;
}

int ParallelBubbleAstar::search(const Eigen::Vector3f &start, const Eigen::Vector3f &goal, vector<Eigen::Vector3f> &path, double timeout,
                                bool best_result, bool only_raycast, const Eigen::Vector3f &bbox_min, const Eigen::Vector3f &bbox_max) {
  // step 1 one-shot
  vector<Eigen::Vector3f>().swap(path);
  double goal_r;
  goal_r = min(1.5, lidar_map_interface_->getDisToOcc(goal) - safe_distance_ - 1e-3);
  if (goal_r <= 1e-2) {
    return ParallelBubbleAstar::NO_PATH;
  }
  double len = (goal - start).norm();
  Eigen::Vector3f dir = (goal - start).normalized();
  Eigen::Vector3f curr_pos = start;
  vector<Eigen::Vector3f> path_tmp;
  path_tmp.push_back(curr_pos);
  while (true) {
    double step;
    step = min(1.5, lidar_map_interface_->getDisToOcc(curr_pos) - safe_distance_ - 1e-3);
    if (step <= 1e-2)
      break;
    if (step + goal_r > (goal - curr_pos).norm()) {
      path.swap(path_tmp);
      path.push_back((goal + curr_pos) / 2.0);
      path.push_back(goal);
      // ROS_INFO("one shot! reach goal");
      return ParallelBubbleAstar::REACH_END;
    } else {
      curr_pos += step * dir;
      path_tmp.push_back(curr_pos);
    }
    len -= step;
    if (len < 0) {
      ROS_ERROR("one shot has bug");
      exit(0);
    }
  }
  if (only_raycast) {
    return ParallelBubbleAstar::NO_PATH;
  }
  // ROS_INFO("one shot fail");

  auto getHeu = [&](const Eigen::Vector3f &x1, const Eigen::Vector3f &x2) -> double {
    if (best_result) {
      return tie_breaker_ * (x1 - x2).norm() * 0.9;
    }
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    return tie_breaker_ * (dx + dy + dz);
  };
  auto arriveGoal = [&](Eigen::Vector3f &curr_pos, double &curr_radius) -> bool {
    double curr_r = curr_radius;
    double dis2goal = (curr_pos - goal).norm();
    return (curr_r + goal_r > dis2goal);
  };

  std::priority_queue<Node::Ptr, std::vector<Node::Ptr>, NodeCompre> open_set;
  std::unordered_map<Eigen::Vector3i, Node::Ptr, v3i_hash> open_set_map;
  std::unordered_set<Eigen::Vector3i, v3i_hash> close_set, safe_set, danger_set;
  safe_set.reserve(1000);
  danger_set.reserve(1000);
  close_set.reserve(1000);
  open_set_map.reserve(1000);
  Node::Ptr curr_node = std::make_shared<Node>();
  Node::Ptr end_node = std::make_shared<Node>();
  curr_node->parent = nullptr;
  curr_node->position = start;
  end_node->position = goal;
  if (!isNodeSafe(curr_node, bbox_min, bbox_max, safe_set, danger_set)) {
    // ROS_ERROR("ParallelBubbleAstar: unsafe start point");
    return ParallelBubbleAstar::START_FAIL;
  }
  if (!isNodeSafe(end_node, bbox_min, bbox_max, safe_set, danger_set)) {
    // ROS_ERROR("ParallelBubbleAstar: unsafe end point");
    return ParallelBubbleAstar::END_FAIL;
  }
  curr_node->g_score = 0.0;
  if (!best_result)
    curr_node->f_score = lambda_heu_ * getHeu(start, goal);
  else
    curr_node->f_score = getHeu(start, goal);

  open_set.push(curr_node);
  Eigen::Vector3i curr_idx;
  posToIndex(curr_node->position, curr_idx);
  open_set_map.insert(make_pair(curr_idx, curr_node));

  auto backtrack = [&](Node::Ptr curr_node) {
    while (curr_node->parent != nullptr) {
      path.push_back(curr_node->position);
      curr_node = curr_node->parent;
    }
    path.push_back(curr_node->position);
    std::reverse(path.begin(), path.end());
    path.push_back(goal);
  };

  ros::Time t1 = ros::Time::now();

  while (!open_set.empty()) {
    auto curr_node = open_set.top();
    double curr_radius = lidar_map_interface_->getDisToOcc(curr_node->position) - safe_distance_;
    if (arriveGoal(curr_node->position, curr_radius)) {
      backtrack(curr_node);
      bool safe = collisionCheck_shortenPath(path);
      if (!safe) {
        // ROS_ERROR("path just found not safe");
        return ParallelBubbleAstar::NO_PATH;
        // exit(1);
      }
      // ROS_INFO_STREAM("ParallelBubbleAstar: time cost = " << (ros::Time::now() - t1).toSec() *
      // 1000 << "ms");
      return ParallelBubbleAstar::REACH_END;
    }
    if ((ros::Time::now() - t1).toSec() > timeout) {
      // ROS_ERROR("ParallelBubbleAstar: TIME_OUT");
      return ParallelBubbleAstar::TIME_OUT;
    }
    open_set.pop();
    posToIndex(curr_node->position, curr_idx);
    open_set_map.erase(curr_idx);
    close_set.insert(curr_idx);
    vector<Eigen::Vector3f> step_lis{
    Eigen::Vector3f(resolution_, 0, 0),  Eigen::Vector3f(-resolution_, 0, 0), Eigen::Vector3f(0, resolution_, 0),
    Eigen::Vector3f(0, -resolution_, 0), Eigen::Vector3f(0, 0, resolution_),  Eigen::Vector3f(0, 0, -resolution_),
    };
    Eigen::Vector3f nbr_pos;
    for (auto &step : step_lis) 
    // for(int x=-1;x<=1;x++)
    // for(int y=-1;y<=1;y++)
    // for(int z=-1;z<=1;z++)
    {
      // if(x==0&&y==0&&z==0)
      //   continue;
      // Eigen::Vector3f step = Eigen::Vector3f(x*resolution_, y*resolution_, z*resolution_);
      nbr_pos = curr_node->position + step;
      Eigen::Vector3i nbr_idx;
      posToIndex(nbr_pos, nbr_idx);
      IndexToPos(nbr_pos, nbr_idx);

      if (close_set.find(nbr_idx) != close_set.end())
        continue;
      auto nei_node = std::make_shared<Node>();
      nei_node->position = nbr_pos;

      if (!isNodeSafe(nei_node, bbox_min, bbox_max, safe_set, danger_set))
        continue;
      nei_node->parent = curr_node;
      double tmp_g_score = step.norm() + curr_node->g_score;
      auto node_iter = open_set_map.find(nbr_idx);
      if (node_iter == open_set_map.end()) {
        nei_node->g_score = tmp_g_score;
        if (!best_result)
          nei_node->f_score = lambda_heu_ * getHeu(nbr_pos, goal) + tmp_g_score;
        else
          nei_node->f_score = getHeu(nbr_pos, goal) + tmp_g_score;

        open_set_map.insert(make_pair(nbr_idx, nei_node));
        open_set.push(nei_node);
      } else if (tmp_g_score < node_iter->second->g_score) {
        nei_node = node_iter->second;
      } else {
        continue;
      }
    }
  }
  // ROS_INFO("all space searched");
  return ParallelBubbleAstar::NO_PATH;
}

void ParallelBubbleAstar::reset() {
  unordered_set<Eigen::Vector3i, v3i_hash>().swap(safe_node);
  unordered_set<Eigen::Vector3i, v3i_hash>().swap(dangerous_node);
}

void ParallelBubbleAstar::posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

void ParallelBubbleAstar::IndexToPos(Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  pt = ((idx.cast<float>() + Eigen::Vector3f(0.5, 0.5, 0.5)) * resolution_ + origin_);
}

bool ParallelBubbleAstar::collisionCheck_shortenPath(vector<Eigen::Vector3f> &path) {
  // if (path.size() < 2)
  //   return false;
  // vector<Eigen::Vector3f> path_shorten;
  // std::vector<double> raduis_lis;
  // for (int i = 0; i < path.size(); i++) {
  //   double dis = lidar_map_interface_->getDisToOcc(path[i]) - safe_distance_;
  //   raduis_lis.push_back(dis);
  //   if (raduis_lis.back() < 1e-3) {
  //     return false;
  //   }
  // }
  // vector<int> path_shorten_idx;
  // path_shorten_idx.push_back(0);
  // for (int i = 1; i < path.size() - 1; i++) {
  //   Eigen::Vector3f back_node = path[path_shorten_idx.back()];
  //   Eigen::Vector3f cur = path[i];
  //   Eigen::Vector3f next = path[i + 1];
  //   bool cur_connect = (cur - back_node).norm() < raduis_lis[i] + raduis_lis[path_shorten_idx.back()];
  //   bool next_connect = ((next - back_node).norm() < raduis_lis[i + 1] + raduis_lis[path_shorten_idx.back()]);
  //   if (cur_connect && next_connect)
  //     continue;
  //   else if (cur_connect && !next_connect) {
  //     path_shorten_idx.push_back(i);
  //   } else if (!cur_connect) {
  //     return false;
  //   }
  // }
  // path_shorten_idx.push_back(path.size() - 1);

  // for (int i = 0; i < path_shorten_idx.size() ; i++) {
  //   path_shorten.push_back(path[path_shorten_idx[i]]);
  // }
  // path.swap(path_shorten);
  // return true;
  if (path.size() < 2)
    return false;
  std::vector<Eigen::Vector3f> path_shorten;
  std::vector<double> raduis_lis;
  for (int i = 0; i < path.size(); i++) {
    double dis = min(2.0, lidar_map_interface_->getDisToOcc(path[i]) - safe_distance_);
    raduis_lis.push_back(dis);
    if (raduis_lis.back() < 1e-3) {
      return false;
    }
  }

  std::deque<int> indices;
  indices.push_front(path.size() - 1); // goal
  for (int i = path.size() - 1; i > 0; i--) {
    double i_raduis = raduis_lis[i];
    bool connected = false;
    for (int j = 0; j < i; j++) {
      double j_raduis = raduis_lis[j];
      double distance = (path[i] - path[j]).norm();
      if (i_raduis + j_raduis + 1e-3 > distance) {
        indices.push_front(j);
        connected = true;
        i = j + 1;
        break;
      }
    }
    if (!connected) {
      return false;
    }
  }
  for (auto &idx : indices) {
    path_shorten.emplace_back(path[idx]);
  }
  path.swap(path_shorten);
  return true;
}

void ParallelBubbleAstar::calculatePathCost(const vector<Eigen::Vector3f> &path, double &cost) {
  cost = 0;
  for (int i = 0; i < path.size() - 1; i++) {
    cost += (path[i + 1] - path[i]).norm();
  }
}
