/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-01 22:38:45
 * @LastEditTime: 2023-12-02 20:52:03
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */
#include <path_searching/bubble_astar.h>
#include <pcl/registration/distances.h>
using namespace std;
using namespace Eigen;

namespace fast_planner {

BubbleAstar::BubbleAstar() {}

BubbleAstar::~BubbleAstar() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i]->safe_bubble;
    delete path_node_pool_[i];
  }
}

void Bubble::init(double radius2_, PointType center_) {
  radius2 = radius2_; // 半径的平方，算根号速度慢，所以直接用平方
  position = center_;
}

void BubbleAstar::init(ros::NodeHandle &nh,
                       const LIOInterface::Ptr &lidar_map) {
  nh.param("bubble_astar/resolution_astar", resolution_, 0.1);
  nh.param("bubble_astar/lambda_heu", lambda_heu_, 1.0);
  nh.param("bubble_astar/allocate_num", allocate_num_, -1);
  nh.param("bubble_astar/safe_distance", safe_distance_, -1.0);
  nh.param("bubble_astar/debug", debug_, false);
  vizer.init(nh);
  safeArea_.reserve(100000);
  tie_breaker_ = 1.0 + 1.0 / 1000;
  this->lidar_map_interface_ = lidar_map;
  origin_ = lidar_map->lp_->global_map_min_boundary_;
  this->inv_resolution_ = 1.0 / resolution_;
  bubble_node_pool_.resize(allocate_num_);
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new GridNode;
    bubble_node_pool_[i] = new Bubble;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
  bubble_used_ = 0;
}

void BubbleAstar::reset(double resolution) {
  ros::Time start = ros::Time::now();
  resolution_ = resolution;
  inv_resolution_ = 1.0 / resolution_;
  open_set_map_.clear();
  close_set_map_.clear();
  path_nodes_.clear();
  bubble_nodes_.clear();
  safeArea_.clear();
  // deadArea_.clear();
  bubble_cnt_ = 0;
  bubble_cnt_2 = 0;
  safe_check_cost_ = 0.0;
  close_set_visit_cnt_ = 0;
  close_set_work_cnt_ = 0;
  loop_cost_ = 0.0;
  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>,
                      BubbleNodeComparator0>()
      .swap(open_set_);
  use_node_num_ = 0;
  iter_num_ = 0;
  bubble_used_ = 0;
  ros::Time end = ros::Time::now();
}

bool BubbleAstar::generateBubble(GridNodePtr &node, bool is_start) {
  node->safe_bubble = bubble_node_pool_[bubble_used_];

  if (bubble_used_ == allocate_num_ - 1) {
    cout << "\033[32m run out of bubble node pool." << endl;
    return false;
  }
  PointVector Nearest_Points;
  vector<float> point_dis;
  // IndexToPos(node->position, node->index);
  this->lidar_map_interface_->KNN(node->position, 1, Nearest_Points, point_dis);
  ros::Time end_time = ros::Time::now();

  if (point_dis.size() <= 0) {
    node->safe_bubble->init(pow(4.0 - safe_distance_ - 0.1, 2), node->position);
    safeArea_.emplace_back(node->safe_bubble);
    if (debug_) {
      vizer.visualizeSingleBubble(node->safe_bubble, {1, 0, 0}, false);
      cout << "press Enter!" << endl;
      getchar();
    }
    bubble_used_ += 1;

    return true;
  }
  point_dis[0] = sqrt(point_dis[0]);
  if (point_dis[0] > safe_distance_ + 0.1) {
    node->safe_bubble->init(pow(point_dis[0] - safe_distance_, 2),
                            node->position);
    safeArea_.emplace_back(node->safe_bubble);
    if (debug_) {
      vizer.visualizeSingleBubble(node->safe_bubble, {1, 0, 0}, false);
      cout << "press Enter!" << endl;
      getchar();
    }
    bubble_used_ += 1;
    return true;
  } else if (is_start && point_dis[0] > safe_distance_) {
    node->safe_bubble->init(pow(point_dis[0], 2), node->position);
    // ROS_INFO("BubbleAstar: start_bubble refine");
    // ROS_INFO("TODO change vp position");
    // TODO: viewpoint 调整
    safeArea_.emplace_back(node->safe_bubble);
    bubble_used_ += 1;
    return true;
  } else {
    // node->safe_bubble->init(pow(safe_distance_ + 0.3 - point_dis[0], 2),
    //                         node->position); // 危险区域
    // deadArea_.emplace_back(node->safe_bubble);
    // bubble_used_ += 1;

    return false;
  }
}

double BubbleAstar::pathLength(const vector<Eigen::Vector3f> &path) {
  double length = 0.0;
  if (path.size() < 2)
    return length;
  for (int i = 0; i < path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

bool BubbleAstar::safeCheck(GridNodePtr node) {

  // if (!lidar_map_interface_->IsInMap(node->position.x, node->position.y,
  // node->position.z))) {
  //   return false;
  // }
  if (dead_set_map_.find(node->index) != dead_set_map_.end()) {
    return false;
  }
  float r_now2 = pcl::squaredEuclideanDistance(
      node->position, node->parent->safe_bubble->position);
  if (r_now2 < node->parent->safe_bubble->radius2) {
    // 在爹球里, 遗产继承
    node->safe_bubble = node->parent->safe_bubble;

    return true;
  }
  // 检查是否存在安全球
  for (const BubblePtr &safe_bubble_ : safeArea_) {
    r_now2 =
        pcl::squaredEuclideanDistance(node->position, safe_bubble_->position);
    if (r_now2 < safe_bubble_->radius2) {
      node->safe_bubble = safe_bubble_;
      return true;
    }
  }
  // 检查是否在死亡区域
  // for (const BubblePtr& dead_bubble_ : deadArea_) {
  //   r_now2 = pcl::squaredEuclideanDistance(node->position,
  //   dead_bubble_->position); if (r_now2 < dead_bubble_->radius2) {
  //     return false;
  //   }
  // }
  ros::Time start_time = ros::Time::now();
  bool generate_bubble_success = generateBubble(node);
  double dur = (ros::Time::now() - start_time).toSec() * 1000;
  bubble_avg_cost_ = bubble_avg_cost_ * bubble_cnt_ / (bubble_cnt_ + 1) +
                     dur / (bubble_cnt_ + 1);
  bubble_cnt_ += 1;
  if (!generate_bubble_success) {
    // 生不出球，直接死了
    dead_set_map_.insert(make_pair(node->index, 1));
    close_set_map_.insert(make_pair(node->index, 1));
    return false;
  } else if ((sqrt(node->safe_bubble->radius2) +
              sqrt(node->parent->safe_bubble->radius2)) >
             sqrt(r_now2) + resolution_) {
    node->is_center = true;
    return true;
  } else {
    node->is_center = false;
    return false;
  }
}

int BubbleAstar::search(const Vector3f &start_pt, const Vector3f &end_pt,
                        const double &max_time, bool refine_goal,
                        bool topo_insert) {
  int count = 0;
  GridNodePtr cur_node = path_node_pool_[0];
  GridNodePtr end_node = path_node_pool_[1];
  use_node_num_ += 2;
  Eigen::Vector3f goal_refined = end_pt;
  Eigen::Vector3f start_refined = start_pt;
  // goal_refine(start_refined);

  if (refine_goal) {
    goal_refine(goal_refined);
  }
  // if ((goal_refined - end_pt).norm() > 1e-3) {
  //   cout << "goal refined!" << endl;
  //   cout << goal_refined.transpose() << endl;
  //   cout << end_pt.transpose() << endl;
  // }
  posToIndex(start_refined, cur_node->index);
  posToIndex(goal_refined, end_node->index);

  cur_node->parent = NULL;
  cur_node->position =
      PointType(start_refined.x(), start_refined.y(), start_refined.z());
  end_node->position =
      PointType(goal_refined.x(), goal_refined.y(), goal_refined.z());
  // start 和end的球没必要再这里生成，一会改一改 TODO
  if (!generateBubble(cur_node, true)) {
    if (!topo_insert)
      ROS_WARN("BubbleAstar: unsafe start point");
    return START_FAIL;
  }
  if (!generateBubble(end_node)) {
    if (!topo_insert)
      ROS_WARN("BubbleAstar: unsafe end point");
    return END_FAIL;
  }
  cur_node->g_score = 0.0; // 走过的
  cur_node->f_score =
      lambda_heu_ * getDiagHeu(start_refined, goal_refined); // 启发式
  open_set_.push(cur_node);
  open_set_map_.insert(
      make_pair(cur_node->index, cur_node)); // HashMap rapid search

  const auto t1 = ros::Time::now();
  /* ---------- search loop ---------- */
  // PointVector searched_points;
  // searched_points.reserve(100000);
  int loop_count = 0;
  ros::Time safeCheck_start;
  ros::Time a_star_start = ros::Time::now();

  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    count++;
    // reatch end or not
    float distance2 = pcl::squaredEuclideanDistance(
        cur_node->position, end_node->safe_bubble->position);
    if (distance2 < end_node->safe_bubble->radius2) {
      // 到终点了！
      end_node->parent = cur_node;
      backtrack(end_node, end_pt, !topo_insert);
      double dur = (ros::Time::now() - a_star_start).toSec() * 1000;

      return REACH_END;
    }
    if (!debug_ && (ros::Time::now() - t1).toSec() > max_time) {
      if (!topo_insert)
        cout << "\033[32m time cost > max_search_time_" << endl;
      return NO_PATH;
    }
    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_visit_cnt_++;
    if (close_set_map_.find(cur_node->index) == close_set_map_.end()) {
      close_set_map_.insert(make_pair(cur_node->index, 1));
    } else {
      close_set_work_cnt_++;
    }
    iter_num_ += 1;
    Eigen::Vector3f cur_pos(cur_node->position.x, cur_node->position.y,
                            cur_node->position.z);
    Eigen::Vector3f nbr_pos;
    Eigen::Vector3f step;
    ros::Time for_loop_start = ros::Time::now();

    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3;
           dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3;
             dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < 1e-3)
            continue;
          if (step.norm() > resolution_ * 1.01)
            continue; //
          loop_count++;
          nbr_pos = cur_pos + step;
          if (!lidar_map_interface_->IsInMap(nbr_pos))
            continue;
          GridNodePtr neighbor = path_node_pool_[use_node_num_];
          neighbor->position = PointType(nbr_pos.x(), nbr_pos.y(), nbr_pos.z());
          posToIndex(nbr_pos, neighbor->index);
          // Check not in close set
          close_set_visit_cnt_++;
          if (close_set_map_.find(neighbor->index) != close_set_map_.end()) {
            close_set_work_cnt_++;
            continue;
          }
          neighbor->parent = cur_node;
          // cur_node->sons.emplace_back(neighbor);

          // Check safety
          safeCheck_start = ros::Time::now();
          bool safe = safeCheck(neighbor);
          safe_check_cost_ += (ros::Time::now() - safeCheck_start).toSec();
          if (!safe)
            continue;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(neighbor->index);
          if (node_iter == open_set_map_.end()) {
            // neighbor = path_node_pool_[use_node_num_];
            // neighbor->is_center = false;
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              cout << "run out of node pool." << endl;
              return NO_PATH;
            }
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;
          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score =
              tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, goal_refined);
          open_set_.push(neighbor);
          open_set_map_[neighbor->index] = neighbor;
        }
    loop_cost_ += (ros::Time::now() - for_loop_start).toSec();
  }

  // Vector3f dir = nbr_pos - cur_pos;
  if (!topo_insert) {
    ROS_INFO("all space searched");
  }
  return NO_PATH;
}

void BubbleAstar::backtrack(const GridNodePtr &end_node,
                            const Eigen::Vector3f &end, bool viz) {
  std::vector<GridNodePtr> bubble_path;
  GridNodePtr cur_node = end_node;
  end_node->is_center = true;
  int count = 0;
  int count_ = 0;
  while (cur_node->parent != NULL) {
    bubble_path.emplace_back(cur_node);
    cur_node = cur_node->parent;
  }

  bubble_path.emplace_back(cur_node);
  reverse(bubble_path.begin(), bubble_path.end());

  std::vector<GridNodePtr> bubble_path_shortened;
  bubble_path_shortened.reserve(bubble_path.size());
  for (int i = 0; i < bubble_path.size(); i++) {
    auto const pt = bubble_path[i];
    if (bubble_path_shortened.size() == 0) {
      bubble_path_shortened.emplace_back(pt);
      continue;
    }

    double distance = pcl::euclideanDistance(
        pt->safe_bubble->position,
        bubble_path_shortened[bubble_path_shortened.size() - 1]
            ->safe_bubble->position);
    if (distance < 1e-5) {
      double dis =
          pcl::euclideanDistance(pt->position, pt->safe_bubble->position);
      continue;
    }
    // 杀死中间的电灯泡
    if (bubble_path_shortened.size() > 2) {
      double c = pcl::euclideanDistance(
          pt->safe_bubble->position,
          bubble_path_shortened[bubble_path_shortened.size() - 2]
              ->safe_bubble->position);

      double a = sqrt(pt->safe_bubble->radius2);
      double b = sqrt(bubble_path_shortened[bubble_path_shortened.size() - 2]
                          ->safe_bubble->radius2);
      // double s = (a + b + c) / 2;
      // double area =
      //     std::sqrt(s * (s - a) * (s - b) * (s - c));  //
      //     海伦公式求面积
      // double r_corridor = 2 * area / c;  // 安全走廊半径
      // if (r_corridor > safe_distance_) {
      //     // bubble_path_shortened.pop_back();
      // }
      if (a + b > c)
        bubble_path_shortened.pop_back();
    }
    bubble_path_shortened.emplace_back(pt);
  }
  vizer.visualize(bubble_path_shortened);
  for (auto const nd : bubble_path_shortened) {
    path_nodes_.emplace_back(nd->safe_bubble->position.x,
                             nd->safe_bubble->position.y,
                             nd->safe_bubble->position.z);
  }
}

std::vector<Eigen::Vector3f> BubbleAstar::getPath() { return path_nodes_; }

double BubbleAstar::getDiagHeu(const Eigen::Vector3f &x1,
                               const Eigen::Vector3f &x2) {
  float dx = fabs(x1.x() - x2.x());
  float dy = fabs(x1.y() - x2.y());
  float dz = fabs(x1.z() - x2.z());
  float h;
  float diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double BubbleAstar::getDiagHeu(const Eigen::Vector3f &x1, const PointType &x2) {
  float dx = fabs(x1.x() - x2.x);
  float dy = fabs(x1.y() - x2.y);
  float dz = fabs(x1.z() - x2.z);
  float h;
  float diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double BubbleAstar::getDiagHeu(const PointType &x1, const PointType &x2) {
  float dx = fabs(x1.x - x2.x);
  float dy = fabs(x1.y - x2.y);
  float dz = fabs(x1.z - x2.z);
  float h;
  float diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

void BubbleAstar::posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

void BubbleAstar::posToIndex(const PointType &pt, Eigen::Vector3i &idx) {
  idx = ((Eigen::Vector3f(pt.x, pt.y, pt.z) - origin_) * inv_resolution_)
            .array()
            .floor()
            .cast<int>();
}

void BubbleAstar::goal_refine(Eigen::Vector3f &goal, bool use_map_bd) {
  double safe_radius = 0.6;
  Eigen::Vector3f max_bd, min_bd;
  if (use_map_bd) {
    max_bd = this->lidar_map_interface_->lp_->global_map_max_boundary_;
    min_bd = this->lidar_map_interface_->lp_->global_map_min_boundary_;
  } else {
    max_bd = this->lidar_map_interface_->lp_->global_box_max_boundary_;
    min_bd = this->lidar_map_interface_->lp_->global_box_min_boundary_;
  }
  auto nearest_occ = [&](const Eigen::Vector3f &curr_goal) -> Eigen::Vector3f {
    PointVector np;
    vector<float> dis2;
    PointType p;
    p.x = curr_goal.x();
    p.y = curr_goal.y();
    p.z = curr_goal.z();
    for (int i = 0; i < 3; i++) {
      if (goal[i] - min_bd[i] < 0) {
        goal[i] = min_bd[i] + 1e-3;
      }

      if (max_bd[i] - goal[i] < 0) {
        goal[i] = max_bd[i] - 1e-3;
      }
    }
    this->lidar_map_interface_->KNN(p, 1, np, dis2);

    vector<Eigen::Vector3f> occ;
    if (np.size() != 0)
      occ.emplace_back(np[0].x, np[0].y, np[0].z);

    if (occ.size() == 0)
      return Eigen::Vector3f::Zero();
    int min_dis_idx = 0;
    double min_dis = 1e5;
    for (int i = 0; i < occ.size(); i++) {
      if ((occ[i] - goal).norm() < min_dis) {
        min_dis = (occ[i] - goal).norm();
        min_dis_idx = i;
      }
    }
    return occ[min_dis_idx];
  };

  ros::Time start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 1e-4) {
    Eigen::Vector3f occ_pt = nearest_occ(goal);
    if (occ_pt.norm() < 1e-3)
      return;
    if ((occ_pt - goal).norm() > safe_distance_ + 0.3) {
      return;
    }
    Eigen::Vector3f dir = (goal - occ_pt).normalized();
    double dis = min((occ_pt - goal).norm(), 0.05f);
    goal = goal + dir * dis;
  }
}

void BubbleAstar::IndexToPos(PointType &pt, Eigen::Vector3i &idx) {
  Eigen::Vector3f pt_ =
      (idx.cast<float>().array() * resolution_ + origin_.array());
  pt.x = pt_(0);
  pt.y = pt_(1);
  pt.z = pt_(2);
}

void BubbleVisualizer::init(ros::NodeHandle &nh) {
  nh_ = nh;
  spherePub = nh.advertise<visualization_msgs::MarkerArray>(
      "/bubble_visualizer/sphere", 1000);
  spherePub_debug_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/bubble_visualizer/sphere_debug", 1000);
  wayPointsPub = nh.advertise<visualization_msgs::MarkerArray>(
      "/bubble_visualizer/frontend_traj", 10);
}

void BubbleVisualizer::visualize(
    const std::vector<GridNodePtr> &bubble_nodes_) {
  visualization_msgs::MarkerArray sphereMarkersArray;
  visualization_msgs::MarkerArray sphereMarkersArray_frontend_traj;
  visualization_msgs::Marker sphereMarkers;
  sphereMarkers.action = visualization_msgs::Marker::DELETEALL;
  sphereMarkersArray.markers.push_back(sphereMarkers);
  spherePub.publish(sphereMarkersArray);

  for (int i = 0; i < bubble_nodes_.size(); i++) {
    // visualization_msgs::Marker sphereMarkers;
    // sphereMarkers.action = visualization_msgs::Marker::DELETEALL;
    // sphereMarkersArray.markers.push_back(sphereMarkers);

    sphereMarkers.id = i;
    sphereMarkers.type = visualization_msgs::Marker::SPHERE;
    sphereMarkers.header.stamp = ros::Time::now();
    sphereMarkers.header.frame_id = "world";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action = visualization_msgs::Marker::ADD;
    sphereMarkers.ns = "spheres";

    sphereMarkers.color.g = 1.00;
    sphereMarkers.color.a = 0.2;
    sphereMarkers.scale.x = sqrt(bubble_nodes_[i]->safe_bubble->radius2) * 2.0;
    sphereMarkers.scale.y = sqrt(bubble_nodes_[i]->safe_bubble->radius2) * 2.0;
    sphereMarkers.scale.z = sqrt(bubble_nodes_[i]->safe_bubble->radius2) * 2.0;
    sphereMarkers.pose.position.x = bubble_nodes_[i]->safe_bubble->position.x;
    sphereMarkers.pose.position.y = bubble_nodes_[i]->safe_bubble->position.y;
    sphereMarkers.pose.position.z = bubble_nodes_[i]->safe_bubble->position.z;

    sphereMarkersArray.markers.push_back(sphereMarkers);
  }
  spherePub.publish(sphereMarkersArray);
}

void BubbleVisualizer::visualizeSingleBubble(BubblePtr &bubble, Vector3f color,
                                             bool clear_all) {
  static visualization_msgs::MarkerArray sphereMarkersArray; // 历史bubble，红色
  visualization_msgs::Marker sphereMarkers; // 新产生的bubble，蓝色
  static int id;
  if (clear_all) {
    sphereMarkers.action = visualization_msgs::Marker::DELETEALL;
    sphereMarkersArray.markers.push_back(sphereMarkers);
    spherePub_debug_.publish(sphereMarkersArray);
    sphereMarkersArray.markers.clear();
    id = 0;
  }
  sphereMarkers.action = visualization_msgs::Marker::ADD;
  sphereMarkers.id = id++;
  sphereMarkers.type = visualization_msgs::Marker::SPHERE;
  sphereMarkers.header.stamp = ros::Time::now();
  sphereMarkers.header.frame_id = "world";
  sphereMarkers.pose.orientation.w = 1.00;
  sphereMarkers.ns = "spheres";
  sphereMarkers.color.a = 0.3;
  sphereMarkers.color.r = color(0);
  sphereMarkers.color.g = color(1);
  sphereMarkers.color.b = color(2);
  sphereMarkers.scale.x = sqrt(bubble->radius2) * 2.0;
  sphereMarkers.scale.y = sqrt(bubble->radius2) * 2.0;
  sphereMarkers.scale.z = sqrt(bubble->radius2) * 2.0;

  sphereMarkers.pose.position.x = bubble->position.x;
  sphereMarkers.pose.position.y = bubble->position.y;
  sphereMarkers.pose.position.z = bubble->position.z;
  sphereMarkersArray.markers.push_back(sphereMarkers);
  spherePub_debug_.publish(sphereMarkersArray);
}

void FastSearcher::init(TopoGraph::Ptr topo_graph,
                        BubbleAstar::Ptr bubble_astar_searcher) {
  topo_graph_ = topo_graph;
  bubble_astar_searcher_ = bubble_astar_searcher;
}

TopoNode::Ptr FastSearcher::createTopoNode(const Eigen::Vector3f &pose,
                                           const Eigen::Vector3f &curr_vel,
                                           bool connect_nei_region,
                                           bool nei_must_known,
                                           bool is_odom_node) {
  TopoNode::Ptr node(new TopoNode);
  node->center_ = pose;
  node->is_viewpoint_ = false;
  PointType cen;
  cen.x = pose(0);
  cen.y = pose(1);
  cen.z = pose(2);
  // BubbleNode::Ptr bubble(new
  // BubbleNode(topo_graph_->lidar_map_interface_->getDisToOcc(cen), pose));
  // node->self_bubble_ = bubble;
  Eigen::Vector3i idx;
  topo_graph_->getIndex(pose, idx);
  std::vector<TopoNode::Ptr> pre_neighbors;
  auto region = topo_graph_->getRegionNode(idx);
  if (region == nullptr)
    return nullptr;

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i idx_nei =
            Eigen::Vector3i(idx[0] + i, idx[1] + j, idx[2] + k);
        auto nei_region = topo_graph_->getRegionNode(idx);
        if (nei_region == nullptr)
          continue;
        for (auto &nei_topo : nei_region->topo_nodes_) {
          pre_neighbors.push_back(nei_topo);
        }
      }
    }
  }

  for (auto &nei_node : pre_neighbors) {
    int res;
    vector<Eigen::Vector3f> path;
    res = topo_graph_->parallel_bubble_astar_->search(
        node->center_, nei_node->center_, path, 2e-3, false);
    if (res == BubbleAstar::START_FAIL) {
      if (is_odom_node)
        cout << "create start topo fail" << endl;
      return nullptr;
    } else if (res == BubbleAstar::NO_PATH ||
               res == ParallelBubbleAstar::TIME_OUT) {
      continue;
    } else if (res == BubbleAstar::END_FAIL) {
      if (connect_nei_region)
        cout << "create end topo fail" << endl;
      continue;
    }
    nei_node->neighbors_.insert(node);
    node->neighbors_.insert(nei_node);
    // auto path = bubble_astar_searcher_->getPath();
    node->paths_[nei_node] = path;
    double weight = 0;
    for (int i = 0; i < path.size() - 1; i++) {
      weight += (path[i + 1] - path[i]).norm();
    }
    if (curr_vel.norm() > 1e-3) {
      Eigen::Vector3f dir = (nei_node->center_ - node->center_).normalized();
      Eigen::Vector3f vdir = curr_vel.normalized();
      double diff = acos(vdir.dot(dir));
      weight += diff;
    }
    node->weight_[nei_node] = weight;
    std::reverse(path.begin(), path.end());
    nei_node->paths_[node] = path;
    nei_node->weight_[node] = weight;
  }
  if (node->paths_.size() == 0)
    return nullptr;
  else {
    region->topo_nodes_.insert(node);
    return node;
  }
}

void FastSearcher::removeTopoNode(TopoNode::Ptr node) {
  if (node == nullptr)
    return;
  Eigen::Vector3i idx;
  topo_graph_->getIndex(node->center_, idx);
  auto region = topo_graph_->getRegionNode(idx);
  if (region == nullptr)
    return;
  region->topo_nodes_.erase(node);
  topo_graph_->removeNode(node);
}

int FastSearcher::search(const TopoNode::Ptr &start_node,
                         const Vector3f &curr_vel,
                         const TopoNode::Ptr &end_node, double max_time,
                         std::vector<Eigen::Vector3f> &path) {
  // 用bubble_astar让start和end往外搜，构建拓扑节点

  // 用topo_astar拓扑路径
  // 沿线理出path
  static std::vector<TopoNode::Ptr> topo_path;
  bool success;
  if (!topo_path.empty()) {
    unordered_set<pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash>
        last_path_set;
    for (int i = 0; i < topo_path.size() - 1; i++) {
      last_path_set.insert({topo_path[i], topo_path[i + 1]});
    }
    success = topo_graph_->graphSearch(start_node, end_node, topo_path,
                                       max_time, true, last_path_set);
  } else {
    success = topo_graph_->graphSearch(start_node, end_node, topo_path,
                                       max_time, false);
  }
  // topo_graph_->removeNodes(end_node_vec);

  if (!success) {
    return BubbleAstar::NO_PATH;
  }
  // if (topo_path.size() <= 2) {
  //   path.emplace_back(start_node->center_);
  //   path.emplace_back(end_node->center_);
  //   return BubbleAstar::REACH_END;
  // }
  for (int i = 0; i < topo_path.size() - 1; i++) {
    auto back = topo_path[i];
    auto front = topo_path[i + 1];
    for (auto &pt : back->paths_[front]) {
      path.emplace_back(pt);
    }
  }
  return BubbleAstar::REACH_END;
}

int FastSearcher::topoSearch(const TopoNode::Ptr &start_node,
                             const TopoNode::Ptr &end_node, double max_time,
                             std::vector<Eigen::Vector3f> &path) {
  if (start_node == nullptr || end_node == nullptr) {
    ROS_ERROR("topo search: start or end node is null");
    return start_node == nullptr ? BubbleAstar::START_FAIL
                                 : BubbleAstar::END_FAIL;
  }
  path.clear();
  std::vector<TopoNode::Ptr> topo_path;
  bool success =
      topo_graph_->graphSearch(start_node, end_node, topo_path, max_time);
  if (!success) {
    return BubbleAstar::NO_PATH;
  }
  // if (topo_path.size() <= 2) {
  //   path.emplace_back(start_node->center_);
  //   path.emplace_back(end_node->center_);
  //   return BubbleAstar::REACH_END;
  // }
  for (int i = 0; i < topo_path.size() - 1; i++) {
    auto back = topo_path[i];
    auto front = topo_path[i + 1];
    for (auto &pt : back->paths_[front]) {
      path.emplace_back(pt);
    }
  }
  return BubbleAstar::REACH_END;
}

} // namespace fast_planner