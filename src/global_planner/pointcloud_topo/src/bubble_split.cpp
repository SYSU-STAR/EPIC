/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-02 21:33:08
 * @LastEditTime: 2024-03-05 12:12:19
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include "pointcloud_topo/graph.h"
void TopoGraph::generateBubble(const Eigen::Vector3f &low_bd,
                               const Eigen::Vector3f &high_bd,
                               vector<BubbleNode::Ptr> &bubble_node_vec,
                               vector<bool> &check_flags) {
  ros::Time start = ros::Time::now();
  Eigen::Vector3f center = (low_bd + high_bd) / 2.0;
  Eigen::Vector3f bd_size = high_bd - low_bd;
  if (!lidar_map_interface_->IsInBox(center))
    return;

  if (low_bd[0] >= high_bd[0] || low_bd[1] >= high_bd[1] ||
      low_bd[2] >= high_bd[2])
    return;

  if (bd_size.x() < min_x_ && bd_size.y() < min_y_ && bd_size.z() < min_z_)
    return;
  Eigen::Vector3i region_idx;
  getIndex(center, region_idx);
  if (!reg_map_idx2ptr_.count(region_idx)) {
    ROS_ERROR("148 region idx not in map");
    exit(1);
  }
  Eigen::Vector3f region_bd_min, region_bd_max;
  index2boundary(region_idx, region_bd_min, region_bd_max);
  Eigen::Vector3f box_search_min_pt, box_search_max_pt;
  box_search_min_pt = low_bd - region_bd_min;
  box_search_max_pt = high_bd - region_bd_min;
  std::vector<int> box_indices;
  check_pts_octree_.boxSearch(box_search_min_pt, box_search_max_pt,
                              box_indices);
  bool all_covered = true;
  for (auto &i : box_indices) {
    if (check_flags[i])
      continue;
    all_covered = false;
    break;
  }
  if (all_covered)
    return;
  double dis2occ = lidar_map_interface_->getDisToOcc(center);

  if (dis2occ < bubble_min_radius_) {
    // 小到没有必要生成球，给你换一个位置吧
    splitCubeBubbleGeneration(low_bd, high_bd, bubble_node_vec, check_flags);
    return;
  }
  std::vector<int> radius_indices;
  std::vector<float> radius_distances;
  pcl::PointXYZ searchPoint(center.x(), center.y(), center.z());
  check_pts_octree_.radiusSearch(searchPoint, (float)dis2occ, radius_indices,
                                 radius_distances);
  for (auto &idx : radius_indices) {
    check_flags[idx] = true;
  }
  BubbleNode::Ptr bubble_node = std::make_shared<BubbleNode>(dis2occ, center);
  bubble_node_vec.push_back(bubble_node);

  if (bubble_node->radius_ > (high_bd - low_bd).norm() / 2.0) {
    // 说明已经覆盖了全空间，直接return
    return;
  }
  if (bubble_node->radius_ < bubble_min_radius_) {
    splitCubeBubbleGeneration(low_bd, high_bd, bubble_node_vec, check_flags);
    return;
  }
  supplementCubeBubbleGeneration(low_bd, high_bd, bubble_node_vec, check_flags,
                                 bubble_node);
}

void TopoGraph::splitCubeBubbleGeneration(
    const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd,
    vector<BubbleNode::Ptr> &bubble_node_vec, vector<bool> &check_flags) {
  double bd_size_x = high_bd[0] - low_bd[0];
  double bd_size_y = high_bd[1] - low_bd[1];
  double bd_size_z = high_bd[2] - low_bd[2];
  if (bd_size_x < min_x_ * 2.0 && bd_size_y < min_y_ * 2.0 &&
      bd_size_z < min_z_ * 2.0)
    return;
  double x_step, y_step, z_step;
  x_step = bd_size_x >= min_x_ * 2.0 ? bd_size_x / 2.0 : bd_size_x;
  y_step = bd_size_y >= min_y_ * 2.0 ? bd_size_y / 2.0 : bd_size_y;
  z_step = bd_size_z >= min_z_ * 2.0 ? bd_size_z / 2.0 : bd_size_z;
  for (size_t i = 0; x_step * (i + 1) + low_bd[0] <= high_bd[0]; i++) {
    for (int j = 0; y_step * (j + 1) + low_bd[1] <= high_bd[1]; j++) {
      for (int k = 0; z_step * (k + 1) + low_bd[2] <= high_bd[2]; k++) {
        generateBubble(
            low_bd + Eigen::Vector3f(x_step * i, y_step * j, z_step * k),
            low_bd + Eigen::Vector3f(x_step * (i + 1), y_step * (j + 1),
                                     z_step * (k + 1)),
            bubble_node_vec, check_flags);
      }
    }
  }
}

void TopoGraph::supplementCubeBubbleGeneration(
    const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd,
    vector<BubbleNode::Ptr> &bubble_node_vec, vector<bool> &check_flags,
    const BubbleNode::Ptr &bubble_node) {
  bubble_node_vec.push_back(bubble_node);
  // step2 计算出26个邻居长方体
  Eigen::Vector3f outer_000 = low_bd;
  Eigen::Vector3f outer_001{low_bd.x(), low_bd.y(), high_bd.z()};
  Eigen::Vector3f outer_010{low_bd.x(), high_bd.y(), low_bd.z()};
  Eigen::Vector3f outer_011{low_bd.x(), high_bd.y(), high_bd.z()};
  Eigen::Vector3f outer_100{high_bd.x(), low_bd.y(), low_bd.z()};
  Eigen::Vector3f outer_101{high_bd.x(), low_bd.y(), high_bd.z()};
  Eigen::Vector3f outer_110{high_bd.x(), high_bd.y(), low_bd.z()};
  Eigen::Vector3f outer_111 = high_bd;

  double length = bubble_node->radius_ / sqrt(3);
  Eigen::Vector3f inner_000 =
      bubble_node->center_ + Eigen::Vector3f(-1, -1, -1) * length;
  inner_000.x() = std::max(inner_000.x(), outer_000.x());
  inner_000.y() = std::max(inner_000.y(), outer_000.y());
  inner_000.z() = std::max(inner_000.z(), outer_000.z());
  Eigen::Vector3f inner_001 =
      bubble_node->center_ + Eigen::Vector3f(-1, -1, 1) * length;
  inner_001.x() = std::max(inner_001.x(), outer_001.x());
  inner_001.y() = std::max(inner_001.y(), outer_001.y());
  inner_001.z() = std::min(inner_001.z(), outer_001.z());
  Eigen::Vector3f inner_010 =
      bubble_node->center_ + Eigen::Vector3f(-1, 1, -1) * length;
  inner_010.x() = std::max(inner_010.x(), outer_010.x());
  inner_010.y() = std::min(inner_010.y(), outer_010.y());
  inner_010.z() = std::max(inner_010.z(), outer_010.z());
  Eigen::Vector3f inner_011 =
      bubble_node->center_ + Eigen::Vector3f(-1, 1, 1) * length;
  inner_011.x() = std::max(inner_011.x(), outer_011.x());
  inner_011.y() = std::min(inner_011.y(), outer_011.y());
  inner_011.z() = std::min(inner_011.z(), outer_011.z());
  Eigen::Vector3f inner_100 =
      bubble_node->center_ + Eigen::Vector3f(1, -1, -1) * length;
  inner_100.x() = std::min(inner_100.x(), outer_100.x());
  inner_100.y() = std::max(inner_100.y(), outer_100.y());
  inner_100.z() = std::max(inner_100.z(), outer_100.z());
  Eigen::Vector3f inner_101 =
      bubble_node->center_ + Eigen::Vector3f(1, -1, 1) * length;
  inner_101.x() = std::min(inner_101.x(), outer_101.x());
  inner_101.y() = std::max(inner_101.y(), outer_101.y());
  inner_101.z() = std::min(inner_101.z(), outer_101.z());
  Eigen::Vector3f inner_110 =
      bubble_node->center_ + Eigen::Vector3f(1, 1, -1) * length;
  inner_110.x() = std::min(inner_110.x(), outer_110.x());
  inner_110.y() = std::min(inner_110.y(), outer_110.y());
  inner_110.z() = std::max(inner_110.z(), outer_110.z());
  Eigen::Vector3f inner_111 =
      bubble_node->center_ + Eigen::Vector3f(1, 1, 1) * length;
  inner_111.x() = std::min(inner_111.x(), outer_111.x());
  inner_111.y() = std::min(inner_111.y(), outer_111.y());
  inner_111.z() = std::min(inner_111.z(), outer_111.z());

  // step3 用26个邻居长方体递归生成球
  // 第一层: 9个

  Eigen::Vector3f low_bd_tmp, high_bd_tmp;
  low_bd_tmp << outer_000.x(), outer_000.y(), outer_000.z();
  high_bd_tmp << inner_000.x(), inner_000.y(), inner_000.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_000.x(), outer_000.y(), outer_000.z();
  high_bd_tmp << inner_100.x(), inner_100.y(), inner_100.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_100.x(), outer_100.y(), outer_100.z();
  high_bd_tmp << outer_100.x(), inner_100.y(), inner_100.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << outer_000.x(), inner_000.y(), outer_000.z();
  high_bd_tmp << inner_010.x(), inner_010.y(), inner_010.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_000.x(), inner_000.y(), outer_000.z();
  high_bd_tmp << inner_110.x(), inner_110.y(), inner_110.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_100.x(), inner_100.y(), outer_100.z();
  high_bd_tmp << outer_110.x(), inner_110.y(), inner_110.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << outer_010.x(), inner_010.y(), outer_010.z();
  high_bd_tmp << inner_010.x(), outer_010.y(), inner_010.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_010.x(), inner_010.y(), outer_010.z();
  high_bd_tmp << inner_110.x(), outer_110.y(), inner_110.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_110.x(), inner_110.y(), outer_110.z();
  high_bd_tmp << outer_110.x(), outer_110.y(), inner_110.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  // 第二层: 8个

  low_bd_tmp << outer_000.x(), outer_000.y(), inner_000.z();
  high_bd_tmp << inner_000.x(), inner_000.y(), inner_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_000.x(), outer_000.y(), inner_000.z();
  high_bd_tmp << inner_100.x(), inner_100.y(), inner_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_100.x(), outer_100.y(), inner_000.z();
  high_bd_tmp << outer_100.x(), inner_100.y(), inner_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << outer_000.x(), inner_000.y(), inner_000.z();
  high_bd_tmp << inner_010.x(), inner_010.y(), inner_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  // low_bd_tmp << inner_000.x(), inner_000.y(), outer_000.z();
  // high_bd_tmp << inner_110.x(), inner_110.y(), inner_110.z();
  // generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec);
  low_bd_tmp << inner_100.x(), inner_100.y(), inner_000.z();
  high_bd_tmp << outer_110.x(), inner_110.y(), inner_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << outer_010.x(), inner_010.y(), inner_000.z();
  high_bd_tmp << inner_010.x(), outer_010.y(), inner_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_010.x(), inner_010.y(), inner_000.z();
  high_bd_tmp << inner_110.x(), outer_110.y(), inner_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_110.x(), inner_110.y(), inner_000.z();
  high_bd_tmp << outer_110.x(), outer_110.y(), inner_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);

  // 第三层:9个
  low_bd_tmp << outer_000.x(), outer_000.y(), inner_001.z();
  high_bd_tmp << inner_000.x(), inner_000.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_000.x(), outer_000.y(), inner_001.z();
  high_bd_tmp << inner_100.x(), inner_100.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_100.x(), outer_100.y(), inner_001.z();
  high_bd_tmp << outer_100.x(), inner_100.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << outer_000.x(), inner_000.y(), inner_001.z();
  high_bd_tmp << inner_010.x(), inner_010.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_000.x(), inner_000.y(), inner_001.z();
  high_bd_tmp << inner_110.x(), inner_110.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_100.x(), inner_100.y(), inner_001.z();
  high_bd_tmp << outer_110.x(), inner_110.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << outer_010.x(), inner_010.y(), inner_001.z();
  high_bd_tmp << inner_010.x(), outer_010.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_010.x(), inner_010.y(), inner_001.z();
  high_bd_tmp << inner_110.x(), outer_110.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
  low_bd_tmp << inner_110.x(), inner_110.y(), inner_001.z();
  high_bd_tmp << outer_110.x(), outer_110.y(), outer_001.z();
  generateBubble(low_bd_tmp, high_bd_tmp, bubble_node_vec, check_flags);
}