/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-07-12 10:30:16
 * @LastEditTime: 2024-07-12 21:35:39
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <frontier_manager/frontier_manager.h>

class UF {
public:
  UF(int size) {
    father.resize(size);
    rank.resize(size, 0);
    for (int i = 0; i < size; ++i) {
      father[i] = i;
    }
  }

  int find(int x) {
    if (x != father[x]) {
      father[x] = find(father[x]); // Path compression
    }
    return father[x];
  }

  void connect(int x, int y) {
    int xx = find(x), yy = find(y);
    if (xx != yy) {
      if (rank[xx] < rank[yy]) {
        father[xx] = yy;
      } else if (rank[xx] > rank[yy]) {
        father[yy] = xx;
      } else {
        father[yy] = xx;
        rank[xx]++;
      }
    }
  }

private:
  vector<int> father;
  vector<int> rank;
};



void FrontierManager::generateTSPViewpoints(Eigen::Vector3f&center,  vector<TopoNode::Ptr> &viewpoints) {

  unordered_set<ClusterInfo::Ptr> revp_clusters_set; // (re)-generate viewpoints clusters
  vector<float> distance_odom2cluster;
  vector<ClusterInfo::Ptr> old_clusters_within_consideration;
  for (auto &cluster : cluster_list_) {
    if (cluster->is_dormant_ || !cluster->is_reachable_)
      continue;
    if (revp_clusters_set.count(cluster))
      continue;
    old_clusters_within_consideration.push_back(cluster);
    // float distance =
    // (center- cluster->center_).norm() + fabs(graph_->odom_node_->center_.z() - cluster->center_.z()) * 0.5;
    float distance = graph_->estimateRoughDistance(cluster->center_, cluster->odom_id_);
    distance_odom2cluster.push_back(distance);
  }

  vector<int> idx;
  for (int i = 0; i < distance_odom2cluster.size(); i++) {
    idx.push_back(i);
  }

  sort(idx.begin(), idx.end(), [&](int a, int b) { return distance_odom2cluster[a] < distance_odom2cluster[b]; });

  int consider_range = min(vpp_.local_tsp_size_, (int)idx.size());
  // cout << "old_clusters_within_consideration num: " << consider_range << endl;
  for (int i = 0; i < consider_range; i++) {
    old_clusters_within_consideration[idx[i]]->is_new_cluster_ = false;
    revp_clusters_set.insert(old_clusters_within_consideration[idx[i]]);
  }
  // 附近的+新生成的
  vector<ClusterInfo::Ptr> revp_clusters_vec; // revp: regenerate viewpoint
  revp_clusters_vec.insert(revp_clusters_vec.end(), revp_clusters_set.begin(), revp_clusters_set.end());
  ros::Time t1 = ros::Time::now();
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &cluster : revp_clusters_vec) {
    initClusterViewpoints(cluster);
  }
  ros::Time t2 = ros::Time::now();
  // cout << "init cluster viewpoint cost: " << (t2 - t1).toSec() * 1000 << "ms" << endl;

  PointVector vp_centers;
  for (auto &cls : revp_clusters_vec) {
    for (auto &vpc : cls->vp_clusters_) {
      vp_centers.emplace_back(vpc.center_.x(), vpc.center_.y(), vpc.center_.z());
    }
  }
  viz_point(vp_centers, "viewpoint_centers");

  removeUnreachableViewpoints(revp_clusters_vec);
  vector<ClusterInfo::Ptr> clusters_can_be_searched_;
  for (auto &cluster : revp_clusters_vec) {
    if (cluster->is_reachable_)
      clusters_can_be_searched_.push_back(cluster);
  }

  ros::Time t3 = ros::Time::now();
  // cout << "remove unreachable cluster cost: " << (t3 - t2).toSec() * 1000 << "ms" << endl;
  // cout << "revp cluster size: " << revp_clusters_vec.size() << endl;
  // cout << "reab cluster size: " << clusters_can_be_searched_.size() << endl;
  // updateHalfSpaces(clusters_can_be_searched_);
  vector<ClusterInfo::Ptr> tsp_clusters;
  mutex mtx;
  unordered_set<int> cluster2remove;
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < clusters_can_be_searched_.size(); i++) {
    auto cluster = clusters_can_be_searched_[i];
    selectBestViewpoint(cluster);
    if (!cluster->is_reachable_)
      continue;
    mtx.lock();
    tsp_clusters.push_back(cluster);
    if (cluster->is_dormant_) {
      cluster2remove.insert(i);
    }
    mtx.unlock();
  }
  // 飞到但看不到，说明odom漂了，这篇工作不处理，直接跳过
  cluster_list_.remove_if([&](ClusterInfo::Ptr cluster) {
    bool remove = cluster2remove.count(cluster->id_);
    if (remove) {
      for (auto &cell : cluster->cells_) {
        Eigen::Vector3i idx;
        pos2idx(cell, idx);
        ByteArrayRaw bytes;
        idx2bytes(idx, bytes);
        frtd_.label_map_[bytes] = DENSE;
      }
    }
    return remove;
  });
  ros::Time t4 = ros::Time::now();
  // cout << "select best viewpoint cost: " << (t4 - t3).toSec() * 1000 << "ms" << endl;
  // 重新topK
  vector<float> distance2odom2;
  vector<int> idx2;
  for (int i = 0; i < tsp_clusters.size(); i++) {
    float distance = tsp_clusters[i]->distance_;
    distance2odom2.push_back(distance);
    idx2.push_back(i);
  }
  sort(idx2.begin(), idx2.end(), [&](int a, int b) { return distance2odom2[a] < distance2odom2[b]; });
  float mean_distance = accumulate(distance2odom2.begin(), distance2odom2.end(), 0.0) / distance2odom2.size();
  viewpoints.clear();
  for (int i = 0; i < (int)idx2.size(); i++) {
    // 剔除异常值
    // if (i > (int)(idx2.size() / 2.0) && distance2odom2[idx2[i]] > mean_distance * 5.0)
    //   break;
    TopoNode::Ptr vp_node = make_shared<TopoNode>();
    vp_node->is_viewpoint_ = true;
    vp_node->center_ = tsp_clusters[idx2[i]]->best_vp_;
    vp_node->yaw_ = tsp_clusters[idx2[i]]->best_vp_yaw_;
    viewpoints.push_back(vp_node);
  }
  ROS_INFO("vp cluster cost: %fms  ,remove unreachable cost: %fms, select vp cost: %fms", (t2 - t1).toSec() * 1000, (t3 - t2).toSec() * 1000,
           (t4 - t3).toSec() * 1000);
}