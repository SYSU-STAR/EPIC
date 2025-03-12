/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-07-12 12:22:06
 * @LastEditTime: 2024-08-05 20:08:38
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#pragma once
#include "lidar_map/ikd_Tree.h"
#include <bits/stdc++.h>
#include <frontier_manager/raycast.h>
#include <lidar_map/lidar_map.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_topo/graph.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
using namespace fast_planner;

enum CELL_STATE { DENSE, SPARSE, UNKNOWN, FRONTIER_DIS, FRONTIER_DIR };

struct Vector3i_Hash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct FrontierParam {
  float cluster_radius_;
  float cluster_min_radius_;
  float cluster_direction_radius_;
  int cluster_minmum_point_num_;
  float cell_size_;
  float inv_cell_size_;
  float occ_min_dis_;
  float good_observation_direction_score_;
  float good_observation_trust_length_;
  float update_length_;
  float good_observation_force_trust_length_;
  int dense_cell_cloud_num_;
  int sparse_cell_cloud_num_;
  int noise_cell_range_;
  float cluster_min_size_;
  Eigen::Vector3f map_min_;
  Eigen::Vector3f map_max_;
  Eigen::Vector3i cell_max_cnt_;
  Eigen::Vector3i bits_need_;
  uint8_t idx_byte_size_;
  bool view_cluster_, view_frt_;
};

struct ViewpointParam {
  int sample_pillar_circle_sample_num_, sample_pillar_radius_layer_num_,
      sample_pillar_height_layer_num_;
  float sample_pillar_min_height_, sample_pillar_max_height_,
      sample_pillar_min_radius_, sample_pillar_max_radius_,
      view_direction_range_;
  float fov_up_, fov_down_, lidar_pitch_;
  int consider_range_, global_recluster_size_, local_tsp_size_;
};
struct ByteArrayRaw {
  uint8_t *data;
  static size_t size;

  ByteArrayRaw() { data = new uint8_t[size]; }

  ByteArrayRaw(const ByteArrayRaw &other) {
    data = new uint8_t[size];
    memcpy(data, other.data, size);
  }

  ByteArrayRaw &operator=(const ByteArrayRaw &other) {
    if (this != &other) {
      memcpy(data, other.data, size);
    }
    return *this;
  }

  ~ByteArrayRaw() { delete[] data; }

  bool operator==(const ByteArrayRaw &other) const {
    return memcmp(data, other.data, size) == 0;
  }
};

struct ByteArrayRawHasher {
  size_t operator()(const ByteArrayRaw &arr) const {
    const uint64_t seed = 0xc3a5c85c97cb3127ULL;
    const uint64_t m = 0xc6a4a7935bd1e995ULL;
    const int r = 47;

    size_t h = seed ^ (ByteArrayRaw::size * m);

    // 处理8字节对齐的数据
    const uint64_t *data = reinterpret_cast<const uint64_t *>(arr.data);
    const uint64_t *end = data + (ByteArrayRaw::size / 8);

    while (data != end) {
      uint64_t k = *data++;

      k *= m;
      k ^= k >> r;
      k *= m;

      h ^= k;
      h *= m;
    }

    // 处理剩余字节
    const unsigned char *tail = reinterpret_cast<const unsigned char *>(data);

    switch (ByteArrayRaw::size & 7) {
    case 7:
      h ^= uint64_t(tail[6]) << 48;
    case 6:
      h ^= uint64_t(tail[5]) << 40;
    case 5:
      h ^= uint64_t(tail[4]) << 32;
    case 4:
      h ^= uint64_t(tail[3]) << 24;
    case 3:
      h ^= uint64_t(tail[2]) << 16;
    case 2:
      h ^= uint64_t(tail[1]) << 8;
    case 1:
      h ^= uint64_t(tail[0]);
      h *= m;
    }

    h ^= h >> r;
    h *= m;
    h ^= h >> r;

    return h;
  }
};
struct FrontierData {
  FrontierData() { ByteArrayRaw::size = 8; }
  FrontierData(size_t idx_bytes_size) {
    ByteArrayRaw::size = idx_bytes_size; // 设置全局size
  }
  unordered_map<ByteArrayRaw, uint8_t, ByteArrayRawHasher> label_map_;
  unordered_map<ByteArrayRaw, Eigen::Vector3f, ByteArrayRawHasher> frt_map_;
  vector<bool> is_gap_;
  vector<float> direction_score_;
  vector<bool> is_fov_edge_;
  vector<float> sphere_distance_;
  Eigen::Vector3f updating_aabb_min;
  Eigen::Vector3f updating_aabb_max;
};

struct ViewpointCluster {
  ViewpointCluster() {
    center_ = Eigen::Vector3f::Zero();
    vps_.clear();
  }

  Eigen::Vector3f center_;
  PointVector vps_;
  float distance_;
};

struct ClusterInfo {
  typedef shared_ptr<ClusterInfo> Ptr;
  // index of the frt
  PointVector cells_;
  vector<Eigen::Vector3f> norms_;
  Eigen::Vector3f center_;
  Eigen::Vector3f normal_;
  Eigen::Vector4f view_halfspace_;

  int id_;
  int odom_id_;
  vector<ViewpointCluster> vp_clusters_;
  Eigen::Vector3f best_vp_;
  float best_vp_yaw_;
  float distance_;

  bool is_dormant_;
  bool is_reachable_;
  bool is_new_cluster_;

  // bbox
  Eigen::Vector3f box_max_;
  Eigen::Vector3f box_min_;
};

struct SuperClusterInfo {
  typedef shared_ptr<SuperClusterInfo> Ptr;
  vector<ClusterInfo::Ptr> sons;
  Eigen::Vector3f waypoint;
};

class FrontierManager {
private:
  FrontierParam frtp_;
  FrontierData frtd_;
  ViewpointParam vpp_;
  unordered_set<int> force_recluster_;
  LIOInterface::Ptr
      lidar_map_interface_; // TODO: 改成
                            // sim_lio,去掉不必要的接口,ikd-treeh和odom存储在里面
  TopoGraph::Ptr graph_;
  ros::NodeHandle nh_;
  vector<Eigen::Vector3f> origin_viewpoints_;
  // frontier update functions:
  void update_lidar_pos();
  void update_updating_aabb(const PointVector &new_frt_pts);
  void cluster_frts(const PointVector &frt_new,
                    vector<ClusterInfo::Ptr> &new_clusters,
                    vector<int> &cluster_removed);
  void compute_cluster_info(const PointVector &frt_pts,
                            const vector<Eigen::Vector3f> &frt_norms,
                            ClusterInfo::Ptr cluster);
  bool has_overlap(const Eigen::Vector3f &box_max_,
                   const Eigen::Vector3f &box_min_);
  void project_pts_2_depth_image(PointVector &pts_vec,
                                 vector<float> &depth_img);
  int surface_pos2idx(const PointType &pt); // 2w个点
  Eigen::Vector3f map_min_bd_, map_max_bd_;
  void pos2idx(const PointType &pt, Eigen::Vector3i &idx);
  void pos2idx(const Eigen::Vector3f &pt, Eigen::Vector3i &idx);
  void pos2bytes(const PointType &pt, ByteArrayRaw &bytes);
  void bytes2pos(const ByteArrayRaw &bytes, PointType &pt);

  void idx2bytes(const Eigen::Vector3i &idx, ByteArrayRaw &bytes);
  void idx2pos(const Eigen::Vector3i &idx, PointType &pt);

  void update_lidar_pt_gap(const vector<float> &depth);
  void update_lidar_fov_edge(const vector<float> &depth);
  void computeNormal(const PointVector &local_pts, Eigen::Vector3f &normal);
  void computeNormalCell(const PointVector &local_pts, Eigen::Vector3f &normal,
                         Eigen::Vector3f &center);
  void Sphere_PosToIndex(const Eigen::Vector3f &lidar_center,
                         const Eigen::Vector3f &pos, Eigen::Vector2i &id);
  CELL_STATE get_state(const PointType &pt);
  CELL_STATE get_state(const Eigen::Vector3i &idx);
  bool is_gap_point(const PointType &pt);
  bool is_fov_edge(const PointType &pt);
  void get_cells_2_update(const PointVector &points,
                          vector<Eigen::Vector3i> &cells_2_update);
  void get_pts_in_cells(const vector<Eigen::Vector3i> &cells_2_update,
                        vector<PointVector> &pts_inside);
  void updateHalfSpaces(vector<ClusterInfo::Ptr> &clusters);
  //                                const vector<float> &bubble_radius);
  void selectBestViewpoint(ClusterInfo::Ptr &cluster);
  void initClusterViewpoints(ClusterInfo::Ptr &cluster);
  void removeUnreachableViewpoints(vector<ClusterInfo::Ptr> &clusters);
  bool isInBox(const PointType &pt);
  bool isInBox(const Eigen::Vector3f &pt);
  bool computeSuperClusterInfo(SuperClusterInfo::Ptr &super_cluster);
  void reclusterSuperCluster(SuperClusterInfo::Ptr &super_cluster);

public:
  typedef std::shared_ptr<FrontierManager> Ptr;
  Eigen::Isometry3f transform_world2lidar;

  void init(ros::NodeHandle &nh, LIOInterface::Ptr &lio_interface,
            TopoGraph::Ptr graph);
  // unordered_map<int, ClusterInfo::Ptr> new_clusters_;
  std::list<ClusterInfo::Ptr> cluster_list_;
  void printMemoryCost();

  void viz_pocc();
  void viz_point(PointVector &pts2viz, string topic_name);
  void viz_point(vector<Eigen::Vector3f> &pts2viz, string topic_name);
  void visfrtnorm(const std::vector<Eigen::Vector3f> &centers,
                  const std::vector<Eigen::Vector3f> &normals);
  void visfrtcluster();
  void vizBestViewpoint();
  void updateFrontierClusters(vector<ClusterInfo::Ptr> &cluster_updated,
                              vector<int> &cluster_removed);
  void generateTSPViewpoints(Eigen::Vector3f &center_pose,
                             vector<TopoNode::Ptr> &viewpoints);
};