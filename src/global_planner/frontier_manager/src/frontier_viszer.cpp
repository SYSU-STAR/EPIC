/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-07-08 15:24:56
 * @LastEditTime: 2024-08-05 20:55:22
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <frontier_manager/frontier_manager.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

enum VizColor { RED = 0, ORANGE = 1, BLACK = 2, YELLOW = 3, BLUE = 4, GREEN = 5, EMERALD = 6, WHITE = 7, MAGNA = 8, PURPLE = 9 };

void inline static SetColor(const VizColor &color, const float &alpha, Marker &scan_marker) {
  std_msgs::ColorRGBA c;
  c.a = alpha;
  if (color == VizColor::RED) {
    c.r = 1.0f, c.g = c.b = 0.f;
  } else if (color == VizColor::ORANGE) {
    c.r = 1.0f, c.g = 0.45f, c.b = 0.1f;
  } else if (color == VizColor::BLACK) {
    c.r = c.g = c.b = 0.1f;
  } else if (color == VizColor::YELLOW) {
    c.r = c.g = 0.9f, c.b = 0.1;
  } else if (color == VizColor::BLUE) {
    c.b = 1.0f, c.r = 0.1f, c.g = 0.1f;
  } else if (color == VizColor::GREEN) {
    c.g = 0.9f, c.r = c.b = 0.f;
  } else if (color == VizColor::EMERALD) {
    c.g = c.b = 0.9f, c.r = 0.f;
  } else if (color == VizColor::WHITE) {
    c.r = c.g = c.b = 0.9f;
  } else if (color == VizColor::MAGNA) {
    c.r = c.b = 0.9f, c.g = 0.f;
  } else if (color == VizColor::PURPLE) {
    c.r = c.b = 0.5f, c.g = 0.f;
  }
  scan_marker.color = c;
}

void inline SetMarker(const VizColor &color, const std::string &ns, const float &scale, const float &alpha, Marker &scan_marker,
                      const float &scale_ratio) {
  scan_marker.header.frame_id = "world";
  scan_marker.header.stamp = ros::Time::now();
  scan_marker.ns = ns;
  scan_marker.action = Marker::ADD;
  scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale * scale_ratio;
  scan_marker.pose.orientation.x = 0.0;
  scan_marker.pose.orientation.y = 0.0;
  scan_marker.pose.orientation.z = 0.0;
  scan_marker.pose.orientation.w = 1.0;
  scan_marker.pose.position.x = 0.0;
  scan_marker.pose.position.y = 0.0;
  scan_marker.pose.position.z = 0.0;
  SetColor(color, alpha, scan_marker);
}

void FrontierManager::visfrtcluster() {
  if (!frtp_.view_cluster_)
    return;
  static ros::Publisher sf_cluster_pub = nh_.advertise<visualization_msgs::MarkerArray>("sf_cluster_marker", 5);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(marker);

  for (auto &sf_cluster : cluster_list_) {

    visualization_msgs::Marker aabb_marker, viewpoint_number;
    visualization_msgs::Marker best_viewpoint, vp_frt_connecter;
    if (!sf_cluster->is_reachable_ || sf_cluster->is_dormant_) {
      SetMarker(VizColor::BLACK, "aabb", 1.0, 0.5, aabb_marker, 1.0);
    } else if (!sf_cluster->is_new_cluster_) {
      SetMarker(VizColor::GREEN, "aabb", 1.0, 0.5, aabb_marker, 1.0);
    } else {
      SetMarker(VizColor::RED, "aabb", 1.0, 0.5, aabb_marker, 1.0);
    }
    SetMarker(VizColor::WHITE, "viewpoint_number", 0.5, 1.0, viewpoint_number, 1.0);
    SetMarker(VizColor::RED, "best_viewpoint", 0.5, 1.0, best_viewpoint, 1.0);
    SetMarker(VizColor::WHITE, "vp_frt_connecter", 0.05, 0.7, vp_frt_connecter, 1.0);

    aabb_marker.id = sf_cluster->id_;
    viewpoint_number.id = sf_cluster->id_;
    best_viewpoint.id = sf_cluster->id_;
    vp_frt_connecter.id = sf_cluster->id_;

    aabb_marker.type = visualization_msgs::Marker::CUBE;
    viewpoint_number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    best_viewpoint.type = visualization_msgs::Marker::ARROW;
    vp_frt_connecter.type = visualization_msgs::Marker::LINE_STRIP;

    aabb_marker.pose.position.x = (sf_cluster->box_min_.x() + sf_cluster->box_max_.x()) / 2.0;
    aabb_marker.pose.position.y = (sf_cluster->box_min_.y() + sf_cluster->box_max_.y()) / 2.0;
    aabb_marker.pose.position.z = (sf_cluster->box_min_.z() + sf_cluster->box_max_.z()) / 2.0;
    aabb_marker.scale.x = sf_cluster->box_min_.x() - sf_cluster->box_max_.x();
    aabb_marker.scale.y = sf_cluster->box_min_.y() - sf_cluster->box_max_.y();
    aabb_marker.scale.z = sf_cluster->box_min_.z() - sf_cluster->box_max_.z();
    viewpoint_number.pose = aabb_marker.pose;
    int size = 0;
    for (auto &vp_cluster : sf_cluster->vp_clusters_) {
      size += vp_cluster.vps_.size();
    }
    viewpoint_number.text = std::to_string(size);

    if (sf_cluster->is_reachable_ && !sf_cluster->is_dormant_ && !sf_cluster->is_new_cluster_) {
      best_viewpoint.scale.x = 0.2;
      best_viewpoint.scale.y = 0.5;
      best_viewpoint.scale.z = 0.5;
      Eigen::Vector3f best_vp = sf_cluster->best_vp_;
      float yaw = sf_cluster->best_vp_yaw_;
      Eigen::Vector3f diff(cos(yaw), sin(yaw), 0);
      geometry_msgs::Point pt;
      pt.x = best_vp.x();
      pt.y = best_vp.y();
      pt.z = best_vp.z();
      best_viewpoint.points.push_back(pt);
      vp_frt_connecter.points.push_back(pt);

      pt.x = best_vp.x() + diff.x();
      pt.y = best_vp.y() + diff.y();
      pt.z = best_vp.z() + diff.z();
      best_viewpoint.points.push_back(pt);
      marker_array.markers.push_back(best_viewpoint);
      Eigen::Vector3f center = (sf_cluster->box_min_ + sf_cluster->box_max_) / 2.0;
      pt.x = center.x();
      pt.y = center.y();
      pt.z = center.z();
      vp_frt_connecter.points.push_back(pt);
      marker_array.markers.push_back(vp_frt_connecter);
    }

    marker_array.markers.push_back(viewpoint_number);
    marker_array.markers.push_back(aabb_marker);
  }

  sf_cluster_pub.publish(marker_array);
}

void FrontierManager::visfrtnorm(const std::vector<Eigen::Vector3f> &centers, const std::vector<Eigen::Vector3f> &normals) {
  static ros::Publisher norm_pub = nh_.advertise<visualization_msgs::MarkerArray>("norm_directions", 1);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  for (size_t i = 0; i < centers.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; // 你的坐标系名称
    marker.header.stamp = ros::Time::now();
    marker.ns = "normal_directions";
    marker.id = i; // unique id for each marker
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    // 设置箭头方向
    geometry_msgs::Point end_point;
    geometry_msgs::Point start_point;
    start_point.x = centers[i].x();
    start_point.y = centers[i].y();
    start_point.z = centers[i].z();
    end_point.x = centers[i].x() + normals[i].x();
    end_point.y = centers[i].y() + normals[i].y();
    end_point.z = centers[i].z() + normals[i].z();
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    // 设置颜色和尺寸
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.scale.x = 0.05; // 箭头宽度
    marker.scale.y = 0.08; // 箭头长度
    marker.scale.z = 0.1;

    marker_array.markers.push_back(marker);
  }

  norm_pub.publish(marker_array);
}

void FrontierManager::viz_point(PointVector &pts2viz, string topic_name) {
  static unordered_map<string, ros::Publisher> pub_map;
  ros::Publisher occ_pub;
  if (pub_map.count(topic_name))
    occ_pub = pub_map[topic_name];
  else {
    occ_pub = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 5);
    pub_map[topic_name] = occ_pub;
  }
  pcl::PointCloud<pcl::PointXYZ> occ_cloud;
  occ_cloud.width = pts2viz.size();
  occ_cloud.height = 1;
  occ_cloud.points = pts2viz;
  sensor_msgs::PointCloud2 occ_msg;
  pcl::toROSMsg(occ_cloud, occ_msg);
  occ_msg.header.stamp = ros::Time::now();
  occ_msg.header.frame_id = "world";
  occ_pub.publish(occ_msg);
}

void FrontierManager::viz_point(vector<Eigen::Vector3f> &pts2viz, string topic_name) {

  PointVector pts;
  pts.reserve(pts2viz.size());
  for (auto &pt : pts2viz) {
    pts.emplace_back(pt.x(), pt.y(), pt.z());
  }
  viz_point(pts, topic_name);
}

void FrontierManager::viz_pocc() {
  if (!frtp_.view_frt_)
    return;
  // cout << "bucket_count: " << frtd_.label_map_.bucket_count() << endl;
  // cout << "load factor: " << frtd_.label_map_.load_factor() << endl;
  static ros::Publisher occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("occ", 5);
  static ros::Publisher pocc_pub = nh_.advertise<sensor_msgs::PointCloud2>("pocc", 5);
  static ros::Publisher frt_pub = nh_.advertise<sensor_msgs::PointCloud2>("frt", 5);
  PointVector occ_pts, pocc_pts, frt_pts;
  for (auto &[bytes, label] : frtd_.label_map_) {
    // Eigen::Vector3i idx = pt_label.first;
    // Eigen::Vector3f pt =
    // (idx.cast<float>() + 0.5 * Eigen::Vector3f::Ones()) * frtp_.cell_size_ + frtp_.map_min_;
    PointType pt;
    bytes2pos(bytes, pt);
    if (label == SPARSE) {
      pocc_pts.emplace_back(pt);
    } else if (label == DENSE) {
      occ_pts.emplace_back(pt);
    } else {
      frt_pts.emplace_back(pt);
    }
  }
  pcl::PointCloud<pcl::PointXYZ> occ_cloud;
  pcl::PointCloud<pcl::PointXYZ> pocc_cloud;
  pcl::PointCloud<pcl::PointXYZ> frt_cloud;
  occ_cloud.width = occ_pts.size();
  occ_cloud.height = 1;
  occ_cloud.points = occ_pts;
  pocc_cloud.width = pocc_pts.size();
  pocc_cloud.height = 1;
  pocc_cloud.points = pocc_pts;
  frt_cloud.width = frt_pts.size();
  frt_cloud.height = 1;
  frt_cloud.points = frt_pts;

  sensor_msgs::PointCloud2 occ_msg, pocc_msg, frt_msg;

  pcl::toROSMsg(occ_cloud, occ_msg);
  pcl::toROSMsg(pocc_cloud, pocc_msg);
  pcl::toROSMsg(frt_cloud, frt_msg);
  occ_msg.header.stamp = ros::Time::now();
  pocc_msg.header.stamp = ros::Time::now();
  occ_msg.header.frame_id = "world";
  pocc_msg.header.frame_id = "world";
  frt_msg.header.frame_id = "world";

  occ_pub.publish(occ_msg);
  pocc_pub.publish(pocc_msg);
  frt_pub.publish(frt_msg);
}
