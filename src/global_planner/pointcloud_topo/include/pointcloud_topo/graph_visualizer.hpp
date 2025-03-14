/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-05 21:13:16
 * @LastEditTime: 2023-12-05 21:13:35
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */
#pragma once

#include <geometry_msgs/Point.h>
#include <pointcloud_topo/graph.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

enum VizColor {
  RED = 0,
  ORANGE = 1,
  BLACK = 2,
  YELLOW = 3,
  BLUE = 4,
  GREEN = 5,
  EMERALD = 6,
  WHITE = 7,
  MAGNA = 8,
  PURPLE = 9
};

class GraphVisualizer {
public:
  ros::NodeHandle nh_;
  ros::Publisher viz_graph_pub_, global_tour_pub_;
  typedef std::shared_ptr<GraphVisualizer> Ptr;

  void init(ros::NodeHandle &nh) {
    nh_ = nh;
    viz_graph_pub_ = nh_.advertise<MarkerArray>("/viz_graph_topic", 5);
    global_tour_pub_ = nh_.advertise<MarkerArray>("/global_tour", 5);
  }

  void inline static SetColor(const VizColor &color, const float &alpha,
                              Marker &scan_marker) {
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

  void inline SetMarker(const VizColor &color, const std::string &ns,
                        const float &scale, const float &alpha,
                        Marker &scan_marker, const float &scale_ratio) {
    scan_marker.header.frame_id = "world";
    scan_marker.header.stamp = ros::Time::now();
    scan_marker.ns = ns;
    scan_marker.action = Marker::ADD;
    scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z =
        scale * scale_ratio;
    scan_marker.pose.orientation.x = 0.0;
    scan_marker.pose.orientation.y = 0.0;
    scan_marker.pose.orientation.z = 0.0;
    scan_marker.pose.orientation.w = 1.0;
    scan_marker.pose.position.x = 0.0;
    scan_marker.pose.position.y = 0.0;
    scan_marker.pose.position.z = 0.0;
    SetColor(color, alpha, scan_marker);
  }

  void inline vizBox(Eigen::Vector3f &lb, Eigen::Vector3f &hb,
                     Marker &box_marker) {
    box_marker.type = Marker::LINE_LIST;
    // box_marker.points.clear();

    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    p1.x = lb.x();
    p1.y = lb.y();
    p1.z = lb.z();
    p2 = p1;
    p2.x = hb.x();
    p3 = p1;
    p3.y = hb.y();
    p5 = p1;
    p5.z = hb.z();
    p8.x = hb.x();
    p8.y = hb.y();
    p8.z = hb.z();
    p4 = p8;
    p4.z = lb.z();
    p6 = p8;
    p6.y = lb.y();
    p7 = p8;
    p7.x = lb.x();
    box_marker.points.push_back(p1);
    box_marker.points.push_back(p2);

    box_marker.points.push_back(p2);
    box_marker.points.push_back(p4);

    box_marker.points.push_back(p4);
    box_marker.points.push_back(p3);

    box_marker.points.push_back(p3);
    box_marker.points.push_back(p1);

    box_marker.points.push_back(p5);
    box_marker.points.push_back(p6);

    box_marker.points.push_back(p6);
    box_marker.points.push_back(p8);

    box_marker.points.push_back(p8);
    box_marker.points.push_back(p7);

    box_marker.points.push_back(p7);
    box_marker.points.push_back(p5);

    box_marker.points.push_back(p5);
    box_marker.points.push_back(p1);

    box_marker.points.push_back(p6);
    box_marker.points.push_back(p2);

    box_marker.points.push_back(p4);
    box_marker.points.push_back(p8);

    box_marker.points.push_back(p7);
    box_marker.points.push_back(p3);
  }

  void inline vizTour(const vector<Eigen::Vector3f> &path, VizColor color,
                      string ns) {
    MarkerArray tour_marker_array;
    Marker clear_marker;
    clear_marker.action = Marker::DELETEALL;
    if (ns == "global")
      tour_marker_array.markers.push_back(clear_marker);

    if (path.size() >= 2) {
      Marker global_path_marker, global_path_order_marker;
      int id = 0;
      global_path_marker.type = Marker::LINE_LIST;
      global_path_order_marker.type = Marker::TEXT_VIEW_FACING;
      this->SetMarker(color, ns + "_path", 0.1f, 1.0f, global_path_marker,
                      1.0f);
      this->SetMarker(VizColor::WHITE, ns + "_path_order", 0.7f, 1.0f,
                      global_path_order_marker, 1.0f);
      geometry_msgs::Point p1, p2;
      for (int i = 0; i < path.size() - 1; i++) {
        global_path_marker.id = id;
        global_path_order_marker.id = id++;

        p1.x = path[i].x();
        p1.y = path[i].y();
        p1.z = path[i].z();
        p2.x = path[i + 1].x();
        p2.y = path[i + 1].y();
        p2.z = path[i + 1].z();
        global_path_marker.points.emplace_back(p1);
        global_path_marker.points.emplace_back(p2);
        global_path_order_marker.pose.position.x = p2.x;
        global_path_order_marker.pose.position.y = p2.y;
        global_path_order_marker.pose.position.z = p2.z + 0.1;
        global_path_order_marker.text = to_string(i + 1);
        tour_marker_array.markers.emplace_back(global_path_order_marker);
      }
      tour_marker_array.markers.emplace_back(global_path_marker);
    }
    global_tour_pub_.publish(tour_marker_array);
  }

  void inline vizBox(const TopoGraph::Ptr &topo_graph) {
    MarkerArray graph_marker_array;
    Marker boundary_marker, skeleton_edge, odom_edge, dead_arad_marker;
    boundary_marker.action = Marker::DELETEALL;
    graph_marker_array.markers.push_back(boundary_marker); // 清空上一帧的残留

    boundary_marker.action = Marker::ADD;
    boundary_marker.type = Marker::LINE_LIST;
    this->SetMarker(VizColor::WHITE, "boundary", 0.1, 0.5, boundary_marker,
                    1.0);

    dead_arad_marker.action = Marker::ADD;
    dead_arad_marker.type = Marker::LINE_LIST;
    this->SetMarker(VizColor::RED, "dead_area", 0.1, 0.5, dead_arad_marker,
                    1.0);

    Marker box_idx_marker;
    this->SetMarker(VizColor::RED, "box id", 1.0, 1.0, box_idx_marker, 1.0);
    box_idx_marker.action = Marker::ADD;
    box_idx_marker.type = Marker::TEXT_VIEW_FACING;
    for (int i = 0; i < topo_graph->lidar_map_interface_->lp_->dead_area_num_;
         i++) {
      vizBox(
          topo_graph->lidar_map_interface_->lp_->dead_area_min_boundary_vec_[i],
          topo_graph->lidar_map_interface_->lp_->dead_area_max_boundary_vec_[i],
          dead_arad_marker);
    }
    for (int i = 0; i < topo_graph->lidar_map_interface_->lp_->box_num_; i++) {
      vizBox(topo_graph->lidar_map_interface_->lp_
                 ->global_box_min_boundary_vec_[i],
             topo_graph->lidar_map_interface_->lp_
                 ->global_box_max_boundary_vec_[i],
             boundary_marker);
      box_idx_marker.pose.position.x = (topo_graph->lidar_map_interface_->lp_
                                            ->global_box_max_boundary_vec_[i]
                                            .x() +
                                        topo_graph->lidar_map_interface_->lp_
                                            ->global_box_min_boundary_vec_[i]
                                            .x()) /
                                       2.0;
      box_idx_marker.pose.position.y = (topo_graph->lidar_map_interface_->lp_
                                            ->global_box_max_boundary_vec_[i]
                                            .y() +
                                        topo_graph->lidar_map_interface_->lp_
                                            ->global_box_min_boundary_vec_[i]
                                            .y()) /
                                       2.0;
      box_idx_marker.pose.position.z = (topo_graph->lidar_map_interface_->lp_
                                            ->global_box_max_boundary_vec_[i]
                                            .z() +
                                        topo_graph->lidar_map_interface_->lp_
                                            ->global_box_min_boundary_vec_[i]
                                            .z()) /
                                       2.0;
      box_idx_marker.id = i;
      box_idx_marker.text = to_string(i);
      graph_marker_array.markers.push_back(box_idx_marker);
    }
    graph_marker_array.markers.push_back(boundary_marker);
    if (!dead_arad_marker.points.empty())
      graph_marker_array.markers.push_back(dead_arad_marker);
    viz_graph_pub_.publish(graph_marker_array);
  }
  void inline vizGraph(const TopoGraph::Ptr &topo_graph) {
    MarkerArray graph_marker_array;
    Marker skeleton_edge, odom_edge;
    skeleton_edge.action = Marker::DELETEALL;
    graph_marker_array.markers.push_back(skeleton_edge); // 清空上一帧的残留

    // 可视化topoNode
    Marker topo_node_marker, odom_node_marker;
    topo_node_marker.type = Marker::SPHERE_LIST;
    odom_node_marker.type = Marker::SPHERE_LIST;
    this->SetMarker(VizColor::GREEN, "topo_node", 0.2f, 1.0f, topo_node_marker,
                    1.0);
    for (auto &[idx, region] : topo_graph->reg_map_idx2ptr_) {
      if (region == nullptr)
        continue;
      for (auto &topo_node : region->topo_nodes_) {
        geometry_msgs::Point p;
        p.x = topo_node->center_.x();
        p.y = topo_node->center_.y();
        p.z = topo_node->center_.z();
        if (!topo_node->is_viewpoint_ && !topo_node->is_history_odom_node_)
          topo_node_marker.points.push_back(p);
      }
    }
    graph_marker_array.markers.emplace_back(topo_node_marker);

    // 可视化连线
    Marker viewpoint_edge;
    skeleton_edge.type = Marker::LINE_LIST;
    viewpoint_edge.type = Marker::LINE_LIST;
    odom_edge.type = Marker::LINE_LIST;
    this->SetMarker(VizColor::GREEN, "skeleton_edge", 0.04f, 0.4f,
                    skeleton_edge, 1.0);
    this->SetMarker(VizColor::BLUE, "viewpoint_edge", 0.1f, 0.4f,
                    viewpoint_edge, 1.0);
    this->SetMarker(VizColor::ORANGE, "odom_edge", 0.04f, 0.4f, odom_edge, 1.0);

    for (auto &[idx, region] : topo_graph->reg_map_idx2ptr_) {
      if (region == nullptr)
        continue;
      for (auto &topo_node : region->topo_nodes_) {
        for (auto &neighbor : topo_node->neighbors_) {
          geometry_msgs::Point p1, p2;

          for (int i = 0; i < neighbor->paths_[topo_node].size() - 1; i++) {
            p1.x = neighbor->paths_[topo_node][i].x();
            p1.y = neighbor->paths_[topo_node][i].y();
            p1.z = neighbor->paths_[topo_node][i].z();
            p2.x = neighbor->paths_[topo_node][i + 1].x();
            p2.y = neighbor->paths_[topo_node][i + 1].y();
            p2.z = neighbor->paths_[topo_node][i + 1].z();
            if (neighbor->is_history_odom_node_ ||
                topo_node->is_history_odom_node_) {
              odom_edge.points.emplace_back(p1);
              odom_edge.points.emplace_back(p2);
            } else if (neighbor->is_viewpoint_ || topo_node->is_viewpoint_) {
              viewpoint_edge.points.emplace_back(p1);
              viewpoint_edge.points.emplace_back(p2);
            } else {
              skeleton_edge.points.emplace_back(p1);
              skeleton_edge.points.emplace_back(p2);
            }
          }
        }
      }
    }
    graph_marker_array.markers.emplace_back(skeleton_edge);
    graph_marker_array.markers.emplace_back(viewpoint_edge);
    graph_marker_array.markers.emplace_back(odom_edge);
    viz_graph_pub_.publish(graph_marker_array);
  }
};