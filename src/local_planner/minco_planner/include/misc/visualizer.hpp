#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "gcopter/geo_utils.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/trajectory.hpp"

// Visualizer for the planner
class Visualizer {
private:
  // config contains the scale for some markers
  ros::NodeHandle nh;

  // These are publishers for path, waypoints on the trajectory,
  // the entire trajectory, the mesh of free-space polytopes,
  // the edge of free-space polytopes, and spheres for safety radius
  ros::Publisher routePub;
  ros::Publisher routeIdPub;
  ros::Publisher wayPointsPub;
  ros::Publisher trajectoryPub;
  ros::Publisher meshPub;
  ros::Publisher edgePub;
  ros::Publisher spherePub;
  ros::Publisher PolysGenerate_timecostPub;
  ros::Publisher trajOptimize_timecostPub;
  ros::Publisher pointCloudProcess_timecostPub;
  ros::Publisher totoalOptimize_timecostPub;

public:
  ros::Publisher speedPub;
  ros::Publisher thrPub;
  ros::Publisher tiltPub;
  ros::Publisher bdrPub;
  ros::Publisher cloud_inputPub;

public:
  Visualizer() {};

  void init(ros::NodeHandle &nh_) {
    nh = nh_;
    routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
    routeIdPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/routeid", 10);
    wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
    trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
    meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
    spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
    speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
    thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
    tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
    bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);
    PolysGenerate_timecostPub = nh.advertise<std_msgs::Float64>("/visualizer/PolysGenerate_timecost", 1000);
    trajOptimize_timecostPub = nh.advertise<std_msgs::Float64>("/visualizer/trajOptimize_timecost", 1000);
    pointCloudProcess_timecostPub = nh.advertise<std_msgs::Float64>("/visualizer/pointCloudProcess_timecost", 1000);
    totoalOptimize_timecostPub = nh.advertise<std_msgs::Float64>("/visualizer/totoalOptimize_timecost", 1000);
    cloud_inputPub = nh.advertise<sensor_msgs::PointCloud2>("/visualizer/cloud_input", 10);
  }

  inline Eigen::Vector3d jetColorMap(double value) {
    double r, g, b;
    if (value < 0.0)
      value = 0.0;
    else if (value > 1.0)
      value = 1.0;

    if (value < 0.25) {
      r = 0.0;
      g = 4.0 * value;
      b = 1.0;
    } else if (value < 0.5) {
      r = 0.0;
      g = 1.0;
      b = 1.0 - 4.0 * (value - 0.25);
    } else if (value < 0.75) {
      r = 4.0 * (value - 0.5);
      g = 1.0;
      b = 0.0;
    } else {
      r = 1.0;
      g = 1.0 - 4.0 * pow((value - 0.75), 1);
      b = 0.0;
    }

    return Eigen::Vector3d(r, g, b);
  }

  // Visualize the trajectory and its front-end path
  inline void visualizeRoute(const std::vector<Eigen::Vector3f> &route) {
    visualization_msgs::Marker routeMarker;
    routeMarker.id = 0;
    routeMarker.type = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp = ros::Time::now();
    routeMarker.header.frame_id = "world";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::Marker::ADD;
    routeMarker.ns = "route";
    routeMarker.color.r = 1.0f;
    routeMarker.color.g = 0.9f;
    routeMarker.color.b = 1.0f;
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.1;
    if (route.size() > 0) {
      bool first = true;
      Eigen::Vector3f last;
      for (auto it : route) {
        if (first) {
          first = false;
          last = it;
          continue;
        }
        geometry_msgs::Point point;

        point.x = last(0);
        point.y = last(1);
        point.z = last(2);
        routeMarker.points.push_back(point);
        point.x = it(0);
        point.y = it(1);
        point.z = it(2);
        routeMarker.points.push_back(point);
        last = it;
      }
      // ROS_INFO("route num = %d", routeMarker.points.size());
      routePub.publish(routeMarker);
    }

  }

  template <int D> inline void visualize(const Trajectory<D> &traj, double max_vel) {
    visualization_msgs::Marker wayPointsMarker, trajMarker;
    visualization_msgs::Marker routeMarker;
    routeMarker.id = 0;
    routeMarker.type = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp = ros::Time::now();
    routeMarker.header.frame_id = "world";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::Marker::ADD;
    routeMarker.ns = "route";
    routeMarker.color.r = 1.0f;
    routeMarker.color.g = 0.9f;
    routeMarker.color.b = 1.0f;
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.1;
    wayPointsMarker = routeMarker;
    wayPointsMarker.id = 0;
    wayPointsMarker.header.frame_id = "world";
    wayPointsMarker.pose.orientation.w = 1.00;
    wayPointsMarker.action = visualization_msgs::Marker::ADD;

    wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    wayPointsMarker.ns = "waypoints";
    wayPointsMarker.color.r = 1.0f;
    wayPointsMarker.color.g = 0.0f;
    wayPointsMarker.color.b = 0.0f;
    wayPointsMarker.scale.x = 0.35;
    wayPointsMarker.scale.y = 0.35;
    wayPointsMarker.scale.z = 0.35;

    trajMarker = wayPointsMarker;
    trajMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    trajMarker.header.frame_id = "world";
    trajMarker.id = 0;
    trajMarker.ns = "trajectory";
    trajMarker.color.r = 0.00;
    trajMarker.color.g = 0.50;
    trajMarker.color.b = 1.00;
    trajMarker.scale.x = 0.10;
    trajMarker.scale.y = 0.10;
    trajMarker.scale.z = 0.10;

    if (traj.getPieceNum() > 0) {
      Eigen::MatrixXd wps = traj.getPositions();
      for (int i = 0; i < wps.cols(); i++) {
        geometry_msgs::Point point;
        point.x = wps.col(i)(0);
        point.y = wps.col(i)(1);
        point.z = wps.col(i)(2);
        wayPointsMarker.points.push_back(point);
      }

      wayPointsPub.publish(wayPointsMarker);
    }

    if (traj.getPieceNum() > 0) {
      double T = 0.01;
      for (double t = T; t < traj.getTotalDuration(); t += T) {
        geometry_msgs::Point point;
        Eigen::Vector3d X = traj.getPos(t);
        double vel = traj.getVel(t).norm() / max_vel;
        Eigen::Vector3d color = jetColorMap(vel);
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        trajMarker.color.r = color(0);
        trajMarker.color.g = color(1);
        trajMarker.color.b = color(2);
        trajMarker.colors.push_back(trajMarker.color);
        trajMarker.points.push_back(point);
      }
      trajectoryPub.publish(trajMarker);
    }
  }

  // Visualize some polytopes in H-representation
  inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys, bool red_edge = false) {
    // Due to the fact that H-representation cannot be directly visualized
    // We first conduct vertex enumeration of them, then apply quickhull
    // to obtain triangle meshs of polyhedra
    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    for (size_t id = 0; id < hPolys.size(); id++) {
      oldTris = mesh;
      Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
      geo_utils::enumerateVs(hPolys[id], vPoly);

      quickhull::QuickHull<double> tinyQH;
      const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
      const auto &idxBuffer = polyHull.getIndexBuffer();
      int hNum = idxBuffer.size() / 3;

      curTris.resize(3, hNum * 3);
      for (int i = 0; i < hNum * 3; i++) {
        curTris.col(i) = vPoly.col(idxBuffer[i]);
      }
      mesh.resize(3, oldTris.cols() + curTris.cols());
      mesh.leftCols(oldTris.cols()) = oldTris;
      mesh.rightCols(curTris.cols()) = curTris;
    }

    // RVIZ support tris for visualization
    visualization_msgs::Marker meshMarker, edgeMarker;

    meshMarker.id = 0;
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = "world";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = "mesh";
    meshMarker.color.r = 0.00;
    meshMarker.color.g = 0.00;
    meshMarker.color.b = 1.00;
    meshMarker.color.a = 0.07;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    if (red_edge) {
      edgeMarker.color.r = 1.00;
      edgeMarker.color.g = 0.00;
    } else {
      edgeMarker.color.r = 0.00;
      edgeMarker.color.g = 1.00;
    }

    edgeMarker.color.b = 1.00;
    edgeMarker.color.a = 0.20;
    edgeMarker.scale.x = 0.02;

    geometry_msgs::Point point;

    int ptnum = mesh.cols();

    for (int i = 0; i < ptnum; i++) {
      point.x = mesh(0, i);
      point.y = mesh(1, i);
      point.z = mesh(2, i);
      meshMarker.points.push_back(point);
    }

    for (int i = 0; i < ptnum / 3; i++) {
      for (int j = 0; j < 3; j++) {
        point.x = mesh(0, 3 * i + j);
        point.y = mesh(1, 3 * i + j);
        point.z = mesh(2, 3 * i + j);
        edgeMarker.points.push_back(point);
        point.x = mesh(0, 3 * i + (j + 1) % 3);
        point.y = mesh(1, 3 * i + (j + 1) % 3);
        point.z = mesh(2, 3 * i + (j + 1) % 3);
        edgeMarker.points.push_back(point);
      }
    }

    meshPub.publish(meshMarker);
    edgePub.publish(edgeMarker);

    return;
  }

  // Visualize all spheres with centers sphs and the same radius
  inline void visualizeSphere(const Eigen::Vector3d &center1, const Eigen::Vector3d &center2, const double &radius) {
    visualization_msgs::Marker sphereMarkers, sphereDeleter;

    sphereMarkers.id = 0;
    sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp = ros::Time::now();
    sphereMarkers.header.frame_id = "world";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action = visualization_msgs::Marker::ADD;
    sphereMarkers.ns = "spheres";
    sphereMarkers.color.r = 0.00;
    sphereMarkers.color.g = 0.00;
    sphereMarkers.color.b = 1.00;
    sphereMarkers.color.a = 1.00;
    sphereMarkers.scale.x = radius * 2.0;
    sphereMarkers.scale.y = radius * 2.0;
    sphereMarkers.scale.z = radius * 2.0;

    sphereDeleter = sphereMarkers;
    sphereDeleter.action = visualization_msgs::Marker::DELETE;

    geometry_msgs::Point point;
    point.x = center1(0);
    point.y = center1(1);
    point.z = center1(2);
    sphereMarkers.points.push_back(point);
    point.x = center2(0);
    point.y = center2(1);
    point.z = center2(2);
    sphereMarkers.points.push_back(point);

    // spherePub.publish(sphereDeleter);
    spherePub.publish(sphereMarkers);
  }

  inline void visualizeStartGoal(const Eigen::Vector3d &center, const double &radius, const int sg) {
    visualization_msgs::Marker sphereMarkers, sphereDeleter;

    sphereMarkers.id = sg;
    sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp = ros::Time::now();
    sphereMarkers.header.frame_id = "world";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action = visualization_msgs::Marker::ADD;
    sphereMarkers.ns = "StartGoal";
    sphereMarkers.color.r = 1.00;
    sphereMarkers.color.g = 0.00;
    sphereMarkers.color.b = 0.00;
    sphereMarkers.color.a = 1.00;
    sphereMarkers.scale.x = radius * 2.0;
    sphereMarkers.scale.y = radius * 2.0;
    sphereMarkers.scale.z = radius * 2.0;

    sphereDeleter = sphereMarkers;
    sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

    geometry_msgs::Point point;
    point.x = center(0);
    point.y = center(1);
    point.z = center(2);
    sphereMarkers.points.push_back(point);

    if (sg == 0) {
      spherePub.publish(sphereDeleter);
      ros::Duration(1.0e-9).sleep();
      sphereMarkers.header.stamp = ros::Time::now();
    }
    spherePub.publish(sphereMarkers);
  }

  inline void visualizeTimeCost(const double PolysGenerate_time, const double trajOptimize_time, const double pointCloudProcess_time) {
    std_msgs::Float64 time_cost;
    time_cost.data = PolysGenerate_time;
    PolysGenerate_timecostPub.publish(time_cost);
    time_cost.data = trajOptimize_time;
    trajOptimize_timecostPub.publish(time_cost);
    time_cost.data = pointCloudProcess_time;
    pointCloudProcess_timecostPub.publish(time_cost);
    time_cost.data = PolysGenerate_time + pointCloudProcess_time + trajOptimize_time;
    totoalOptimize_timecostPub.publish(time_cost);
  }

  // inline void visualizeCloud(
  //     const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  //     sensor_msgs::PointCloud2 cloud_msg;
  //     pcl::toROSMsg(*cloud, cloud_msg);
  //     cloud_msg.header.stamp = ros::Time::now();
  //     cloud_msg.header.frame_id = "world";
  //     cloud_inputPub.publish(cloud_msg);
  // }
};

#endif