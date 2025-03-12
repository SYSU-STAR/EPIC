/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-21 21:31:51
 * @LastEditTime: 2024-03-06 10:28:56
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <epic_planner/fast_exploration_manager.h>
#include <iostream>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pointcloud_topo/graph_visualizer.hpp>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <string>
#include <thread>
#include <vector>
#include <visualization_msgs/Marker.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, CAUTION, EXEC_TRAJ, FINISH, LAND };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, global_path_update_timer_;
  ros::Subscriber trigger_sub_, map_update_sub_, battary_sub_;
  ros::Publisher stop_pub_, new_pub_, replan_pub_, poly_traj_pub_, heartbeat_pub_, time_cost_pub_, poly_yaw_traj_pub_, static_pub_, state_pub_,
  land_pub_;
  double total_time_;

  /*cloud odom callback*/
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  SynchronizerCloudOdom sync_cloud_odom_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  void CloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg, const nav_msgs::Odometry::ConstPtr &odom_);

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call, bool red = false);
  void battaryCallback(const sensor_msgs::BatteryStateConstPtr &msg);
  /* ROS functions */
  void FSMCallback(const ros::TimerEvent &e);
  // void PlannerDebugFSMCallback(const ros::TimerEvent &e);
  void safetyCallback(const ros::TimerEvent &e);
  void updateTopoAndGlobalPath();
  void globalPathUpdateCallback(const ros::TimerEvent &e);
  void triggerCallback(const nav_msgs::PathConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void stopTraj();

  // void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void visualize();
  void pubState();

public:
  FastExplorationFSM(/* args */) {}

  ~FastExplorationFSM() {}

  void init(ros::NodeHandle &nh, FastExplorationManager::Ptr &explorer);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner
