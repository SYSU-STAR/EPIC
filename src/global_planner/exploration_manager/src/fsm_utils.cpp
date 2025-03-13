#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_fsm.h>

void FastExplorationFSM::pubState() {

  std_msgs::Empty heartbeat_msg;
  heartbeat_pub_.publish(heartbeat_msg);
  std_msgs::Bool msg;
  msg.data = fd_->static_state_;
  static_pub_.publish(msg);
  Marker state_marker;
  state_marker.type = Marker::TEXT_VIEW_FACING;
  state_marker.pose.position.x = fd_->odom_pos_.x();
  state_marker.pose.position.y = fd_->odom_pos_.y();
  state_marker.pose.position.z = fd_->odom_pos_.z();
  state_marker.pose.orientation.w = 1.0;
  state_marker.scale.x = state_marker.scale.y = state_marker.scale.z = 0.5;
  state_marker.action = Marker::ADD;
  state_marker.color.r = 1.0;
  state_marker.color.a = 1.0;
  state_marker.text = fd_->state_str_[int(state_)];
  state_marker.header.frame_id = "world";
  state_marker.header.stamp = ros::Time::now();

  state_pub_.publish(state_marker);
}

int FastExplorationFSM::callExplorationPlanner() {
  // if (planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_) < planner_manager_->gcopter_config_->dilateRadiusHard)
  //   return START_FAIL;
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
    return START_FAIL;
  if (expl_manager_->ed_->global_tour_.size() < 2)
    return NO_FRONTIER;

  // debug
  if (planner_manager_->lidar_map_interface_->getDisToOcc(expl_manager_->ed_->next_goal_node_->center_) <
      planner_manager_->topo_graph_->bubble_min_radius_) { // TODO:
    cout << "410:  next goal in occ, update it" << endl;
    updateTopoAndGlobalPath();
    return FAIL;
  }
  vector<Eigen::Vector3f> path_next_goal;

  int res = planner_manager_->fast_searcher_->search(planner_manager_->topo_graph_->odom_node_, fd_->odom_vel_, expl_manager_->ed_->next_goal_node_,
                                                     0.2, path_next_goal);
  if (res == ParallelBubbleAstar::NO_PATH) {
    ROS_ERROR("ExplorationPlanner: No path to goal");
    return FAIL;

  } else if (res == ParallelBubbleAstar::START_FAIL) {
    ROS_ERROR("ExplorationPlanner: Start point in occ");
    return START_FAIL;
  } else if (res == ParallelBubbleAstar::END_FAIL) {
    ROS_ERROR("ExplorationPlanner: End point in occ");
    return FAIL;
  } else if (res == ParallelBubbleAstar::TIME_OUT) {
    ROS_ERROR("ExplorationPlanner: Time out");
    return FAIL;
  }

  auto info = &planner_manager_->local_data_;

  if (!fd_->static_state_) {
    double plan_finish_time_exp = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
    if (plan_finish_time_exp > info->duration_) {
      plan_finish_time_exp = info->duration_;
    }
    Eigen::Vector3d start_exp = info->minco_traj_.getPos(plan_finish_time_exp);
    path_next_goal.insert(path_next_goal.begin(), start_exp.cast<float>());
  }
  vector<Eigen::Vector3f> path_next_goal_tmp;
  path_next_goal_tmp.push_back(path_next_goal[0]);

  for (int i = 1; i < path_next_goal.size();) {
    Eigen::Vector3f end_pt = path_next_goal_tmp.back();
    if ((path_next_goal[i] - end_pt).norm() > 1.0) {
      Eigen::Vector3f dir = (path_next_goal[i] - end_pt).normalized();
      path_next_goal_tmp.push_back(end_pt + 1.0 * dir);
    } else if ((path_next_goal[i] - end_pt).norm() < 0.01) {
      i++;
    } else {
      path_next_goal_tmp.push_back(path_next_goal[i]);
      i++;
    }
  }
  expl_manager_->ed_->path_next_goal_.swap(path_next_goal_tmp);
  if (planner_manager_->planExploreTraj(expl_manager_->ed_->path_next_goal_, fd_->static_state_)) {
    traj_utils::PolyTraj poly_traj_msg;
    planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
    fd_->newest_traj_ = poly_traj_msg;
    traj_utils::PolyTraj poly_yaw_traj_msg;
    planner_manager_->polyYawTraj2ROSMsg(poly_yaw_traj_msg, info->start_time_);
    fd_->newest_yaw_traj_ = poly_yaw_traj_msg;
    return SUCCEED;
  } else {
    return FAIL;
  }
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg) {
  if (msg->poses[0].pose.position.z < -0.1)
    return;

  if (state_ != WAIT_TRIGGER)
    return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  total_time_ = ros::Time::now().toSec();
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::CloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg, const nav_msgs::Odometry::ConstPtr &odom_) {
  ros::Time t1 = ros::Time::now();
  planner_manager_->lidar_map_interface_->updateCloudMapOdometry(msg, odom_);
  double collision_time;
  bool safe = planner_manager_->checkTrajCollision(collision_time);
  if (!safe) {
    transitState(PLAN_TRAJ, "safetyCallback: not safe, time:" + to_string(collision_time), true);
    if (collision_time < fp_->replan_time_ + 0.2)
      stopTraj();
  }
  ros::Time t2 = ros::Time::now();
  ros::Time t3 = ros::Time::now();

  if (planner_manager_->lidar_map_interface_->ld_->lidar_cloud_.points.empty())
    return;
  auto& ld = planner_manager_->lidar_map_interface_->ld_;
  fd_->odom_pos_ = ld->lidar_pose_;
  fd_->odom_vel_ = ld->lidar_vel_;

  fd_->odom_yaw_ = (float)tf::getYaw(odom_->pose.pose.orientation);
  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  planner_manager_->topo_graph_->odom_node_->center_ = fd_->odom_pos_;
  fd_->have_odom_ = true;
  vector<ClusterInfo::Ptr> new_clusters;
  vector<int> cluster_removed;
  expl_manager_->frontier_manager_ptr_->updateFrontierClusters(new_clusters, cluster_removed);
  for (auto &cls : new_clusters) {
    cls->odom_id_ = planner_manager_->topo_graph_->history_odom_nodes_.size() - 1;
  }
  ros::Time t4 = ros::Time::now();

  ROS_INFO_STREAM_THROTTLE(1.0, "cloud odom callback cost: " << "ikd-tree insert:" << (t2 - t1).toSec() * 1000 << "ms  "
                                                             << "update frontier clusters: " << (t4 - t3).toSec() * 1000 << "ms  "
                                                             << "total: " << (t4 - t1).toSec() * 1000 << "ms" << endl);
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call, bool red) {
  int pre_s = int(state_);
  state_ = new_state;
  if (!red) {
    cout << "\033[32m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  } else {
    cout << "\033[31m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  }
}

void FastExplorationFSM::stopTraj() {
  replan_pub_.publish(std_msgs::Empty());
  ros::Time time_now = ros::Time::now();
  ros::Time start_time = planner_manager_->local_data_.start_time_;
  double curr_dur = planner_manager_->local_data_.duration_;
  planner_manager_->local_data_.duration_ = min(curr_dur, (time_now - start_time).toSec() + fp_->replan_time_);
  if (planner_manager_->local_data_.duration_ <= (time_now - start_time).toSec())
    fd_->static_state_ = true;
}
