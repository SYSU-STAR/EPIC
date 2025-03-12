#include "visualization_msgs/Marker.h"
#include <lidar_map/lidar_map.h>
#include <pcl/filters/voxel_grid.h>
namespace fast_planner {

LIOInterface::LIOInterface() {}

LIOInterface::~LIOInterface() {}

void LIOInterface::init(ros::NodeHandle &nh) {
  lp_.reset(new LIOInterfaceParam);
  ld_.reset(new LIOInterfaceData);
  ikd_Tree_map.setMap_ikdtree(this);
  ikd_Tree_map.Set_delete_criterion_param(0.3);
  ikd_Tree_map.Set_balance_criterion_param(0.6);
  string odom_topic, cloud_topic;
  nh.getParam("odometry_topic", odom_topic);
  nh.getParam("cloud_topic", cloud_topic);
  nh.getParam("box_num", lp_->box_num_);
  nh.getParam("lidar_perception/lidar_pitch", lp_->lidar_pitch_);
  for (int i = 0; i < lp_->box_num_; i++) {
    std::vector<double> tmp;
    nh.getParam("box_" + to_string(i) + "/down", tmp);
    Eigen::Vector3f tmp1(tmp[0], tmp[1], tmp[2]);
    nh.getParam("box_" + to_string(i) + "/up", tmp);
    Eigen::Vector3f tmp2(tmp[0], tmp[1], tmp[2]);
    Eigen::Vector3f min, max;
    for (int i = 0; i < 3; i++) {
      min(i) = fmin(tmp1(i), tmp2(i));
      max(i) = fmax(tmp1(i), tmp2(i));
    }
    lp_->global_box_min_boundary_vec_.emplace_back(min);
    lp_->global_box_max_boundary_vec_.emplace_back(max);
  }
  nh.getParam("dead_area_num", lp_->dead_area_num_);
  for (int i = 0; i < lp_->dead_area_num_; i++) {
    std::vector<double> tmp;
    nh.getParam("dead_" + to_string(i) + "/down", tmp);
    Eigen::Vector3f tmp1(tmp[0], tmp[1], tmp[2]);
    nh.getParam("dead_" + to_string(i) + "/up", tmp);
    Eigen::Vector3f tmp2(tmp[0], tmp[1], tmp[2]);
    Eigen::Vector3f min, max;
    for (int i = 0; i < 3; i++) {
      min(i) = fmin(tmp1(i), tmp2(i));
      max(i) = fmax(tmp1(i), tmp2(i));
    }
    lp_->dead_area_min_boundary_vec_.emplace_back(min);
    lp_->dead_area_max_boundary_vec_.emplace_back(max);
  }

  lp_->global_box_max_boundary_ = lp_->global_box_min_boundary_vec_[0];
  lp_->global_box_min_boundary_ = lp_->global_box_max_boundary_vec_[0];
  for (auto &vec : lp_->global_box_min_boundary_vec_) {
    lp_->global_box_min_boundary_.x() =
        vec.x() < lp_->global_box_min_boundary_.x()
            ? vec.x()
            : lp_->global_box_min_boundary_.x();
    lp_->global_box_min_boundary_.y() =
        vec.y() < lp_->global_box_min_boundary_.y()
            ? vec.y()
            : lp_->global_box_min_boundary_.y();
    lp_->global_box_min_boundary_.z() =
        vec.z() < lp_->global_box_min_boundary_.z()
            ? vec.z()
            : lp_->global_box_min_boundary_.z();
  }
  for (auto &vec : lp_->global_box_max_boundary_vec_) {
    lp_->global_box_max_boundary_.x() =
        vec.x() > lp_->global_box_max_boundary_.x()
            ? vec.x()
            : lp_->global_box_max_boundary_.x();
    lp_->global_box_max_boundary_.y() =
        vec.y() > lp_->global_box_max_boundary_.y()
            ? vec.y()
            : lp_->global_box_max_boundary_.y();
    lp_->global_box_max_boundary_.z() =
        vec.z() > lp_->global_box_max_boundary_.z()
            ? vec.z()
            : lp_->global_box_max_boundary_.z();
  }
  lp_->global_map_max_boundary_ = lp_->global_box_max_boundary_;
  lp_->global_map_min_boundary_ = lp_->global_box_min_boundary_;
  cout << "max boundary: " << lp_->global_map_max_boundary_ << endl;
  cout << "min boundary: " << lp_->global_map_min_boundary_ << endl;
  nh.param("lidar_perception/fov_up", lp_->fov_up, -0.1);
  nh.param("lidar_perception/fov_down", lp_->fov_down, -0.1);
  nh.param("lidar_perception/fov_viewpoint_up", lp_->fov_vp_up, -0.1);
  nh.param("lidar_perception/fov_viewpoint_down", lp_->fov_vp_down, -0.1);
  nh.getParam("lidar_perception/max_ray_length", lp_->max_ray_length_);
  ld_->first_map_flag_ = true;
  // update_trigger_puber_ =
  //     nh.advertise<std_msgs::Empty>("/lio_interface/map_updated", 1);
  // cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
  //     nh, cloud_topic, 5));
  // odom_sub_.reset(
  //     new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic, 1000));
  // sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
  //     SyncPolicyCloudOdom(1), *cloud_sub_, *odom_sub_));
  // sync_cloud_odom_->registerCallback(
  //     boost::bind(&LIOInterface::updateCloudMapOdometry, this, _1, _2));
}

} // namespace fast_planner