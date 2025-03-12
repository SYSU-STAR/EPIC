/*
If you need to replace with other point cloud data structures, please
re-implement the following interfaces:

1. getDisToOcc: Returns the distance from the specified point to the nearest
obstacle in the map.
2. KNN: Nearest neighbor search. boxSearch: Region search.
3. updateCloudMapOdometry: Point cloud map update. 
4. LIOInterfaceData::Ptr ld: Current frame world coordinate point clouds and
lidar-odometry.

If you need to integrate EPIC with Lidar SLAM algorithm and shares
memory, thread mutual exclusion should be noted.
*/
#include "visualization_msgs/Marker.h"
#include <lidar_map/lidar_map.h>
#include <pcl/filters/voxel_grid.h>
namespace fast_planner {

double LIOInterface::getDisToOcc(const PointType &pt) {
  PointVector nes_pts;
  vector<float> diss;
  KNN(pt, 1, nes_pts, diss);
  if (nes_pts.size() == 0)
    return 10.0;
  else
    return sqrt(diss[0]);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3d &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3f &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
void LIOInterface::KNN(const PointType &pt, int k, PointVector &pts,
                       vector<float> &dis) {
  ikd_Tree_map.Nearest_Search(pt, k, pts, dis, 10.0);
}
void LIOInterface::boxSearch(const Eigen::Vector3f &min_bd,
                             const Eigen::Vector3f &max_bd, PointVector &pts) {
  BoxPointType boxpoint;
  for (int i = 0; i < 3; i++) {
    boxpoint.vertex_min[i] = min_bd(i);
    boxpoint.vertex_max[i] = max_bd(i);
  }
  ikd_Tree_map.Box_Search(boxpoint, pts);
}
void LIOInterface::updateCloudMapOdometry(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    const nav_msgs::Odometry::ConstPtr &odom_) {
  ld_->map_update = true;
  static Eigen::Vector3f last_lidar_pose(0, 0, 0);
  Eigen::Vector3f lidar_pos_(odom_->pose.pose.position.x,
                             odom_->pose.pose.position.y,
                             odom_->pose.pose.position.z);
  Eigen::Vector3f lidar_vel_(odom_->twist.twist.linear.x,
                             odom_->twist.twist.linear.y,
                             odom_->twist.twist.linear.z);
  last_lidar_pose = lidar_pos_;
  ld_->lidar_pose_ = lidar_pos_;
  ld_->lidar_vel_ = lidar_vel_;
  Eigen::AngleAxisf y_axis_angle(M_PI / 180.0 * lp_->lidar_pitch_,
                                 Eigen::Vector3f::UnitY());
  Eigen::Quaternionf q_y(y_axis_angle);
  ld_->lidar_q_ = Eigen::Quaternionf(odom_->pose.pose.orientation.w,
                                     odom_->pose.pose.orientation.x,
                                     odom_->pose.pose.orientation.y,
                                     odom_->pose.pose.orientation.z) *
                  q_y;

  pcl::fromROSMsg(*msg, ld_->lidar_cloud_);
  ros::Time start = ros::Time::now();
  // PointVector pcl_map = points.points;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(0.1, 0.1, 0.1);
  vg.setInputCloud(ld_->lidar_cloud_.makeShared());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*filtered_points);
  PointVector pcl_map = filtered_points->points;

  if (pcl_map.empty())
    return;

  if (ld_->first_map_flag_) {
    // this->ikd_Tree_map(0.3,0.6,0.2);
    this->ikd_Tree_map.set_downsample_param(0.1);
    this->ikd_Tree_map.Build(pcl_map);
    ld_->first_map_flag_ = false;
  } else {
    this->ikd_Tree_map.Add_Points(pcl_map, true);
  }
  ros::Time ikd_update_end_stamp = ros::Time::now();
}

} // namespace fast_planner