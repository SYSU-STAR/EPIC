#ifndef _UNIT_SPHERE_H
#define _UNIT_SPHERE_H
#include "std_msgs/Empty.h"
#include "lidar_map/ikd_Tree.h"
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;
using namespace std;

template <typename PointType> class KD_TREE;

namespace fast_planner {
struct LIOInterfaceParam;
struct LIOInterfaceData;

class LocalSphere {
public:
  Eigen::Vector3f PreSphereCenter;
  std::vector<double> PreSphere;

  LocalSphere() {};
  ~LocalSphere() {};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LIOInterface {
public:
  typedef std::shared_ptr<LIOInterface> Ptr;
  LIOInterface();
  ~LIOInterface();
  void init(ros::NodeHandle &nh);
  bool IsInBox(const Eigen::Vector3f &pos);
  bool IsInBox(const PointType &pos);
  bool IsInMap(const Eigen::Vector3f &pos);
  bool IsInMap(const PointType &pos);

  double getDisToOcc(const PointType &pt);
  double getDisToOcc(const Eigen::Vector3f &pt);
  double getDisToOcc(const Eigen::Vector3d &pt);
  void KNN(const PointType &pt, int k, PointVector &pts, vector<float> &dis);
  void boxSearch(const Eigen::Vector3f &min, const Eigen::Vector3f &max,
                 PointVector &pts);
  void updateCloudMapOdometry(const sensor_msgs::PointCloud2ConstPtr &msg,
                              const nav_msgs::Odometry::ConstPtr &odom_);
  unique_ptr<LIOInterfaceParam> lp_;
  unique_ptr<LIOInterfaceData> ld_;

private:
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicyCloudOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>>
      SynchronizerCloudOdom;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerCloudOdom sync_cloud_odom_;
  ros::Publisher update_trigger_puber_;

  KD_TREE<PointType> ikd_Tree_map;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// constant
struct LIOInterfaceParam {
  Eigen::Vector3f global_box_min_boundary_, global_box_max_boundary_;
  Eigen::Vector3f global_map_min_boundary_, global_map_max_boundary_;
  int box_num_;
  int dead_area_num_;
  vector<Eigen::Vector3f> global_box_min_boundary_vec_;
  vector<Eigen::Vector3f> global_box_max_boundary_vec_;
  vector<Eigen::Vector3f> dead_area_min_boundary_vec_;
  vector<Eigen::Vector3f> dead_area_max_boundary_vec_;
  double lidar_pitch_;

  double max_ray_length_;
  double fov_up, fov_down;
  double fov_vp_up, fov_vp_down;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// variable
struct LIOInterfaceData {
  bool map_update;
  bool first_map_flag_;
  Eigen::Vector3f lidar_pose_;
  Eigen::Quaternionf lidar_q_;
  Eigen::Vector3f lidar_vel_;
  pcl::PointCloud<pcl::PointXYZ> lidar_cloud_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline bool LIOInterface::IsInBox(const Eigen::Vector3f &pos) {
  auto inbox = [&](const Eigen::Vector3f &pt, const Eigen::Vector3f &min,
                   const Eigen::Vector3f &max) -> bool {
    for (int i = 0; i < 3; i++) {
      if (pt(i) < min(i) || pt(i) > max(i))
        return false;
    }
    return true;
  };

  for (int i = 0; i < lp_->dead_area_num_; i++) {
    Eigen::Vector3f min_ = lp_->dead_area_min_boundary_vec_[i];
    Eigen::Vector3f max_ = lp_->dead_area_max_boundary_vec_[i];
    if (inbox(pos, min_, max_))
      return false;
  }

  for (int i = 0; i < lp_->box_num_; i++) {
    Eigen::Vector3f min_ = lp_->global_box_min_boundary_vec_[i];
    Eigen::Vector3f max_ = lp_->global_box_max_boundary_vec_[i];
    if (inbox(pos, min_, max_))
      return true;
  }
  return false;
}
inline bool LIOInterface::IsInMap(const Eigen::Vector3f &pos) {
  if (pos(0) < lp_->global_map_min_boundary_(0) + 1e-4 ||
      pos(1) < lp_->global_map_min_boundary_(1) + 1e-4 ||
      pos(2) < lp_->global_map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > lp_->global_map_max_boundary_(0) - 1e-4 ||
      pos(1) > lp_->global_map_max_boundary_(1) - 1e-4 ||
      pos(2) > lp_->global_map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

inline bool LIOInterface::IsInBox(const PointType &pos) {
  Eigen::Vector3f p(pos.x, pos.y, pos.z);
  return IsInBox(p);
}

inline bool LIOInterface::IsInMap(const PointType &pos) {
  if (pos.x < lp_->global_map_min_boundary_(0) + 1e-4 ||
      pos.y < lp_->global_map_min_boundary_(1) + 1e-4 ||
      pos.z < lp_->global_map_min_boundary_(2) + 1e-4)
    return false;
  if (pos.x > lp_->global_map_max_boundary_(0) - 1e-4 ||
      pos.y > lp_->global_map_max_boundary_(1) - 1e-4 ||
      pos.z > lp_->global_map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

} // namespace fast_planner
#endif