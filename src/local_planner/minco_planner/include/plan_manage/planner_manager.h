#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <path_searching/bubble_astar.h>

#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/PolyTraj.h>
#include <lidar_map/lidar_map.h>
#include <random>
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include "misc/visualizer.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pointcloud_topo/graph.h>
#include <pointcloud_topo/graph_visualizer.hpp>
#include <pointcloud_topo/parallel_bubble_astar.h>
#include <tf/tf.h>

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called
struct GcopterConfig {
  std::string mapTopic;
  std::string targetTopic;
  double dilateRadiusSoft, dilateRadiusHard;
  double timeoutRRT;
  double maxVelMag;
  double maxBdrMag;
  double maxTiltAngle;
  double minThrust;
  double maxThrust;
  double vehicleMass;
  double gravAcc;
  double horizDrag;
  double vertDrag;
  double parasDrag;
  double speedEps;
  double weightT;
  double WeightSafeT;
  std::vector<double> chiVec;
  double smoothingEps;
  int integralIntervs;
  double relCostTol;
  double corridor_size;
  double yaw_max_vel;
  double yaw_rho_vis;
  double yaw_time_fwd;

  void init(const ros::NodeHandle &nh_priv) {
    nh_priv.getParam("DilateRadiusSoft", dilateRadiusSoft);
    nh_priv.getParam("DilateRadiusHard", dilateRadiusHard);
    nh_priv.getParam("MaxVelMag", maxVelMag);
    nh_priv.getParam("maxBdrMag", maxBdrMag);
    nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
    nh_priv.getParam("MinThrust", minThrust);
    nh_priv.getParam("MaxThrust", maxThrust);
    nh_priv.getParam("VehicleMass", vehicleMass);
    nh_priv.getParam("GravAcc", gravAcc);
    nh_priv.getParam("HorizDrag", horizDrag);
    nh_priv.getParam("VertDrag", vertDrag);
    nh_priv.getParam("ParasDrag", parasDrag);
    nh_priv.getParam("SpeedEps", speedEps);
    nh_priv.getParam("WeightT", weightT);
    nh_priv.getParam("WeightSafeT", WeightSafeT);
    nh_priv.getParam("ChiVec", chiVec);
    nh_priv.getParam("SmoothingEps", smoothingEps);
    nh_priv.getParam("IntegralIntervs", integralIntervs);
    nh_priv.getParam("RelCostTol", relCostTol);
    nh_priv.getParam("MaxCorridorSize", corridor_size);
    nh_priv.getParam("yaw_rho_vis", yaw_rho_vis);
    nh_priv.getParam("yaw_max_vel", yaw_max_vel);
    nh_priv.getParam("yaw_time_fwd", yaw_time_fwd);
  }
};

class FastPlannerManager {
  // SECTION stable
public:
  typedef shared_ptr<FastPlannerManager> Ptr;
  FastPlannerManager();
  ~FastPlannerManager();
  void printTimeCost(double time_threhold, double time_cost, string printInfo);

  bool planExploreTraj(const vector<Eigen::Vector3f> &path, bool is_static);
  bool flyToSafeRegion(bool is_static);
  void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, const ros::Time &start_time);
  void polyYawTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, const ros::Time &start_time);

  void initPlanModules(ros::NodeHandle &nh, ParallelBubbleAstar::Ptr &parallel_path_finder,
                       TopoGraph::Ptr &graph);

  bool checkTrajCollision(double &collision_time);
  bool checkTrajVelocity();

  bool YawTrajOpt(double &start_yaw, double &end_yaw, bool is_static, bool use_shorten_path);
  bool YawTrajwithoutOpt(double &start_yaw, double &end_yaw, bool is_static, bool use_shorten_path);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void posCallback(const nav_msgs::OdometryConstPtr &msg);
  bool YawInterpolationwithoutOpt(double &start, double &end, vector<double> &newYaw,
                                  vector<double> &newDur, double &CompT);
  void YawLookforward(const Trajectory<5> &pos_traj, double &start, double &end,
                      vector<double> &newYaw, vector<double> &newDur, double &CompT);
  void YawLookforwardwithoutOpt(double &start, double &end, vector<double> &newYaw,
                                vector<double> &newDur, double &CompT, bool use_short_path);
  void angleLimite(double &angle);
  void calculateTimelb(const vector<Eigen::Vector3d> &path2next_goal,
                                 const double &current_yaw, const double &goal_yaw, double &time_lb);

  double start_yaw, end_yaw;
  double is_static_yaw = false;

  ros::Subscriber goal_sub;
  ros::Subscriber pos_sub;
  ros::Publisher yaw_state_pub;

  minco::MINCO_S3NU yaw_traj_opt_;
  LocalTrajData local_data_;
  double max_traj_len_;
  LIOInterface::Ptr lidar_map_interface_;
  unique_ptr<Visualizer> gcopter_viz_;
  unique_ptr<GcopterConfig> gcopter_config_;
  BubbleAstar::Ptr bubble_path_finder_;
  ParallelBubbleAstar::Ptr parallel_path_finder_;
  TopoGraph::Ptr topo_graph_;
  GraphVisualizer::Ptr graph_visualizer_;
  FastSearcher::Ptr fast_searcher_;
  bool use_mid360;
  double max_ray_length;
  double fov_up, fov_down;
  double lidar_pitch;

private:
  /* main planning algorithms & modules */
  shared_ptr<SDFMap> sdf_map_;

  // topology guided optimization

  void findCollisionRange(vector<Eigen::Vector3d> &colli_start, vector<Eigen::Vector3d> &colli_end,
                          vector<Eigen::Vector3d> &start_pts, vector<Eigen::Vector3d> &end_pts);

  Eigen::MatrixXd paramLocalTraj(double start_t, double &dt, double &duration);
  Eigen::MatrixXd reparamLocalTraj(const double &start_t, const double &duration, const double &dt);

public:
  void planYawActMap(const Eigen::Vector3d &start_yaw);
  void test();
  void searchFrontier(const Eigen::Vector3d &p);

private:
  // Benchmark method, local exploration
public:
  bool localExplore(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                    Eigen::Vector3d end_pt);

  // !SECTION
};
} // namespace fast_planner

#endif