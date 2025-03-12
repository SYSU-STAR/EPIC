#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/local_explore_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");

  int planner;
  nh.param("planner_node/planner", planner, -1);
  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
