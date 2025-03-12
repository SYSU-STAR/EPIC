/*** 
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-25 15:47:36
 * @LastEditTime: 2024-03-05 19:53:01
 * @Description: 
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved. 
 */


#ifndef BUBBLE_VISUALIZER_HPP
#define BUBBLE_VISUALIZER_HPP
#include <path_searching/bubble_astar.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
namespace fast_planner {
class BubbleVisualizer {
   public:
    ros::NodeHandle nh_;
    ros::Publisher spherePub, wayPointsPub;

    BubbleVisualizer() {}
    ~BubbleVisualizer() {}
    void init(ros::NodeHandle &nh) {
        nh_ = nh;
        spherePub = nh.advertise<visualization_msgs::MarkerArray>(
            "/bubble_visualizer/sphere", 1000);
        wayPointsPub = nh.advertise<visualization_msgs::MarkerArray>(
            "/bubble_visualizer/frontend_traj", 10);
    }
    void inline visualize(const std::vector<GridNodePtr> &bubble_nodes_) {
        visualization_msgs::MarkerArray sphereMarkersArray;
        visualization_msgs::MarkerArray sphereMarkersArray_frontend_traj;

        for (int i = 0; i < bubble_nodes_.size(); i++) {
            visualization_msgs::Marker sphereMarkers;
            sphereMarkers.id = i;
            sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
            sphereMarkers.header.stamp = ros::Time::now();
            sphereMarkers.header.frame_id = "world";
            sphereMarkers.pose.orientation.w = 1.00;
            sphereMarkers.action = visualization_msgs::Marker::ADD;
            sphereMarkers.ns = "spheres";
            sphereMarkers.color.r = 0.00;
            sphereMarkers.color.g = 0.00;
            sphereMarkers.color.b = 1.00;
            sphereMarkers.color.a = 0.10;
            sphereMarkers.scale.x = sqrt(bubble_nodes_[i]->safe_bubble->radius2) * 2.0;
            sphereMarkers.scale.y = sqrt(bubble_nodes_[i]->safe_bubble->radius2) * 2.0;
            sphereMarkers.scale.z = sqrt(bubble_nodes_[i]->safe_bubble->radius2) * 2.0;
            sphereMarkersArray.markers.push_back(sphereMarkers);

            sphereMarkers.color.r = 0.00;
            sphereMarkers.color.g = 0.00;
            sphereMarkers.color.b = 1.00;
            sphereMarkers.color.a = 1.00;
            sphereMarkers.scale.x = 0.1;
            sphereMarkers.scale.y = 0.1;
            sphereMarkers.scale.z = 0.1;
            sphereMarkersArray_frontend_traj.markers.push_back(sphereMarkers);
        }
        spherePub.publish(sphereMarkersArray);
        wayPointsPub.publish(sphereMarkersArray_frontend_traj);
    }
};
}  // namespace fast_planner
#endif