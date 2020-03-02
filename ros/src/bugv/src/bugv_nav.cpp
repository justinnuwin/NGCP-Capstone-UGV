#include "bugv_nav.h"

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <cstdint>
#include <teb_local_planner/robot_footprint_model.h>
#include <geometry_msgs/Point.h>

// Marker List:
// - bugv/0: Current Position
// - goal/0: Goal

extern BugvNav *nav;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped path_step;
    path_step.header = msg->header;
    path_step.pose = msg->pose.pose;
    nav->update_path_history(path_step);
    nav->planner.visualize();
    //nav.teb_visual->publishViaPoints(via_points);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "bugv";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = msg->pose.pose;
    // Offset to put the arrow in the middle of the model
    marker.pose.position.z += 0.075;
    marker.pose.position.x -= 0.2;
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    nav->marker_pub.publish(marker);
}

void map_cb(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg) {
    // ObstacleArrayMsg to teb Obstacle conversion see test_optim_node.cpp
    // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
    for (size_t i = 0; i < msg->obstacles.size(); ++i) {
        if (msg->obstacles.at(i).polygon.points.size() == 1 ) {
            if (msg->obstacles.at(i).radius == 0) {
                nav->obst_vector.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(msg->obstacles.at(i).polygon.points.front().x,
                                                                         msg->obstacles.at(i).polygon.points.front().y)));
            } else {
                nav->obst_vector.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::CircularObstacle(msg->obstacles.at(i).polygon.points.front().x,
                                                                            msg->obstacles.at(i).polygon.points.front().y,
                                                                            msg->obstacles.at(i).radius)));
            }
        } else {
            teb_local_planner::PolygonObstacle* polyobst = new teb_local_planner::PolygonObstacle;
            for (size_t j=0; j<msg->obstacles.at(i).polygon.points.size(); ++j) {
                polyobst->pushBackVertex(msg->obstacles.at(i).polygon.points[j].x,
                                         msg->obstacles.at(i).polygon.points[j].y);
            }
            polyobst->finalizePolygon();
            nav->obst_vector.push_back(teb_local_planner::ObstaclePtr(polyobst));
        }  
       
        if (!nav->obst_vector.empty())
            nav->obst_vector.back()->setCentroidVelocity(msg->obstacles.at(i).velocities, msg->obstacles.at(i).orientation);
    }
    nav->update_path_plan();
}

BugvNav::BugvNav() {
}

BugvNav::BugvNav(ros::NodeHandle &nh, teb_local_planner::PoseSE2 start, teb_local_planner::PoseSE2 goal) {
    path_history_pub = nh.advertise<nav_msgs::Path> ("bugv/path_history", 10);
    path_plan_pub = nh.advertise<nav_msgs::Path> ("bugv/path_plan", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker> ("bugv/markers", 10);
    odom_sub = nh.subscribe<nav_msgs::Odometry> ("/odom", 10, odom_cb);
    map_sub = nh.subscribe<costmap_converter::ObstacleArrayMsg> ("costmap_converter/costmap_obstacles", 10, map_cb);

    teb_visual = teb_local_planner::TebVisualizationPtr(new teb_local_planner::TebVisualization(nh, teb_config));
    geometry_msgs::Point footprint_start;
    geometry_msgs::Point footprint_end;
    footprint_start.x = -0.1;     footprint_start.y = 0.0;
    footprint_end.x   = 0.4;      footprint_end.y   = 0.0;
    planner = teb_local_planner::TebOptimalPlanner(teb_config, NULL, boost::make_shared<teb_local_planner::LineRobotFootprint> (footprint_start, footprint_end), teb_visual, NULL);
    planner.plan(start, goal);
}

void BugvNav::update_path_history(geometry_msgs::PoseStamped &current_pose) {
    path_history.header = current_pose.header;
    path_history.poses.push_back(current_pose);
    path_history_pub.publish(path_history);
}

void BugvNav::update_path_plan() {
    this->obstacles = obstacles;
    planner.setObstVector(&obst_vector);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1.5;
    marker.pose.position.y = 1;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_pub.publish(marker);
}
