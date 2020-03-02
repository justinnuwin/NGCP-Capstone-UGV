#include "bugv_nav.h"

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <cstdint>
#include <teb_local_planner/robot_footprint_model.h>

// Marker List:
// - bugv/0: Current Position
// - map/0: Origin
// - goal/0: Goal

extern BugvNav *nav;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped path_step;
    path_step.header = msg->header;
    path_step.pose = msg->pose.pose;
    nav->update_path_history(path_step);
    nav->planner.visualize();
    //nav.teb_visual->publishViaPoints(via_points);
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
    map_sub = nh.subscribe<costmap_converter::ObstacleArrayMsg> ("/costmap_obstacles", 10, map_cb);

    // teb_config.loadRosParamFromNodeHandle(nh);
    planner = teb_local_planner::TebOptimalPlanner(teb_config, NULL, boost::make_shared<teb_local_planner::PointRobotFootprint>(), teb_visual, NULL);
    planner.plan(start, goal);
}

void BugvNav::update_path_history(geometry_msgs::PoseStamped &current_pose) {
    path_history.header = current_pose.header;
    path_history.poses.push_back(current_pose);
    path_history_pub.publish(path_history);
}

geometry_msgs::PoseStamped &BugvNav::get_current_pose() {
    return path_history.poses.back();
}

void BugvNav::update_path_plan() {
    this->obstacles = obstacles;
    planner.setObstVector(&obst_vector);

    geometry_msgs::PoseStamped &current_pose = get_current_pose();
    nav_msgs::Path path;
    std::vector<geometry_msgs::PoseStamped> poses(2);
    poses.at(0).header = current_pose.header;
    poses.at(0).pose = current_pose.pose;
    poses.at(1).header.seq = 1;
    poses.at(1).header.frame_id = "map";
    poses.at(1).header.stamp = ros::Time::now();
    poses.at(1).pose.position.x = 1.5;
    poses.at(1).pose.position.y = 1;
    poses.at(1).pose.position.z = 0.1;
    poses.at(1).pose.orientation.x = 0;
    poses.at(1).pose.orientation.y = 0;
    poses.at(1).pose.orientation.z = 0;
    poses.at(1).pose.orientation.w = 1;
    path.header = poses.back().header;
    path.poses = poses;
    path_plan_pub.publish(path);

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
