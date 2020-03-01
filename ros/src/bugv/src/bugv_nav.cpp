#include "bugv_nav.h"

extern BugvNav nav;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped path_step;
    path_step.header = msg->header;
    path_step.pose = msg->pose.pose;
    nav.update_path_history(path_step);
}

void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    nav.map = *msg;
    nav.update_path_plan();
}

BugvNav::BugvNav() {
}

BugvNav::BugvNav(ros::NodeHandle nh) {
    path_history_pub = nh.advertise<nav_msgs::Path> ("bugv/path_history", 10);
    path_plan_pub = nh.advertise<nav_msgs::Path> ("bugv/path_plan", 10);
    odom_sub = nh.subscribe<nav_msgs::Odometry> ("/odom", 10, odom_cb);
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid> ("rtabmap/grid_map", 10, map_cb);
}

void BugvNav::update_path_history(geometry_msgs::PoseStamped &pose) {
    path_history.header = pose.header;
    path_history.poses.push_back(pose);
    path_history_pub.publish(path_history);
}

void BugvNav::update_path_plan() {
    nav_msgs::Path path;
    std::vector<geometry_msgs::PoseStamped> poses(1000);
    for (int i = 0; i < 1000; ++i) {
        poses.at(i).header.seq = i;
        poses.at(i).header.frame_id = "map";
        poses.at(i).header.stamp = ros::Time::now();
        poses.at(i).pose.position.x = (double)i / 10;
        poses.at(i).pose.position.y = 0;
        poses.at(i).pose.position.z = 0.1;
    }

    path.header = poses.at(999).header;
    path.poses = poses;
    path_plan_pub.publish(path);
}
