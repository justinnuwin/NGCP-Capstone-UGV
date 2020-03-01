#include "bugv_nav.h"

extern BugvNav nav;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped path_step;
    path_step.header = msg->header;
    path_step.pose = msg->pose.pose;
    nav.update_path(path_step);
}

BugvNav::BugvNav() {
}

BugvNav::BugvNav(ros::NodeHandle nh) {
    paths_pub = nh.advertise<nav_msgs::Path> ("bugv/paths", 10);
    odom_sub = nh.subscribe<nav_msgs::Odometry> ("/odom", 10, odom_cb);
}

void BugvNav::update_path(geometry_msgs::PoseStamped pose) {

    /*
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::PoseStamped> poses(1000);
    for (int i = 0; i < 1000; ++i) {
        poses.at(i).header.frame_id = "map";
        poses.at(i).header.stamp = path.header.stamp;
        poses.at(i).pose.position.x = (double)i / 10;
        poses.at(i).pose.position.y = 0;
        poses.at(i).pose.position.z = 1;
    }
    path.poses = poses;
    */

    // TODO: Implement this as path_history topic and add the above comments as planned path topic
    path.header = pose.header;
    path.poses.push_back(pose);
    paths_pub.publish(path);
}
