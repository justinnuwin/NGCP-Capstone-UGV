#include "bugv_nav.h"

#include <nav_msgs/Path.h>

BugvNav::BugvNav(ros::NodeHandle nh) {

    paths_pub = nh.advertise<nav_msgs::Path> ("bugv/paths", 10);

    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    for (int i = 0; i < 1000; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = path.header.stamp;
        pose.pose.position.x = (double)i / 10;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        path.poses.push_back(pose);
    }

    paths_pub.publish(path);

}
