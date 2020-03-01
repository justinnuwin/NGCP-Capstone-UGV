#ifndef BUGVNAV_H_
#define BUGVNAV_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class BugvNav {
    public:
        BugvNav();
        BugvNav(ros::NodeHandle nh);
        friend void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    private:
        ros::Publisher paths_pub;
        ros::Subscriber odom_sub;
        nav_msgs::Path path;

        void update_path(geometry_msgs::PoseStamped pose);
};

 #endif
