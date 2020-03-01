#ifndef BUGVNAV_H_
#define BUGVNAV_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

class BugvNav {
    public:
        BugvNav();
        BugvNav(ros::NodeHandle nh);
        friend void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        friend void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    private:
        ros::Publisher path_history_pub;
        ros::Publisher path_plan_pub;
        ros::Subscriber odom_sub;
        ros::Subscriber map_sub;
        nav_msgs::Path path_history;
        nav_msgs::OccupancyGrid map;

        void update_path_history(geometry_msgs::PoseStamped &pose);
        void update_path_plan();
};

 #endif
