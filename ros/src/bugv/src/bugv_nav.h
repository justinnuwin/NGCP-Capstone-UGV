#ifndef BUGVNAV_H_
#define BUGVNAV_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/pose_se2.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <teb_local_planner/obstacles.h>
#include <vector>

class BugvNav {
    public:
        BugvNav();
        BugvNav(ros::NodeHandle &nh, teb_local_planner::PoseSE2 start, teb_local_planner::PoseSE2 goal);
        friend void map_cb(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg);
        friend void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    private:
        ros::Publisher path_history_pub;
        ros::Publisher path_plan_pub;
        ros::Publisher marker_pub;
        ros::Subscriber odom_sub;
        ros::Subscriber map_sub;
        nav_msgs::Path path_history;
        teb_local_planner::TebConfig teb_config;
        teb_local_planner::TebVisualizationPtr teb_visual;
        teb_local_planner::TebOptimalPlanner planner;
        costmap_converter::ObstacleArrayMsg obstacles;
        std::vector<teb_local_planner::ObstaclePtr> obst_vector;


        void update_path_history(geometry_msgs::PoseStamped &pose);
        void update_path_plan();
};

 #endif
