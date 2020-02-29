// Originally sourced from https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html
// Our PX4 uses the APM rover stack instead therefore: OFFBOARD -> MANUAL

#include <iostream>
#include <signal.h>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "bugv_control.h"

nav_msgs::OccupancyGrid map;
void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map = *msg;
    //ROS_WARN_STREAM(ros::message_operations::Printer.stream(msg))
    //ROS_WARN_STREAM("manual_rover: " << map);
}

void clean_quit(int sig) {
    // set_mode("HOLD", false, 2);
    // This will not work if mavros dies before this module.
    // Manually power cycle PixHawk
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bugv_logic");
    ros::NodeHandle nh;

    // mavros_client_init(nh);
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>
            ("rtabmap/grid_map", 10, map_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // override default sigint
    signal(SIGINT, clean_quit);

    while (ros::ok()) {
        // local_pos_pub.publish(pose);
        // mavros

        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();

    return 0;
}


