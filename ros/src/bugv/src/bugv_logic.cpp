#include <signal.h>
#include <string>

#include <ros/ros.h>

#include "bugv_control.h"
#include "bugv_nav.h"

BugvNav nav;

void clean_quit(int sig) {
    // set_mode("HOLD", false, 2);
    // This will not work if mavros dies before this module.
    // Manually power cycle PixHawk
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bugv_logic");
    signal(SIGINT, clean_quit);

    ros::NodeHandle nh;
    nav = BugvNav(nh);
    BugvControl control(nh);

    ros::Rate rate(1);
    while (ros::ok()) {
        // local_pos_pub.publish(pose);     // mavros test

        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();

    return 0;
}
