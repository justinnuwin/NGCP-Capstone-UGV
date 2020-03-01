#ifndef BUGVNAV_H_
#define BUGVNAV_H_

#include <ros/ros.h>

class BugvNav {
    public:
        BugvNav(ros::NodeHandle nh);
    private:
        ros::Publisher paths_pub;
};

 #endif
