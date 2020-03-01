#ifndef BUGV_CONTROL_H_
#define BUGV_CONTROL_H_

#include <ros/ros.h>
#include <mavros_msgs/State.h>

class BugvControl {
    public:
        BugvControl(ros::NodeHandle nh);
        void set_mode(std::string mode, bool armed, float rate);
        friend void state_cb(const mavros_msgs::State::ConstPtr& msg);
    private:
        ros::ServiceClient set_mode_client;
        ros::ServiceClient arming_client;
        ros::Publisher override_rc;

        ros::Subscriber state_sub;
        ros::Publisher local_pos_pub;

};

#endif
