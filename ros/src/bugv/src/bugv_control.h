#ifndef BUGV_CONTROL_H_
#define BUGV_CONTROL_H_

#include <ros/ros.h>
#include <mavros_msgs/State.h>

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void mavros_client_init(ros::NodeHandle nh);
void set_mode(std::string mode, bool armed, float rate=0.2);

#endif
