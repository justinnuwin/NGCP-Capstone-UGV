#include "bugv_control.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>

mavros_msgs::State current_state;
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
ros::Publisher override_rc;

ros::Subscriber state_sub;
ros::Publisher local_pos_pub;

ros::Rate r(20);

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void mavros_client_init(ros::NodeHandle nh) {
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    override_rc = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 10);

    while (ros::ok() && !current_state.connected) {     // wait for FCU connection
        ros::spinOnce();
        r.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        r.sleep();
    }

    set_mode("GUIDED", true);
}

void set_mode(std::string mode, bool armed, float rate) {
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = mode;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = armed;

    ROS_WARN_STREAM("Setting mode " << mode << (armed ? "ARMED" : "DISARMED"));

    ros::Rate r(rate);
    while (!(current_state.mode == mode && current_state.armed == armed)) {
        if (current_state.mode != mode) {
            if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                ROS_INFO_STREAM("Mode set: " << current_state.mode);
        } 
        if (current_state.armed != armed) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                if (armed)
                    ROS_FATAL("ARMED!");
                else
                    ROS_FATAL("DISARMED!");
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}

mavros_msgs::OverrideRCIn rc;
void test_rc() {
    int test = 0;

    while (test < 20) {
        if (test < 5) {
            ROS_WARN_STREAM("\tGoing Test value: " << test);
            rc.channels[2] = 1580;
            override_rc.publish(rc);
        } else {
            ROS_WARN_STREAM("\tStopping: " << test);
            rc.channels[2] = 1490;
            override_rc.publish(rc);
        }
        test++;
    }
}
