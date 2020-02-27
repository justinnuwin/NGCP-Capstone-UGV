// Originally sourced from https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html
// Our PX4 uses the APM rover stack instead therefore: OFFBOARD -> MANUAL

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <nav_msgs/OccupancyGrid.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

nav_msgs::OccupancyGrid map;
void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map = *msg;
    //ROS_WARN_STREAM(ros::message_operations::Printer.stream(msg))
    //ROS_WARN_STREAM("manual_rover: " << map);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bugv_logic");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher override_rc = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 10);
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>
            ("rtabmap/grid_map", 10, map_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    /*
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    */

    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "AUTO";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(current_state.mode != "AUTO" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(set_mode) &&
                set_mode.response.mode_sent){
                ROS_INFO("Auto mode enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);



        mavros_msgs::OverrideRCIn rc;
        rc.channels[2] = 1580;
        override_rc.publish(rc);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


