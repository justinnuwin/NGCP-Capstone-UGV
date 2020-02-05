#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

using namespace tf;

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;
 
    ros::Rate r(100);
 
    TransformBroadcaster broadcaster;
 
    while(n.ok()){
        // TODO: Generalize this code to use roslaunch args instead
        broadcaster.sendTransform(StampedTransform(Transform(Quaternion(0, 0, 0, 1),
                                                             Vector3(0.105, -0.105, 0.030)),
                                                             ros::Time::now(),
                                                             "base_link", "lidar"));
        // zed_camera.launch -> zed_state_publisher
        broadcaster.sendTransform(StampedTransform(Transform(Quaternion(0, 0, 0, 1), 
                                                             Vector3(0.16, 0.085, 0.02)),
                                                             ros::Time::now(),
                                                             "base_link", "zed_description"));
        r.sleep();
    }
}


