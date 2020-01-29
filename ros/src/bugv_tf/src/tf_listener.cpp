#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformLidar(const tf::TransformListener& listener);
void transformZed(const tf::TransformListener& listener);

void transformPoint(const tf::TransformListener& listener){
  transformLidar(listener);
  transformZed(listener);
}

void transformLidar(const tf::TransformListener& listener){
  //we'll create a point in the base_lidar frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped lidar_point;
  lidar_point.header.frame_id = "lidar";

  //we'll just use the most recent transform available for our simple example
  lidar_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  lidar_point.point.x = 1.0;
  lidar_point.point.y = 0.2;
  lidar_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base", lidar_point, base_point);

    ROS_INFO("base_lidar: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        lidar_point.point.x, lidar_point.point.y, lidar_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_lidar\" to \"base_link\": %s", ex.what());
  }
}

void transformZed(const tf::TransformListener& listener){
  //we'll create a point in the base_camera frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped camera_point;
  camera_point.header.frame_id = "camera";

  //we'll just use the most recent transform available for our simple example
  camera_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  camera_point.point.x = 1.0;
  camera_point.point.y = 0.2;
  camera_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base", camera_point, base_point);

    ROS_INFO("base_camera: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        camera_point.point.x, camera_point.point.y, camera_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_camera\" to \"base_link\": %s", ex.what());
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
