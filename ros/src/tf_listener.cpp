#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

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
  //we'll create a point in the base_zed frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped zed_point;
  zed_point.header.frame_id = "zed";

  //we'll just use the most recent transform available for our simple example
  zed_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  zed_point.point.x = 1.0;
  zed_point.point.y = 0.2;
  zed_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base", zed_point, base_point);

    ROS_INFO("base_zed: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        zed_point.point.x, zed_point.point.y, zed_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_zed\" to \"base_link\": %s", ex.what());
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
