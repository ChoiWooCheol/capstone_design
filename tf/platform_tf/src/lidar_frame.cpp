#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//"base_frame";
//"lidar_frame";

int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_tf");
  ros::NodeHandle nh;
  
  double x, y, yaw;
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  tf::Quaternion Quaternion;

  ros::Rate rate(10.0);
  while (nh.ok()){
    ros::NodeHandle nh;
    nh.getParam("tf_lidar_x", x); 
    nh.getParam("tf_lidar_y", y);
    nh.getParam("tf_lidar_yaw", yaw);

    transform.setOrigin( tf::Vector3(x, y, 0) );  // sensor_frame과 "base_frame"의 position 차이 (x, y만 바꾸면 됨)
  
    Quaternion.setRPY(0.0, 0.0, yaw);
    transform.setRotation(Quaternion);                  // sensor_frame과 "base_frame"의 orientation 차이 (yaw만 바꾸면 됨) 

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_frame", "lidar_frame"));
    rate.sleep();
  }
  return 0;
};