#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>


/*
  Localization 정보 ("map"프레임과 "base_link"의 관계)
    방법1. Odometry: dt만큼 움직인 만큼 로봇을 이동시켜 다음 위치를 예측 하는 방법 (Local한 정보를 이용)
    방법2. GPS를 이용: GPS로부터 global한 정보를 이용해 추정위치를 얻는 방법 (Global한 정보를 이용)
    *** 위 2개의 Odometry정보 받아서 EKF ***
*/

void callback(const nav_msgs::OdometryConstPtr& odom){ // map와 base_link은 고정X (따라서 위치의 변화를 작성해야 함)
 
  static tf::TransformBroadcaster broadcaster;
  
  tf::Transform transform;
  
  transform.setOrigin( tf::Vector3(odom->pose.pose.position.x, 
                                   odom->pose.pose.position.y, 
                                   odom->pose.pose.position.z) );

  transform.setRotation( tf::Quaternion(odom->pose.pose.orientation.x, 
                                        odom->pose.pose.orientation.y, 
                                        odom->pose.pose.orientation.z, 
                                        odom->pose.pose.orientation.w) );

  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "Estimated__Pose_by_Odometry");

  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("odom/wheelbased", 100, &callback);

  ros::spin();
  return 0;
}

