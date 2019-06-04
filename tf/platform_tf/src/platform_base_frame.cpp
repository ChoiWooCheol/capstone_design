#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>


/*
  Localization 정보 ("world"프레임과 "base_frame"의 관계)
    방법1. Odometry: dt만큼 움직인 만큼 로봇을 이동시켜 다음 위치를 예측 하는 방법 (Local한 정보를 이용)
    방법2. GPS를 이용: GPS로부터 global한 정보를 이용해 추정위치를 얻는 방법 (Global한 정보를 이용)
*/

void poseCallback(const geometry_msgs::PoseConstPtr& msg){ // world와 base_frame은 고정X (따라서 위치의 변화를 작성해야 함)
 
  static tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  
  transform.setOrigin( tf::Vector3(msg->position.x, msg->position.y, 0.0) );
  transform.setRotation( tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w) );

  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_frame"));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "Estimated_Base_Frame_Pose");

  ros::NodeHandle nh;
  //ros::Subscriber sub = nh.subscribe(base_frame+"/pose", 100, &poseCallback);
ros::Subscriber sub = nh.subscribe("platform_pose_msg", 100, &poseCallback);

  ros::spin();
  return 0;
}