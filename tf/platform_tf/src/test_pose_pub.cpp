#include "ros/ros.h"                            // ROS 기본 헤더파일
#include "geometry_msgs/Pose.h"                  // test_msg 메시지 파일 헤더 (빌드후 자동 생성됨)



int main(int argc, char **argv)                 // 노드 메인 함수
{
  ros::init(argc, argv, "pub_pose_node");  // 노드명 초기화 (test_pub_node_name)
  ros::NodeHandle nh;                           // ROS 시스템과 통신을 위한 노드 핸들 선언 (인스턴스)

  // 퍼블리셔 선언, test_pkg 패키지의 test_msg 메시지 파일을 이용한
  // 퍼블리셔 test_pub 를 작성한다. 토픽명은 "test_msg_topic_name" 이며,
  // 퍼블리셔 큐(queue) 사이즈를 100개로 설정한다는 것이다
  ros::Publisher test_pub = nh.advertise<geometry_msgs::Pose>("platform_pose_msg", 100);

  // 루프 주기를 설정한다. "10" 이라는 것은 10Hz를 말하는 것으로 0.1초 간격으로 반복된다
  ros::Rate loop_rate(10);

  geometry_msgs::Pose pose;                       // test_msg 메시지 파일 형식으로 msg 라는 메시지를 선언
  
  int count = 0;                                // 메시지에 사용될 변수 선언

  while (ros::ok())
  {
    pose.position.x = 0;                           // count라는 변수 값을 msg의 하위 data 메시지에 담는다
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    //ROS_INFO("send msg = %d", pose.stamp.sec);    // stamp.sec 메시지를 표시한다
    //ROS_INFO("send msg = %d", pose.stamp.nsec);   // stamp.nsec 메시지를 표시한다
    //ROS_INFO("send msg = %d", pose.pose);         // data 메시지를 표시한다

    test_pub.publish(pose);                       // 메시지를 발행한다. 약 0.1초 간격으로 발행된다

    loop_rate.sleep();                           // 위에서 정한 루프 주기에 따라 슬립에 들어간다

    ++count;                                     // count 변수 1씩 증가
  }

  return 0;
}
