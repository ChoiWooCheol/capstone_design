#include <ros/ros.h>
#include <serial/serial.h>
#include <deque>
#include <thread>
#include <mutex>
#include "autodrive/autodrive.h"
#include "pid_controller.h"
#include "platform_rx_msg/platform_rx_msg.h"

#define TX_PACKET_LENGTH 14
#define TX_SERIAL_FREQUENCY 14
#define TX_STOP_CHECK_PERIOD 10


PID_Controller PID();

uint8_t packet[TX_PACKET_LENGTH] = {};  //플랫폼에 패킷을 보낼때 받을때와는 다르게 14바이트로 보넴 packet[14]
double curr_speed;
//serial
serial::Serial *ser;
std::mutex lock;

void initTx(const ros::NodeHandle& nh){
    packet[0] = static_cast<uint8_t>(0x53);
    packet[1] = static_cast<uint8_t>(0x54);
    packet[2] = static_cast<uint8_t>(0x58);
    //manual OR auto. 0x00 - manual/0x01 - auto
    packet[3] = static_cast<uint8_t>(0x01);
    //estop. 0x00 - off/0x01 - on
    packet[4] = static_cast<uint8_t>(0x00);
    packet[12] = static_cast<uint8_t>(0x0D);//0x0D
    packet[13] = static_cast<uint8_t>(0x0A);//0x0A
}

void serialWrite(){ // thread tr이 수행하는 함수.
    ros::Rate loop_rate(TX_SERIAL_FREQUENCY);
    uint8_t alive = 0;
    uint8_t packet_local[14] = ""; 
    while(true){
        lock.lock();
        packet[11] = static_cast<uint8_t>(alive);         
        for(int i = 0 ; i < 14; ++i)
            packet_local[i] = packet[i];
        lock.unlock();
        alive = (alive + 1) % 256;
        /*
        ROS_INFO("alive  : %d", packet_local[11]);
        ROS_INFO("speed  : %d", packet_local[7]);
        ROS_INFO("mode   : %d", packet_local[3]);
        ROS_INFO("e-stop : %d", packet_local[4]);
        ROS_INFO("gear   : %d", packet_local[5]);
        ROS_INFO("steer  : %d", packet_local[8]);
        ROS_INFO("brake  : %d", packet_local[10]);
        *///debug
        ser->write(packet_local,TX_PACKET_LENGTH); //packet을 보내는 함수
        loop_rate.sleep();
    }
}

void createSerialPacket(const autodrive::autodrive::ConstPtr& msg){
/*  
    GEAR
    0x00 : forward
    0x01 : neutral
    0x02 : backward
*/  
    float angle = msg->goto_radian; //   checkSteeringBound(angle);
    uint16_t serialspeed = 30; // speed 
    int16_t serialSteeringAngle = 0;

    if(msg->goto_radian == 0.0){serialSteeringAngle = 0;}
    else if(msg->goto_radian >= -5.0 && msg->goto_radian <= 5.0){serialSteeringAngle = angle * 103.9 * 0.25;}
    else { serialSteeringAngle = angle * 103.9 * 0.3;}

    lock.lock();
    if(fabs(angle) >= 40.0){
       packet[5] = static_cast<uint8_t>(1);
       ROS_INFO("E-STOP!!");
    }
    packet[5] = static_cast<uint8_t>(0); //gear;
    *(uint8_t*)(packet + 6) = *((uint8_t*)(&serialspeed) + 1);
    *(uint8_t*)(packet + 7) = *(uint8_t*)(&serialspeed);
    *(int8_t*)(packet + 8) = *((int8_t*)(&serialSteeringAngle) + 1);
    *(int8_t*)(packet + 9) = *(int8_t*)(&serialSteeringAngle);
    packet[10] = static_cast<uint8_t>(1);
    lock.unlock();
}

void RX_CB(const platform_rx_msg::platform_rx_msg::ConstPtr& msg){
    curr_speed = msg->speedfilter;

}

//14byte packet = [S][T][X][MODE][E-STOP][GEAR][SPEED0][SPEED1][STEER0][STEER1][BRAKE][ALIVE][ETX0][ETX1]

int main(int argc, char *argv[]){
    if(argc < 2){
        ROS_ERROR("give me [path]"); //rosrun ~~ ~~ /dev/ttyUSB0
        return -1;
    }
//open serial=============================================================================================
    ser = new serial::Serial(); // serial::Serial* ser = new serial::Serial();
    ser->setPort(argv[1]);
    ser->setBaudrate(115200); // platform 기본 시리얼 설정
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // platform 기본 시리얼 설정
    ser->setTimeout(to); // platform 기본 시리얼 설정
    ser->open(); //serial 연결
    if(!ser->isOpen()) throw serial::IOException("ser.isOpen() error!",__LINE__,"ser.isOpen() error!");
    ROS_INFO("serial setting done");
//========================================================================================================
    autodrive::autodrive msg;
    ros::init(argc, argv, "autodrive_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("opencv_msg", 1000, &createSerialPacket);
    ros::Subscriber subRX = nh.subscribe("raw/platform_rx", 1000, )
    initTx(nh);
    
    std::thread tr(serialWrite);
    tr.detach();
    ros::spin();
}