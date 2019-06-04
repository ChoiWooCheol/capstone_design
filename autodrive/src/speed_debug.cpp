#include <ros/ros.h>
#include <serial/serial.h>
#include <deque>
#include <thread>
#include "autodrive/autodrive.h"
#include <mutex>

#define TX_PACKET_LENGTH 14
#define TX_SERIAL_FREQUENCY 10
#define TX_STOP_CHECK_PERIOD 10

serial::Serial *ser;
std::mutex lock;

uint8_t packet[TX_PACKET_LENGTH] = {};

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
    while(true){
        packet[11] = static_cast<uint8_t>(alive); 
        alive = (alive + 1) % 256; // 0~255 alive 값을 보내주기 위한 식. 한번 write할때마다 + 1
        
        ROS_INFO("alive  : %d", packet[11]);
        ROS_INFO("speed  : %d", packet[6]);
        ROS_INFO("mode   : %d", packet[3]);
        ROS_INFO("e-stop : %d", packet[4]);
        ROS_INFO("gear   : %d", packet[5]);
        ROS_INFO("steer  : %d", packet[8]);
        ROS_INFO("brake  : %d", packet[10]);
        ser->write(packet,TX_PACKET_LENGTH); //packet을 보내는 함수
        loop_rate.sleep();
    }
}

void createSerialPacket(){
    packet[5] = static_cast<uint8_t>(0); //gear;
    uint16_t serialspeed = 30; // speed 
    *(uint8_t*)(packet + 6) = *((uint8_t*)(&serialspeed) + 1);
    *(uint8_t*)(packet + 7) = *(uint8_t*)(&serialspeed); //big엔디안으로 넣어줘야함.
//    ROS_INFO("serial speed : %d", serialspeed);
    int16_t serialSteeringAngle = 0;
    *(int8_t*)(packet + 8) = *((int8_t*)(&serialSteeringAngle) + 1);
    *(int8_t*)(packet + 9) = *(int8_t*)(&serialSteeringAngle);
//    ROS_INFO("serial angle : %d", serialSteeringAngle);
    packet[10] = static_cast<uint8_t>(1);
//    ROS_INFO("serial brake : %u", 1);
}

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
    ros::init(argc, argv, "speed_debug");
    ros::NodeHandle nh;
    initTx(nh);

    std::thread tr(serialWrite);
    tr.detach();
    while(ros::ok()){
        createSerialPacket();
    }
    //5hz->9.75
    //10hz->9.95
    //15hz->9.56
    //20hz->9.68
    //50hz->9.02

}