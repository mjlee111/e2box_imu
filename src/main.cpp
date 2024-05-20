#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include<sensor_msgs/Imu.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include "../include/t_serial.h"

t_serial serialInstance;

ros::Publisher imu_data_publisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "e2box_imu");
    ros::NodeHandle nh;

    std::string device, topic;
    int baudrate;

    nh.param<std::string>("device", device, "/dev/serial");
    nh.param("baudrate", baudrate, 115200);
    nh.param<std::string>("topic", topic, "imu/data_raw");

    if(!serialInstance.portOpen(const_cast<char*>(device.c_str()), baudrate))
    {
        ROS_ERROR("Failed to open device : %s", device.c_str());
        return 0;
    }

    imu_data_publisher = nh.advertise<sensor_msgs::Imu>(topic, 100);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        serialInstance.deviceRead();
        std::cout << serialInstance.getData() << std::endl;
    }
    
    return 0;
}