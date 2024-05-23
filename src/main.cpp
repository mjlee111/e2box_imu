#include "../include/ebimu9dofv4.h"
#include "../include/t_serial.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <vector>

std::string device, topic, frame;
int baudrate;

ros::Publisher imu_pub;

float serialDataFloat[10];

void extractFloatData(const unsigned char *data, float *floatArray) {
  char *temp = strdup((const char *)data + 1);
  if (!temp) {
    printf("Memory allocation failed!\n");
    return;
  }

  char *token;
  int index = 0;
  token = strtok(temp, ",");
  while (token != NULL && index < 10) {
    floatArray[index] = atof(token);
    token = strtok(NULL, ",");
    index++;
  }

  free(temp);
}

void handleIMUData(unsigned char *data) {
  sensor_msgs::Imu imu_msg_;
  // Parsing raw data
  float floatArray[10];
  extractFloatData(data, floatArray);

  imu_msg_.header.stamp = ros::Time::now();
  imu_msg_.header.frame_id = frame;
  imu_msg_.orientation.x = floatArray[0];
  imu_msg_.orientation.y = floatArray[1];
  imu_msg_.orientation.z = floatArray[2];
  imu_msg_.orientation.w = floatArray[3];

  imu_msg_.angular_velocity.x = floatArray[4];
  imu_msg_.angular_velocity.y = floatArray[5];
  imu_msg_.angular_velocity.z = floatArray[6];

  imu_msg_.linear_acceleration.x = floatArray[7];
  imu_msg_.linear_acceleration.y = floatArray[8];
  imu_msg_.linear_acceleration.z = floatArray[9];

  // Publish IMU data
  imu_pub.publish(imu_msg_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "e2box_imu");
  ros::NodeHandle nh;

  ros::param::get("/e2box_imu_node/device", device);
  std::cout << " Trying to open device : " << device << std::endl;

  ros::param::get("/e2box_imu_node/baudrate", baudrate);
  std::cout << " Baudrate : " << baudrate << std::endl;

  ros::param::get("/e2box_imu_node/topic", topic);
  std::cout << " Publishing data as : " << topic << std::endl;

  ros::param::get("/e2box_imu_node/frame", frame);
  std::cout << " Data header frame : " << frame << std::endl;

  imu_pub = nh.advertise<sensor_msgs::Imu>(topic, 100);

  ros::Rate loop_rate(10);
  ebimu imu(device, baudrate);

  while (ros::ok()) {
    handleIMUData(imu.readResponse());
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
