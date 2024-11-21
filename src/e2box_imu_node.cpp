/*
 * Copyright 2024 Myeong Jin Lee
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "../include/e2box_imu/e2box_imu_node.hpp"

pthread_t thread_serial;
pthread_t tSerialThread;

namespace e2box_imu
{

E2BoxIMUNode::E2BoxIMUNode() : Node("e2box_imu"), is_shutting_down_(false)
{
  RCLCPP_INFO(this->get_logger(), "E2BoxIMUNode Constructed.");

  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<int>("loop_rate", 100);
  this->declare_parameter<double>("angular_velocity_threshold", 0.3);
  this->declare_parameter<double>("linear_acceleration_threshold", 2.0);

  this->get_parameter("port_name", port_name);
  this->get_parameter("baudrate", baudrate);
  int loop_rate;
  this->get_parameter("loop_rate", loop_rate);
  this->get_parameter("angular_velocity_threshold", angular_velocity_threshold);
  this->get_parameter("linear_acceleration_threshold", linear_acceleration_threshold);

  if (!serial_manager_.openSerial(port_name.c_str(), baudrate)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
    exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(this->get_logger(), "Serial port %s is opened.", port_name.c_str());
  RCLCPP_INFO(this->get_logger(), "Baudrate is set to %d.", baudrate);
  RCLCPP_INFO(this->get_logger(), "Loop rate is set to %d.", loop_rate);
  RCLCPP_INFO(
    this->get_logger(), "Angular velocity threshold is set to %f.", angular_velocity_threshold);
  RCLCPP_INFO(
    this->get_logger(), "Linear acceleration threshold is set to %f.",
    linear_acceleration_threshold);

  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().best_effort());

  auto period = std::chrono::milliseconds(1000 / loop_rate);
  timer_ = this->create_wall_timer(period, std::bind(&E2BoxIMUNode::timerCallback, this));

  initialize();

  setCallback();
}

E2BoxIMUNode::~E2BoxIMUNode()
{
  is_shutting_down_ = true;
  RCLCPP_INFO(this->get_logger(), "E2BoxIMUNode Destructed.");
}

void E2BoxIMUNode::initialize()
{
  m_dwordCounterCheckSumPass = 0;
  m_dwordCounterCheckSumFail = 0;

  memset(m_dQuaternion, 0, sizeof(m_dQuaternion));
  memset(m_dAngleRate, 0, sizeof(m_dAngleRate));
  memset(m_dAccel, 0, sizeof(m_dAccel));

  setStatusHeaderDetect(false);
  data_acquisision = false;
  setStatusUpdateData(false);

  m_iRawDataIndex = 0;
}

void E2BoxIMUNode::extractData(byte byte_data)
{
  if (!getStatusHeaderDetect()) {
    if (byte_data == 0x2A) {
      setStatusHeaderDetect(true);
      memset(m_abyteRawData, 0, sizeof(m_abyteRawData));
      m_abyteRawData[0] = byte_data;
      m_iRawDataIndex = 1;
    }
  }

  else {
    m_abyteRawData[m_iRawDataIndex++] = byte_data;
    if (byte_data == 0x0D) {
      for (int i = 0; i < m_iRawDataIndex; i++) {
        m_acCopiedRawData[i] = m_abyteRawData[i];
      }
      data_acquisision = true;
      setStatusHeaderDetect(false);
    }
  }
}

void E2BoxIMUNode::interpretGeneral()
{
  char c_branch;
  sscanf(&m_acCopiedRawData[0], "%c", &c_branch);
  if (c_branch == '*') {
    sscanf(
      &m_acCopiedRawData[1], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &m_dQuaternion[0],
      &m_dQuaternion[1], &m_dQuaternion[2], &m_dQuaternion[3], &m_dAngleRate[0], &m_dAngleRate[1],
      &m_dAngleRate[2], &m_dAccel[0], &m_dAccel[1], &m_dAccel[2]);
  }

  setStatusUpdateData(true);
}

bool E2BoxIMUNode::handleRawIMUData()
{
  if (getStatusHeaderDetect() == false) {
    interpretGeneral();
    setStatusUpdateData(true);
    m_dwordCounterCheckSumPass++;
  } else {
    m_dwordCounterCheckSumFail++;
    return false;
  }
  return true;
}

bool E2BoxIMUNode::calculateChecksum()
{
  sscanf(&m_acCopiedRawData[m_iRawDataIndex - 3], "%x", &i_checksum);

  for (int i = 0; i < m_iRawDataIndex - 5; i++) {
    if (i == 1) {
      i_candisum = checkByteError(m_acCopiedRawData[i], m_acCopiedRawData[i + 1]);
    } else {
      i_candisum = checkByteError(i_candisum, m_acCopiedRawData[i]);
    }
  }

  if (i_checksum == i_candisum) {
    return true;
  }
  return false;
}

DWORD E2BoxIMUNode::checkByteError(byte byte_cpr1, byte byte_cpr2)
{
  byte byte_mask = 0x80;
  byte byte_result = 0;

  for (int i = 0; i < 8; i++) {
    byte_result <<= 1;

    if ((byte_cpr1 & byte_mask) == (byte_cpr2 & byte_mask)) {
      byte_result |= 0x0;
    } else {
      byte_result |= 0x1;
    }
    byte_mask >>= 1;
  }
  return byte_result;
}

void E2BoxIMUNode::timerCallback()
{
  onReceiveImu();
  publishIMUData();
}

void E2BoxIMUNode::onReceiveImu()
{
  if (!serial_manager_.getStatusConnected() && !is_shutting_down_) {
    RCLCPP_ERROR(this->get_logger(), "Serial connection lost. Shutting down node.");
    rclcpp::shutdown();
    return;
  }

  int n = serial_manager_.getLength();
  byte * buffer = serial_manager_.getBuffer();

  if (n >= 10) {
    for (int i = 0; i < n; i++) {
      extractData(buffer[i]);
      if (data_acquisision) {
        serial_manager_.resetBuffer();
        handleRawIMUData();
        data_acquisision = false;
        break;
      }
    }
  }
}

void E2BoxIMUNode::publishIMUData()
{
  if (!data_acquisision) {
    imu_msg_.header.stamp = this->now();
    imu_msg_.header.frame_id = "imu_link";

    imu_msg_.orientation.x = m_dQuaternion[2];
    imu_msg_.orientation.y = m_dQuaternion[1];
    imu_msg_.orientation.z = m_dQuaternion[0];
    imu_msg_.orientation.w = m_dQuaternion[3];

    imu_msg_temp_.angular_velocity.x = m_dAngleRate[0] * M_PI / 180.0;
    imu_msg_temp_.angular_velocity.y = m_dAngleRate[1] * M_PI / 180.0;
    imu_msg_temp_.angular_velocity.z = m_dAngleRate[2] * M_PI / 180.0;

    imu_msg_temp_.linear_acceleration.x = m_dAccel[0] * 9.80665;
    imu_msg_temp_.linear_acceleration.y = m_dAccel[1] * 9.80665;
    imu_msg_temp_.linear_acceleration.z = m_dAccel[2] * 9.80665;

    imu_msg_prev_.angular_velocity.x = imu_msg_temp_.angular_velocity.x;
    imu_msg_prev_.angular_velocity.y = imu_msg_temp_.angular_velocity.y;
    imu_msg_prev_.angular_velocity.z = imu_msg_temp_.angular_velocity.z;

    imu_msg_prev_.linear_acceleration.x = imu_msg_temp_.linear_acceleration.x;
    imu_msg_prev_.linear_acceleration.y = imu_msg_temp_.linear_acceleration.y;
    imu_msg_prev_.linear_acceleration.z = imu_msg_temp_.linear_acceleration.z;

    gap_ang_vel_x = fabs(imu_msg_prev_.angular_velocity.x - imu_msg_temp_.angular_velocity.x) / 2.0;
    if (gap_ang_vel_x > angular_velocity_threshold) {
      imu_msg_temp_.angular_velocity.x = imu_msg_prev_.angular_velocity.x;
    }
    gap_ang_vel_y = fabs(imu_msg_prev_.angular_velocity.y - imu_msg_temp_.angular_velocity.y) / 2.0;
    if (gap_ang_vel_y > angular_velocity_threshold) {
      imu_msg_temp_.angular_velocity.y = imu_msg_prev_.angular_velocity.y;
    }
    gap_ang_vel_z = fabs(imu_msg_prev_.angular_velocity.z - imu_msg_temp_.angular_velocity.z) / 2.0;
    if (gap_ang_vel_z > angular_velocity_threshold) {
      imu_msg_temp_.angular_velocity.z = imu_msg_prev_.angular_velocity.z;
    }

    imu_msg_.angular_velocity.x =
      (imu_msg_temp_.angular_velocity.x + imu_msg_prev_.angular_velocity.x) / 2.0;
    imu_msg_.angular_velocity.y =
      (imu_msg_temp_.angular_velocity.y + imu_msg_prev_.angular_velocity.y) / 2.0;
    imu_msg_.angular_velocity.z =
      (imu_msg_temp_.angular_velocity.z + imu_msg_prev_.angular_velocity.z) / 2.0;

    gap_acc_x =
      fabs(imu_msg_prev_.linear_acceleration.x - imu_msg_temp_.linear_acceleration.x) / 2.0;
    if (gap_acc_x > linear_acceleration_threshold) {
      imu_msg_temp_.linear_acceleration.x = imu_msg_prev_.linear_acceleration.x;
    }
    gap_acc_y =
      fabs(imu_msg_prev_.linear_acceleration.y - imu_msg_temp_.linear_acceleration.y) / 2.0;
    if (gap_acc_y > linear_acceleration_threshold) {
      imu_msg_temp_.linear_acceleration.y = imu_msg_prev_.linear_acceleration.y;
    }
    gap_acc_z =
      fabs(imu_msg_prev_.linear_acceleration.z - imu_msg_temp_.linear_acceleration.z) / 2.0;
    if (gap_acc_z > linear_acceleration_threshold) {
      imu_msg_temp_.linear_acceleration.z = imu_msg_prev_.linear_acceleration.z;
    }

    imu_msg_.linear_acceleration.x =
      (imu_msg_temp_.linear_acceleration.x + imu_msg_prev_.linear_acceleration.x) / 2.0;
    imu_msg_.linear_acceleration.y =
      (imu_msg_temp_.linear_acceleration.y + imu_msg_prev_.linear_acceleration.y) / 2.0;
    imu_msg_.linear_acceleration.z =
      (imu_msg_temp_.linear_acceleration.z + imu_msg_prev_.linear_acceleration.z) / 2.0;

    imu_publisher_->publish(imu_msg_);
  }
}

void E2BoxIMUNode::setCallback()
{
  serial_manager_.setCallback(
    [](void * arg) {
      auto node = static_cast<E2BoxIMUNode *>(arg);
      if (!node->serial_manager_.getStatusConnected() && !node->is_shutting_down_) {
        RCLCPP_ERROR(node->get_logger(), "Serial connection lost. Shutting down node.");
        rclcpp::shutdown();
      }
    },
    this);
}

}  // namespace e2box_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<e2box_imu::E2BoxIMUNode>());
  rclcpp::shutdown();
  return 0;
}