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

#include "../include/e2box_imu/serial_manager.hpp"

namespace e2box_imu
{

SerialManager::SerialManager()
: fd(-1), isConnected(false), n_length(0), pCallback(NULL), pCallbackArg(NULL)
{
  std::cout << "SerialManager Constructed." << std::endl;
}

SerialManager::~SerialManager()
{
  std::cout << "SerialManager Destructed." << std::endl;
  closeSerial();
}

void * serialThread(void * p)
{
  SerialManager * pThis = (SerialManager *)p;
  if (pThis->getStatusConnected()) {
    pThis->readSerial();
  }

  pthread_exit(NULL);
}

bool SerialManager::openSerial(const char * port, int baudrate)
{
  memset(&newtio, 0, sizeof(newtio));

  fd = open(port, O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY, S_IRUSR | S_IWUSR);
  if (fd == -1) {
    std::cerr << "Failed to open serial port: " << port << std::endl;
    return false;
  }

  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_cflag = CS8 | CLOCAL | CREAD;

  if (baudrate == -1) {
    baudrate = 115200;
  }

  switch (baudrate) {
    case 115200:
      newtio.c_cflag |= B115200;
      break;
    case 57600:
      newtio.c_cflag |= B57600;
      break;
    case 38400:
      newtio.c_cflag |= B38400;
      break;
    case 19200:
      newtio.c_cflag |= B19200;
      break;
    case 9600:
      newtio.c_cflag |= B9600;
      break;
    case 4800:
      newtio.c_cflag |= B4800;
      break;
    case 2400:
      newtio.c_cflag |= B2400;
      break;
    default:
      newtio.c_cflag |= B115200;
      break;
  }

  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 1;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  isConnected = true;
  pthread_create(&thread_serial, NULL, serialThread, this);

  return true;
}

void SerialManager::closeSerial()
{
  if (fd != -1) {
    isConnected = false;
    pthread_cancel(thread_serial);
    pthread_join(thread_serial, NULL);
    close(fd);
    fd = -1;
    std::cout << "Serial port closed." << std::endl;
  }
}

void SerialManager::writeSerial(byte * data, int length)
{
  if (!isConnected) {
    std::cerr << "Serial port is not connected." << std::endl;
    return;
  }

  write(fd, data, length);
}

void SerialManager::writeEbSerial(byte * data)
{
  if (!isConnected) {
    std::cerr << "Serial port is not connected." << std::endl;
    return;
  }

  write(fd, data, 1);
}

void SerialManager::readSerial()
{
  fd_set fc;
  char ch[2] = {' ', '\0'};
  struct timeval timeout;
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;

  while (isConnected) {
    FD_ZERO(&fc);
    FD_SET(fd, &fc);

    int select_result = select(fd + 1, &fc, NULL, NULL, &timeout);

    if (select_result == -1) {
      isConnected = false;
      if (pCallback != NULL) {
        pCallback(pCallbackArg);
      }
      return;
    } else if (select_result == 0) {
      isConnected = false;
      if (pCallback != NULL) {
        pCallback(pCallbackArg);
      }
      return;
    }

    if (FD_ISSET(fd, &fc)) {
      int rc = read(fd, ch, 1);
      if (rc < 0) {
        isConnected = false;
        if (pCallback != NULL) {
          pCallback(pCallbackArg);
        }
        return;
      }
      pBuffer[n_length++] = ch[0];
      if (n_length >= MAX_SERIAL_BUFFER_SIZE) {
        n_length = 0;
      }
      if (pCallback != NULL) {
        pCallback(pCallbackArg);
      }
    }
  }
}

}  // namespace e2box_imu
