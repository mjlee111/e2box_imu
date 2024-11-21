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

#ifndef E2BOX_IMU_SERIAL_MANAGER_HPP
#define E2BOX_IMU_SERIAL_MANAGER_HPP

#include <fcntl.h>
#include <linux/serial.h>
#include <pthread.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <functional>
#include <iostream>
#include <string>

typedef unsigned long DWORD;
typedef unsigned short WORD;
typedef unsigned char byte;
typedef std::string CString;

#define MAX_SERIAL_BUFFER_SIZE 4096

extern pthread_t thread_serial;
extern pthread_t tSerialThread;

namespace e2box_imu
{
class SerialManager
{
public:
  SerialManager();
  virtual ~SerialManager();

private:  // Private Variables
  int fd;
  int n_length;
  bool isConnected;

public:  // Public Variables
  byte pBuffer[MAX_SERIAL_BUFFER_SIZE];
  struct termios newtio;

  void (*pCallback)(void *);
  void * pCallbackArg;

public:  // Public Functions
  byte * getBuffer() { return pBuffer; }
  int getLength() { return n_length; }

  bool openSerial(const char * port, int baudrate);
  void closeSerial();
  void writeSerial(byte * data, int length);
  void writeEbSerial(byte * data);
  void readSerial();
  void resetBuffer() { n_length = 0; }

  bool getStatusConnected() { return isConnected; }

  void setCallback(void (*callback)(void *), void * arg)
  {
    pCallback = callback;
    pCallbackArg = arg;
  }
};
}  // namespace e2box_imu

#endif  // E2BOX_IMU_SERIAL_MANAGER_HPP
