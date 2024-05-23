#ifndef T_SERIAL_H
#define T_SERIAL_H

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/serial.h>
#include <mutex>
#include <termios.h>
#include <unistd.h>

#define MAX_BUFFER_SIZE 4096

class t_serial {
public:
  t_serial();
  virtual ~t_serial();

  // fuctions
  bool portOpen(char *device, const int baudrate);
  void portClose();
  bool deviceWrite(char *data, size_t size);
  void deviceRead();
  void deviceReset();

  // serial
  struct termios newtermios;
  unsigned char serialData[MAX_BUFFER_SIZE];
  int getLength();
  unsigned char *getData();

private:
  std::mutex rx_mut;
  int serialPortFd;
  bool isConnected;
  int data_length;
};

#endif // T_SERIAL_H