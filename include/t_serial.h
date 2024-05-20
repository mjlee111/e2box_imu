#ifndef T_SERIAL_H
#define T_SERIAL_H

#include <iostream>
#include <linux/serial.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

#define MAX_BUFFER_SIZE 4096

class t_serial{
public:
    t_serial();
    virtual ~t_serial();

    // fuctions
    bool portOpen(char* device, const int baudrate);
    void portClose();
    void deviceWrite(unsigned char);
    void deviceRead();
    void deviceReset();

    // serial
    unsigned char serialData[MAX_BUFFER_SIZE];
    int getLength();
    unsigned char* getData();

private:
    int serialPortFd;
    bool isConnected;
    int data_length;
};

#endif // T_SERIAL_H