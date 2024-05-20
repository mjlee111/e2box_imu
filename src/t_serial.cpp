#include "../include/t_serial.h"

t_serial::t_serial() 
{
    serialPortFd = -1;
    isConnected = false;
    data_length = 0;
}

t_serial::~t_serial() {
    if (isConnected) {
        portClose();
    }
}

bool t_serial::portOpen(char* device, const int baudrate) {
    serialPortFd = open(reinterpret_cast<const char*>(device), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPortFd == -1) {
        std::cerr << "Failed to open port : " << device << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(serialPortFd, &options);

    speed_t baud;
    switch (baudrate) {
        case 9600:
            baud = B9600;
            break;
        case 19200:
            baud = B19200;
            break;
        case 38400:
            baud = B38400;
            break;
        case 57600:
            baud = B57600;
            break;
        case 115200:
            baud = B115200;
            break;
        default:
            std::cerr << "Unsupported baud rate" << std::endl;
            portClose();
            return false;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(serialPortFd, TCSANOW, &options);
    isConnected = true;
    return true;
}

void t_serial::portClose() {
    if (isConnected) {
        close(serialPortFd);
        serialPortFd = -1;
        isConnected = false;
    }
}

void t_serial::deviceWrite(unsigned char data) {
    if (isConnected) {
        int bytesWritten = write(serialPortFd, &data, 1);
        if (bytesWritten < 0) {
            std::cerr << "Failed to write to the port" << std::endl;
        }
    }
}

void t_serial::deviceRead() {
    if (isConnected) {
        memset(serialData, 0, sizeof(serialData));
        data_length = read(serialPortFd, serialData, sizeof(serialData));
        if (data_length < 0) {
            std::cerr << "Failed to read from the port" << std::endl;
        } 
    }
}

void t_serial::deviceReset() {
    if (isConnected) {
        tcflush(serialPortFd, TCIOFLUSH);
    }
}

int t_serial::getLength()
{
    return data_length;
}

unsigned char* t_serial::getData()
{
    return serialData;
}