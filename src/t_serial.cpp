#include "../include/t_serial.h"

t_serial::t_serial() {
  serialPortFd = -1;
  isConnected = false;
  data_length = 0;
}

t_serial::~t_serial() {
  if (isConnected) {
    portClose();
  }
}

bool t_serial::portOpen(char *device, const int baudrate) {
  memset(&newtermios, 0, sizeof(newtermios));
  serialPortFd = open(device, O_RDWR | O_NOCTTY, S_IRUSR | S_IWUSR);

  if (serialPortFd == -1) {
    std::cerr << "Failed to open port : " << device << " - " << strerror(errno)
              << std::endl;
    return false;
  }

  newtermios.c_iflag = IGNPAR;
  newtermios.c_oflag = 0;
  newtermios.c_cflag = CS8 | CLOCAL | CREAD;

  switch (baudrate) {
  case 9600:
    newtermios.c_cflag |= B9600;
    break;
  case 19200:
    newtermios.c_cflag |= B19200;
    break;
  case 38400:
    newtermios.c_cflag |= B38400;
    break;
  case 57600:
    newtermios.c_cflag |= B57600;
    break;
  case 115200:
    newtermios.c_cflag |= B115200;
    break;
  default:
    std::cerr << "Unsupported baud rate" << std::endl;
    portClose();
    return false;
  }

  newtermios.c_lflag = 0;
  newtermios.c_cc[VTIME] = 0;
  newtermios.c_cc[VMIN] = 1;

  tcflush(serialPortFd, TCIFLUSH);
  tcsetattr(serialPortFd, TCSANOW, &newtermios);

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

int t_serial::getLength() { return data_length; }

unsigned char *t_serial::getData() { return serialData; }