#include "../include/ebimu9dofv4.h"

ebimu::ebimu(std::string device, const int baudrate) {
  if (!ser.portOpen(const_cast<char *>(device.c_str()), baudrate)) {
    std::cerr << " Failed to open device : " << device << std::endl;
    return;
  } else {
    std::cout << " Succeded to open device : " << device << std::endl;
  }
  for (int i = 0; i < 9; i++) {
    imuData->orientation[i] = 0;
    imuData->angular_velocity[i] = 0;
    imuData->linear_acceleration[i] = 0;
  }
}

ebimu::~ebimu() { delete imuData; }

unsigned char *ebimu::readResponse(void) {
  unsigned char *data;
  data = ser.getData();
  return data;
}

void ebimu::writePacket(char *data) {
  ser.deviceWrite(data, strlen(data));
  readResponse();
}
