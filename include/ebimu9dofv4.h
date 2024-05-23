#ifndef EBIMU9DOFV4_H
#define EBIMU9DOFV4_H

#define MAX_PACKET_SIZE 100

#include "../include/t_serial.h"
#include <iostream>

struct IMU {
  float orientation[9];
  float angular_velocity[9];
  float linear_acceleration[9];
};

class ebimu {
public:
  ebimu(std::string device, int baudrate);
  ~ebimu();

private:
  void generatePacket(char *packet, const char *string, int data) {
    sprintf((char *)packet, "<%s%d>", string, data);
  }
  IMU *imuData;
  t_serial ser;

public:
  unsigned char *readResponse(void);

  void writePacket(char *data);
  /*
   ** OUTPUT COMMAND **
   */
  // Set baudrate. Default : 5(115200)
  /* 1 : 9600, 2 : 19200, 3 : 38400, 4 : 57600, 5 : 115200, 6 : 230400, 7 :
   * 460800, 8 : 921600 */
  void setBaudRate(int baudrate);

  // Set output rate. Default : 10
  void setOutputRate(int rate); // 1 ~ 1000, 0 : polling mode [ms * data]

  // Set output data type. Default : 1(ASCII)
  // 1 : ASCII, 2 : HEX
  void setOutputType(int type);
  void setOutputType(std::string type);

  // Set output data format. Default : 1(Euler angle)
  // 1 : Euler, 2 : Quaternion
  void setOutputFormat(int format);
  void setOutputFormat(std::string format);

  // Set gyro data output. Default : 0(non-gyro)
  // 0 : non-gyro, 1 : gyro
  void setOutputGyro(bool gyro);

  // Set accelero data output. Default : 0(non-acc)
  /* 0 : non-acc, 1 : acc, 2 : local gravity removed acc, 3 : global gravity
   * removed acc, 4 : local spd, 5 : global spd*/
  void setOutputAccelero(int acc);

  // Set megneto data output. Default : 0(non-megneto)
  // 0 : non-meg, 1 : meg
  void setOutputMegneto(bool meg);

  // Set distance data output. Default : 0(non-distance)
  // 0 : non-distance, 1 : local distance, 2 : global distance
  void setOutputDistance(int dis);

  // Set temperature data output. Default : 0(non-temperature)
  // 0 : non-temperature, 1 : temperature
  void setOutputTemperature(bool temp);

  /*
   ** SENSOR COMMAND **
   */

  // Set enable magnetometer. Default : 1(Magnetometer ON)
  // 0 : Magnetometer OFF, 1 : Magnetometer ON
  void setEnableMagneto(bool mag);

  // Set gyro sensor sensitivity. Default : 4(2000dps)
  // 1 : 250dps, 2 : 500dps, 3 : 1000dps, 4 : 2000dps
  void setGyroSensitivity(int dps);

  // Set low pass filter. Default : 5(92Hz)
  /* 0 : No LPF,
         1 : 5Hz,
         2 : 10Hz,
         3 : 20Hz,
         4 : 41Hz,
         5 : 92Hz,
         6 : 184Hz,
         7 : 250Hz */
  void setLPF(int lpf);

  // Set filter factor. Default : 10
  // 1 ~ 50
  void setFilterFactor(int fac);

  /* Set robust heading algorithm parameters. Default : RHA level - 0.1, RHA
   * timeout - 10000*/
  // 0.00 ~ 100.00 / 0 ~ 4000000000
  void setRHAParameter(float level, float timeout);

  /* Set auto gyroscope calibration parameters. Default : AGC enable - 1, AGC
   * threshold - 0.6, AGC drift - 0.5*/
  // 0, 1 / 0.00 ~ 100.00 / 0.00 ~ 10.00
  void setAGCParameter(bool agc, float threshold, float drift);

  // Set posiiton filter parameters. Default : 0.01, 50, 0.001, 0.9
  // 0.0000 ~ 1.0000 / 0 ~ 1000 / 0.0000 ~ 1.0000 / 0.0000 ~ 1.0000
  void setPosfParameter(float p1, float p2, float p3, float p4);

  /*
   ** CALIBRATION COMMAND **
   */
  void setPositionZero(void);            // Set current position to zero.
  void gyroSensorCalibration(void);      // Calibrate gyro sensor.
  void accSensorFullCalibration(void);   // Fully calibrate accelero sensor.
  void accSensorSimpleCalibration(void); // Simply calibrate accelero sensor.
  void magSensorXYCalibration(void);     // Calibrate magneto sensor XY.
  void magSensorZCalibration(void);      // Calibrate magneto sensor Z.
  void setMotionOffset(void);            // Motion offset set.
  void cloearMotionOffset(void);         // Motion offset clear.

  /*
   ** ETC COMMAND **
   */
  void getConfiguration(void); // Get sensor settings.
  // Set start sensor on power. Default : 0(false)
  // 0 : false, 1 : true
  void setPowerOnStart(bool power);
  void startIMU(void);            // Start ebimu.
  void stopIMU(void);             // Stop ebimu.
  void loadFactorySettings(void); // Load factory settings.
  void resetIMU(void);            // Reset ebimu.
  void versionCheck(void);        // Check sensor version.
};

#endif
