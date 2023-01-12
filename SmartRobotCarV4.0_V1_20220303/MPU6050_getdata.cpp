/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-30 10:34:30
 * @LastEditors: Changhua
 * @Description: MPU6050 Data solution
 * @FilePath: 
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MPU6050_getdata.h"
#include <stdio.h>
#include <math.h>

MPU6050 accelgyro;
MPU6050_getdata MPU6050Getdata;

// static void MsTimer2_MPU6050getdata(void)
// {
//   sei();
//   int16_t ax, ay, az, gx, gy, gz;
//   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Read the raw values of the six axes
//   float gyroz = -(gz - MPU6050Getdata.gzo) / 131 * 0.005f;
//   MPU6050Getdata.yaw += gyroz;
// }

bool MPU6050_getdata::MPU6050_dveInit(void)
{
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout;
  do
  {
    chip_id = accelgyro.getDeviceID();
    Serial.print("MPU6050_chip_id: ");
    Serial.println(chip_id);
    delay(10);
    cout += 1;
    if (cout > 10)
    {
      return true;
    }
  } while (chip_id == 0X00 || chip_id == 0XFF); //Ensure that the slave device is online（Wait forcibly to get the ID）
  accelgyro.initialize();
  // unsigned short times = 100; //Sampling times
  // for (int i = 0; i < times; i++)
  // {
  //   gz = accelgyro.getRotationZ();
  //   gzo += gz;
  // }
  // gzo /= times; //Calculate gyroscope offset
  return false;
}
bool MPU6050_getdata::MPU6050_calibration(void)
{
  int16_t gz;
  int16_t ax, ay;
  // int16_t tmp;
  unsigned short times = 100; //Sampling times
  for (int i = 0; i < times; i++)
  {
    gz = accelgyro.getRotationZ();
    ax = accelgyro.getAccelerationX();
    ay = accelgyro.getAccelerationY();
    // accelgyro.getMotion6(&ax, &ay, &tmp, &tmp, &tmp, &gz); //Read the raw values of the six axes
    gzo += gz;
    axo += ax;
    ayo += ay;
  }
  gzo /= times; //Calculate gyro offset
  axo /= times; //Calculate accX offset
  ayo /= times; //Calculate accY offset
  return false;
}
//bool MPU6050_getdata::MPU6050_dveGetEulerAngles(float *VelX, float *VelY, float *Yaw)
bool MPU6050_getdata::MPU6050_data(float *VelX, float *VelY, float *Yaw)
//bool MPU6050_getdata::MPU6050_data(float *Yaw)
{
  int16_t gz;
  int16_t ax, ay;
  // int16_t tmp;
  unsigned long now = millis();           //Record the current time(ms)
  dt = (now - lastTime) / 1000.0;         //Caculate the derivative time(s)
  lastTime = now;                         //Record the last sampling time(ms)
  gz = accelgyro.getRotationZ();          //Read the raw values of the six axes
  ax = accelgyro.getAccelerationX();
  ay = accelgyro.getAccelerationY();
  // accelgyro.getMotion6(&ax, &ay, &tmp, &tmp, &tmp, &gz); //Read the raw values of the six axes
  float gyroz = -(gz - gzo) / 131.0 * dt; //z-axis angular velocity
  if (fabs(gyroz) < 0.05)                 //Clear instant zero drift signal
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z-axis angular velocity integral
  *Yaw = agz;
  float velox =  (ax - axo) * 0.059855 * dt; //x-axis linear velocity in cm/s
  float veloy =  (ay - ayo) * 0.059855 * dt; //y-axis linear velocity in cm/s
  vx += velox;  // accX integral
  vy += veloy;  // accY integral
  *VelX = vx;
  *VelY = vy;
  return false;
}
