/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-29 16:04:05
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <Arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_Bootup(void);
  void ApplicationFunctionSet_RGB(void);
  void ApplicationFunctionSet_Expression(void);
  void ApplicationFunctionSet_Rocker(void);             //APP Rocker Control
  void ApplicationFunctionSet_Tracking(void);           //Line Tracking Mode
  void ApplicationFunctionSet_Obstacle(void);           //Obstacle Avoidance
  void ApplicationFunctionSet_Follow(void);             //Following Mode
  void ApplicationFunctionSet_Servo(uint8_t Set_Servo); //Servo Control
  void ApplicationFunctionSet_Standby(void);            //Standby Mode
  void ApplicationFunctionSet_KeyCommand(void);         //Mode Switch Button
  void ApplicationFunctionSet_SensorDataUpdate(void);   //Sensor Data Update
  void ApplicationFunctionSet_SerialPortDataAnalysis(void);
  void ApplicationFunctionSet_IRrecv(void);

public: /*CMD*/
  void CMD_UltrasoundModuleStatus_xxx0(uint8_t is_get);
  void CMD_TraceModuleStatus_xxx0(uint8_t is_get);
  void CMD_Car_LeaveTheGround_xxx0(uint8_t is_get);

  void CMD_inspect_xxx0(void);
  void CMD_MotorControl_xxx0(void);
  void CMD_MotorControl_xxx0(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed);
  void CMD_CarControlTimeLimit_xxx0(void);
  void CMD_CarControlTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint32_t is_Timer);
  void CMD_CarControlNoTimeLimit_xxx0(void);
  void CMD_CarControlNoTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed);
  void CMD_MotorControlSpeed_xxx0(void);
  void CMD_MotorControlSpeed_xxx0(uint8_t is_Speed_L, uint8_t is_Speed_R);
  void CMD_ServoControl_xxx0(void);
  void CMD_YawControl_xxx0(void);
  void CMD_VoiceControl_xxx0(uint16_t is_VoiceName, uint32_t is_VoiceTimer);
  void CMD_LightingControlTimeLimit_xxx0(void);
  void CMD_LightingControlTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B, uint32_t is_LightingTimer);
  void CMD_LightingControlNoTimeLimit_xxx0(void);
  void CMD_LightingControlNoTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B);
  void CMD_LEDCustomExpressionControl_xxx0(void);
  void CMD_ClearAllFunctions_xxx0(void);
  void CMD_LEDNumberDisplayControl_xxx0(uint8_t is_LEDNumber);
  void CMD_TrajectoryControl_xxx0(void);

private:
  /*Sensor Raw Value*/
  volatile float VoltageData_V;        //Battery Voltage Value
  volatile uint16_t UltrasoundData_mm; //Ultrasonic Sensor Value (mm)
  volatile uint16_t UltrasoundData_cm; //Ultrasonic Sensor Value (cm)
  volatile int TrackingData_L;         //Line Tracking Module Value (Left)
  volatile int TrackingData_M;         //Line Tracking Module Value (Middle)
  volatile int TrackingData_R;         //Line Tracking Module Value (Right)
  /*Sensor Status*/
  boolean VoltageDetectionStatus = false;
  boolean UltrasoundDetectionStatus = false;
  boolean TrackingDetectionStatus_R = false;
  boolean TrackingDetectionStatus_M = false;
  boolean TrackingDetectionStatus_L = false;

public:
  boolean Car_LeaveTheGround = true;

  /*Sensor Threshold Setting*/
  const float VoltageDetection = 7.00;
  const uint8_t ObstacleDetection = 20;

  String CommandSerialNumber;
  uint8_t Rocker_CarSpeed = 250;
  uint8_t Rocker_temp;

public:
  uint8_t TrackingDetection_S = 250;
  uint16_t TrackingDetection_E = 850;
  uint16_t TrackingDetection_V = 950;

public:
  uint8_t CMD_is_Servo;
  uint8_t CMD_is_Servo_angle;

public:
  uint8_t CMD_is_MotorSelection; //motor
  uint8_t CMD_is_MotorDirection;
  uint8_t CMD_is_MotorSpeed;
  uint32_t CMD_is_MotorTimer;

public:
  uint8_t CMD_is_CarDirection; //car
  uint8_t CMD_is_CarSpeed;
  uint32_t CMD_is_CarTimer;
  int CMD_is_Yaw;
  float Yaw, Yaw0, Yaw_prev;
  float VelX, VelY;  

public:
  uint8_t CMD_is_MotorSpeed_L; //motor
  uint8_t CMD_is_MotorSpeed_R;

public:
  uint8_t CMD_is_LightingSequence; //Lighting (Left, front, right, back and center)
  uint8_t CMD_is_LightingColorValue_R;
  uint8_t CMD_is_LightingColorValue_G;
  uint8_t CMD_is_LightingColorValue_B;
  uint32_t CMD_is_LightingTimer;

private:
  uint8_t CMD_is_FastLED_setBrightness = 5;
  // uint8_t Cnt = 0;
  // uint16_t Mot[200];
};
extern ApplicationFunctionSet Application_FunctionSet;
#endif
