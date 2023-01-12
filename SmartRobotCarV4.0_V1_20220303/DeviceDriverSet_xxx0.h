/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-29 16:02:26
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#ifndef _DeviceDriverSet_xxx0_H_
#define _DeviceDriverSet_xxx0_H_

#define _Test_DeviceDriverSet 0

/*RBG LED*/
#include "FastLED.h"
class DeviceDriverSet_RBGLED
{
public:
  void DeviceDriverSet_RBGLED_Init(uint8_t set_Brightness);
  void DeviceDriverSet_RBGLED_xxx(uint16_t Duration, uint8_t Traversal_Number, CRGB colour);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_RBGLED_Test(void);
#endif
  void DeviceDriverSet_RBGLED_Color(uint8_t LED_s, uint8_t r, uint8_t g, uint8_t b);

public:
private:
#define PIN_RBGLED 4
#define NUM_LEDS 1
public:
  CRGB leds[NUM_LEDS];
};

/*Key Detection*/
class DeviceDriverSet_Key
{
public:
  void DeviceDriverSet_Key_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Key_Test(void);
#endif
  void DeviceDriverSet_key_Get(uint8_t *get_keyValue);

public:
#define PIN_Key 2
#define keyValue_Max 4
public:
  static uint8_t keyValue;
};

/*ITR20001 Detection*/
class DeviceDriverSet_ITR20001
{
public:
  bool DeviceDriverSet_ITR20001_Init(void);
  int DeviceDriverSet_ITR20001_getAnaloguexxx_L(void);
  int DeviceDriverSet_ITR20001_getAnaloguexxx_M(void);
  int DeviceDriverSet_ITR20001_getAnaloguexxx_R(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_ITR20001_Test(void);
#endif

private:
//03
// #define PIN_ITR20001xxxL A0
// #define PIN_ITR20001xxxM A1
// #define PIN_ITR20001xxxR A2
//04
#define PIN_ITR20001xxxL A2
#define PIN_ITR20001xxxM A1
#define PIN_ITR20001xxxR A0
};

/*Voltage Detection*/
class DeviceDriverSet_Voltage
{
public:
  void DeviceDriverSet_Voltage_Init(void);
  float DeviceDriverSet_Voltage_getAnalogue(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Voltage_Test(void);
#endif
private:
#define PIN_Voltage A3
};

/*Motor*/
class DeviceDriverSet_Motor
{
public:
  void DeviceDriverSet_Motor_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Motor_Test(void);
#endif
  void DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //Group A motor parameters
                                     boolean direction_B, uint8_t speed_B, //Group B motor parameters
                                     boolean controlED                     //AB enable setting (true)
  );                                                                       //motor control
private:
  // #define PIN_Motor_PWMA 5
  // #define PIN_Motor_PWMB 6
  // #define PIN_Motor_STBY 8
  // #define PIN_Motor_BIN_1 7
  // #define PIN_Motor_AIN_1 9
//TB6612
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3
public:
#define speed_Max 255
#define direction_just true
#define direction_back false
#define direction_void 3

#define Duration_enable true
#define Duration_disable false
#define control_enable true
#define control_disable false
};
/*ULTRASONIC*/

//#include <NewPing.h>
class DeviceDriverSet_ULTRASONIC
{
public:
  void DeviceDriverSet_ULTRASONIC_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_ULTRASONIC_Test(void);
#endif
  void DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get /*out*/);

private:
#define TRIG_PIN 13      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 12      // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
};
/*Servo*/
#include <Servo.h>
class DeviceDriverSet_Servo
{
public:
  void DeviceDriverSet_Servo_Init(unsigned int Position_angle);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Servo_Test(void);
#endif
  void DeviceDriverSet_Servo_control(unsigned int Position_angle);
  void DeviceDriverSet_Servo_controls(uint8_t Servo, unsigned int Position_angle);

private:
#define PIN_Servo_z 10
#define PIN_Servo_y 11
};
/*IRrecv*/
#include "IRremote.h"
class DeviceDriverSet_IRrecv
{
public:
  void DeviceDriverSet_IRrecv_Init(void);
  bool DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/);
  void DeviceDriverSet_IRrecv_Test(void);

public:
  unsigned long IR_PreMillis;

private:
#define RECV_PIN 9

/*A:4294967295*/
#define aRECV_upper 16736925
#define aRECV_lower 16754775
#define aRECV_Left 16720605
#define aRECV_right 16761405
#define aRECV_ok 16712445
#define aRECV_1 16738455
#define aRECV_2 16750695
#define aRECV_3 16756815
#define aRECV_4 16724175
#define aRECV_5 16718055
#define aRECV_6 16743045
#define aRECV_7 16716015
#define aRECV_8 16726215
#define aRECV_9 16734885
// #define aRECV_ *16728765
// #define aRECV_0 16730805
// #define aRECV_ # 16732845
/*B:*/
#define bRECV_upper 5316027
#define bRECV_lower 2747854299
#define bRECV_Left 1386468383
#define bRECV_right 553536955
#define bRECV_ok 3622325019
#define bRECV_1 3238126971
#define bRECV_2 2538093563
#define bRECV_3 4039382595
#define bRECV_4 2534850111
#define bRECV_5 1033561079
#define bRECV_6 1635910171
#define bRECV_7 2351064443
#define bRECV_8 1217346747
#define bRECV_9 71952287
  // #define bRECV_ *851901943
  // #define bRECV_0 465573243
  // #define bRECV_ # 1053031451
};

#endif
