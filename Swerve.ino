#include <PestoLink-Receive.h>
#include <Keys.h>
#include <Arduino.h>
#include <cmath>
#include <tgmath.h>
#include "SwerveModule.h"
#include "SwerveDrivetrain.h"
#include <FastLED.h>

#define PI 3.14159265359

//Swerve vars
double turnMag = 0.0;
double driveAngle = 0.0;
double driveMag = 0.0;
double drivetrainVectors[4][2] = { { 0, 0 },
                                  { 0, 0 },
                                  { 0, 0 },
                                  { 0, 0 } };
double theta;
double headingOffset = 0.0;
int mod1Offset = 405;
int mod2Offset = 135;
int mod3Offset = 315;
int mod4Offset = 225;
int lastOffsetTime = millis();
const bool AM_DEBUGGING = false;

//swerve motors and servos declaration
NoU_Motor Drive1(4);
NoU_Servo Turn1(2, 500, 2500);
NoU_Motor Drive2(5);
NoU_Servo Turn2(1, 500, 2500);
NoU_Motor Drive3(8);
NoU_Servo Turn3(3, 500, 2500);
NoU_Motor Drive4(1);
NoU_Servo Turn4(4, 500, 2500);

//swerve module declaration
SwerveModule module1(&Drive1, &Turn1);
SwerveModule module2(&Drive2, &Turn2);
SwerveModule module3(&Drive3, &Turn3);
SwerveModule module4(&Drive4, &Turn4);

//swerve drivetrain declaration
SwerveDrivetrain drivetrain(&module1, &module2, &module3, &module4);

//The gyroscope sensor is by default precise, but not accurate. This is fixable by adjusting the angular scale factor.
//Tuning procedure:
//Rotate the robot in place 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process.
int measured_angle = 27.562;
int angular_scale = (5.0 * 2.0 * PI) / measured_angle;

void setup() {
  NoU3.begin();
  PestoLink.begin("ROBOT NAME HERE");
  Serial.begin(115200);
  NoU3.calibrateIMUs();
  xTaskCreatePinnedToCore(taskUpdateSwerve, "taskUpdateSwerve", 4096, NULL, 2, NULL, 1);
  NoU3.setServiceLight(LIGHT_DISABLED);
}
void loop() {}

void taskUpdateSwerve(void* pvParameters) {
  while (true) {

    // Set up Gyro variables and take inputs
    theta = NoU3.yaw - headingOffset;
    driveAngle = atan2(PestoLink.getAxis(1), PestoLink.getAxis(0));
    driveMag = sqrt(pow(PestoLink.getAxis(1), 2) + pow(PestoLink.getAxis(0), 2));
    turnMag = PestoLink.getAxis(2);

    // set RSL based on whether a gamepad is connected
    if (PestoLink.isConnected()) {

      NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
      NoU3.setServiceLight(LIGHT_DISABLED);
    }
    
    drivetrain.holonomicDrive(driveAngle, driveMag, turnMag, theta);

    //Heading Offset Control
    if (PestoLink.isConnected() && PestoLink.buttonHeld(10) && PestoLink.buttonHeld(11)) headingOffset = NoU3.yaw;

    vTaskDelay(pdMS_TO_TICKS(10));  //this line is like arduino delay() but for rtos tasks
  }
}