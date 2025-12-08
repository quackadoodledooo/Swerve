#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <cmath>
#include <tgmath.h>

//Drivetrain Motor/Servo Declarations
NoU_Motor Drive1(1);
NoU_Servo Turn1(1, 500, 2500);
NoU_Motor Drive2(2);
NoU_Servo Turn2(2, 500, 2500);
NoU_Motor Drive3(3);
NoU_Servo Turn3(3, 500, 2500);
NoU_Motor Drive4(4);
NoU_Servo Turn4(4, 500, 2500);

//turn magnitude and the vectors for each individual module in format {{mag1, dir1}, {mag2, dir2}, {mag3, dir3}, {mag4, dir4}}
double turnMag = 0.0;
double driveAngle = 0.0;
double driveMag = 0.0;
double drivetrainVectors[4][2] = 
{ { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 } };

double theta;
double headingOffset = 0.0;
int mod1Offset = 0;
int mod2Offset = 0;
int mod3Offset = 0;
int mod4Offset = 0;
int lastOffsetTime = millis();

const bool AM_DEBUGGING = false;

void setup() {
  //Basic Board Setup
  NoU3.begin();
  NoU3.setServiceLight(LIGHT_DISABLED);
  xTaskCreatePinnedToCore(taskUpdateSwerve, "taskUpdateSwerve", 4096, NULL, 2, NULL, 1);
  NoU3.calibrateIMUs();

  //Motor Configs
  Drive1.setMotorCurve(0.6, 1, 0, 2);
  Drive2.setMotorCurve(0.6, 1, 0, 2);
  Drive3.setMotorCurve(0.6, 1, 0, 2);
  Drive4.setMotorCurve(0.6, 1, 0, 2);

  PestoLink.begin("Zorp");
  Serial.begin(115200);
}

void loop() {
  //debug
  if (AM_DEBUGGING & (millis() - lastOffsetTime) > 50) {
    Serial.println("Robot Angle");
    Serial.println(theta);
    if (PestoLink.buttonHeld(0))
      mod1Offset++;
    Serial.println("Mod 1 Offset:");
    Serial.println(mod1Offset);
    if (PestoLink.buttonHeld(1))
      mod2Offset++;
    Serial.println("Mod 2 Offset:");
    Serial.println(mod2Offset);
    if (PestoLink.buttonHeld(2))
      mod3Offset++;
    Serial.println("Mod 3 Offset:");
    Serial.println(mod3Offset);
    if (PestoLink.buttonHeld(3))
      mod4Offset++;
    Serial.println("Mod 4 Offset:");
    Serial.println(mod4Offset);

    lastOffsetTime = millis();
  }
}

void taskUpdateSwerve(void* pvParameters) {
  while (true) {

    // Set up Gyro and its variables
    theta = NoU3.yaw - headingOffset;

    // get magnitude and direction and assign to drivetrainVectors array, add offsets
    // set turn vector magnitude
    // set RSL based on whether a gamepad is connected
    // ANGLES IN RADIANS
    if (PestoLink.isConnected()) {

      driveAngle = atan2(PestoLink.getAxis(1), PestoLink.getAxis(0));
      driveMag = sqrt(pow(PestoLink.getAxis(1), 2) + pow(PestoLink.getAxis(0), 2));
      turnMag = PestoLink.getAxis(2);

      drivetrainVectors[0][0] = driveMag;
      drivetrainVectors[0][1] = driveAngle + ((mod1Offset + theta) * (PI / 180));

      drivetrainVectors[1][0] = driveMag;
      drivetrainVectors[1][1] = driveAngle + ((mod2Offset + theta) * (PI / 180));

      drivetrainVectors[2][0] = driveMag;
      drivetrainVectors[2][1] = driveAngle + ((mod3Offset + theta) * (PI / 180));

      drivetrainVectors[3][0] = driveMag;
      drivetrainVectors[3][1] = driveAngle + ((mod4Offset + theta) * (PI / 180));

      NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
      NoU3.setServiceLight(LIGHT_DISABLED);
    }

    //Heading Offset Control

    if (PestoLink.isConnected() && PestoLink.buttonHeld(10) && PestoLink.buttonHeld(11)) {
      headingOffset = NoU3.yaw;
    }



    //Vector Addition
    //Finds the component form of the current drive vector on the unit circle for each individual module
    //Uses the fact that turn vector is always 0 degrees, adds it to x coordinate.
    //reconverts back into magnitude and directon form
    //ANGLES IN DEGREES

    double xCord1 = drivetrainVectors[0][0] * cos(drivetrainVectors[0][1]) + turnMag;
    double yCord1 = drivetrainVectors[0][0] * sin(drivetrainVectors[0][1]);

    double xCord2 = drivetrainVectors[1][0] * cos(drivetrainVectors[1][1]) + turnMag;
    double yCord2 = drivetrainVectors[1][0] * sin(drivetrainVectors[1][1]);

    double xCord3 = drivetrainVectors[2][0] * cos(drivetrainVectors[2][1]) + turnMag;
    double yCord3 = drivetrainVectors[2][0] * sin(drivetrainVectors[2][1]);

    double xCord4 = drivetrainVectors[3][0] * cos(drivetrainVectors[3][1]) + turnMag;
    double yCord4 = drivetrainVectors[3][0] * sin(drivetrainVectors[3][1]);

    double cordArray[4] = { abs(xCord1), abs(xCord2), abs(xCord3), abs(xCord4) };

    //Find max x coordinate (since only adding to x)
    double massiveCord = 0.0;
    for (int i = 0; i < 3; i++) {
      if (cordArray[i] > massiveCord) {
        massiveCord = cordArray[i];
      }
    }
    //scales all components to the one with the largest magnitude
    if (massiveCord > 1 && massiveCord != 0) {
      xCord1 /= massiveCord;
      yCord1 /= massiveCord;
      xCord2 /= massiveCord;
      yCord2 /= massiveCord;
      xCord3 /= massiveCord;
      yCord3 /= massiveCord;
      xCord4 /= massiveCord;
      yCord4 /= massiveCord;
    }

    //component form --> magnitude and direction form as semifinal values
    drivetrainVectors[0][1] = atan2(yCord1, xCord1) * (180 / PI);
    drivetrainVectors[0][0] = sqrt(pow(xCord1, 2) + pow(yCord1, 2));
    drivetrainVectors[1][1] = atan2(yCord2, xCord2) * (180 / PI);
    drivetrainVectors[1][0] = sqrt(pow(xCord2, 2) + pow(yCord2, 2));
    drivetrainVectors[2][1] = atan2(yCord3, xCord3) * (180 / PI);
    drivetrainVectors[2][0] = sqrt(pow(xCord3, 2) + pow(yCord3, 2));
    drivetrainVectors[3][1] = atan2(yCord4, xCord4) * (180 / PI);
    drivetrainVectors[3][0] = sqrt(pow(xCord4, 2) + pow(yCord4, 2));

    //Wrapping functions for 360 degree motion with 180 degree servos
    if (drivetrainVectors[0][1] < -1) {
      drivetrainVectors[0][1] += 181;
      drivetrainVectors[0][0] *= -1;
    }
    if (drivetrainVectors[0][1] > 181) {
      drivetrainVectors[0][1] -= 181;
      drivetrainVectors[0][0] *= -1;
    }

    if (drivetrainVectors[1][1] < -1) {
      drivetrainVectors[1][1] += 181;
      drivetrainVectors[1][0] *= -1;
    }
    if (drivetrainVectors[1][1] > 181) {
      drivetrainVectors[1][1] -= 181;
      drivetrainVectors[1][0] *= -1;
    }

    if (drivetrainVectors[2][1] < -1) {
      drivetrainVectors[2][1] += 181;
      drivetrainVectors[2][0] *= -1;
    }
    if (drivetrainVectors[2][1] > 181) {
      drivetrainVectors[2][1] -= 181;
      drivetrainVectors[2][0] *= -1;
    }

    if (drivetrainVectors[3][1] < -1) {
      drivetrainVectors[3][1] += 181;
      drivetrainVectors[3][0] *= -1;
    }
    if (drivetrainVectors[3][1] > 181) {
      drivetrainVectors[3][1] -= 181;
      drivetrainVectors[3][0] *= -1;
    }


    //write to drivetrain + deadzone
    if (PestoLink.isConnected() && (abs(PestoLink.getAxis(0)) + abs(PestoLink.getAxis(1)) + abs(PestoLink.getAxis(2))) > 0.02) {
      Turn1.write(int(drivetrainVectors[0][1]));
      Drive1.set(drivetrainVectors[0][0]);
      Turn2.write(int(drivetrainVectors[1][1]));
      Drive2.set(drivetrainVectors[1][0]);
      Turn3.write(int(drivetrainVectors[2][1]));
      Drive3.set(drivetrainVectors[2][0]);
      Turn4.write(int(drivetrainVectors[3][1]));
      Drive4.set(drivetrainVectors[3][0]);
    } else {
      Turn1.write(0);
      Drive1.setBrakeMode(true);
      Turn2.write(0);
      Drive2.setBrakeMode(true);
      Turn3.write(0);
      Drive3.setBrakeMode(true);
      Turn4.write(0);
      Drive4.setBrakeMode(true);
    }

    vTaskDelay(pdMS_TO_TICKS(10));  //this line is like arduino delay() but for rtos tasks
  }
}
