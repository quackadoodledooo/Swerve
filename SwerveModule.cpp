#include "SwerveModule.h"
#include <cmath>

#define PI 3.14159265359

SwerveModule::SwerveModule(NoU_Motor *motor, NoU_Servo *servo) {
    this->Drive = motor;
    this->Turn = servo;
    Drive->setBrakeMode(true);
    Drive->setMotorCurve(0.6, 1, 0, 2);
    this->id = id;
    xCord = yCord = magnitude = direction = 0;
}

// Assign vector for the module based on joystick inputs and gyro
void SwerveModule::assignDrivetrainVector(double driveAngle, double driveMag, double turnMag, double theta) {
    direction = driveAngle + ((theta) * (PI / 180));
    magnitude = driveMag;
    xCord = magnitude * cos(direction);
    yCord = magnitude * sin(direction);
}

// Add turning component
void SwerveModule::addTurnVector(double turnMag) {
    xCord += turnMag;
}

// Convert component form back to magnitude/direction and wrap angles
void SwerveModule::calculatePosition() {
    direction = atan2(yCord, xCord) * (180 / PI);
    magnitude = sqrt(xCord * xCord + yCord * yCord);

    if (direction < -1) {
        direction += 181;
        magnitude *= -1;
    }
    if (direction > 181) {
        direction -= 181;
        magnitude *= -1;
    }
}

// Write to motors
void SwerveModule::write() {
    if (PestoLink.isConnected() && (abs(PestoLink.getAxis(0)) + abs(PestoLink.getAxis(1)) + abs(PestoLink.getAxis(2))) > 0.02) {
        Turn->write(int(direction));
        Drive->set(magnitude);
    } else {
        Turn->write(0);
        Drive->setBrakeMode(true);
        Drive->set(0);
    }
}