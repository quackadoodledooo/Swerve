#ifndef SWERVE_MODULE
#define SWERVE_MODULE

#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>

//single module for swerve, controls motor and servo
class SwerveModule {
public:
    SwerveModule(NoU_Motor *motor, NoU_Servo *servo);
    void assignDrivetrainVector(double driveAngle, double driveMag, double turnMag, double theta);
    void addTurnVector(double turnMag);
    void calculatePosition();
    void write();
    // Encapsulated vector variables
    double xCord;
    double yCord;
    double magnitude;
    double direction; // in degrees
private:
    int id;
    NoU_Motor *Drive;
    NoU_Servo *Turn;
};

#endif