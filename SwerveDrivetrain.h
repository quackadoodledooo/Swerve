#ifndef SWERVE_DRIVETRAIN
#define SWERVE_DRIVETRAIN

#include "SwerveModule.h"

class SwerveDrivetrain {
public:
    SwerveDrivetrain(SwerveModule *m1, SwerveModule *m2, SwerveModule *m3, SwerveModule *m4);
    void holonomicDrive(double driveAngle, double driveMag, double turnMag, double theta);

private:
    double maxMagnitude(double arr[4]);
    SwerveModule *module[4];
};

#endif