#include "SwerveDrivetrain.h"
#include <cmath>

SwerveDrivetrain::SwerveDrivetrain(SwerveModule *m1, SwerveModule *m2, SwerveModule *m3, SwerveModule *m4) {
    module[0] = m1;
    module[1] = m2;
    module[2] = m3;
    module[3] = m4;
}

double SwerveDrivetrain::maxMagnitude(double arr[4]) {
    double maxVal = 0;
    for (int i = 0; i < 4; i++) {
        if (arr[i] > maxVal) maxVal = arr[i];
    }
    return maxVal;
}
/*
    @param axis0 - the left/right axis
    @param axis1 - the forward/backward axis
    @param turnMag - the rotation axis
    @param theta - the current robot rotation
*/
void SwerveDrivetrain::holonomicDrive(double axis0, double axis1, double turnMag, double theta) {
    double driveAngle = atan2(axis1, axis0);
    double driveMag = sqrt(pow(axis1, 2) + pow(axis0, 2));
    double magnitudes[4];

    // Assign vectors and add turn component
    for (int i = 0; i < 4; i++) {
        module[i]->assignDrivetrainVector(driveAngle, driveMag, turnMag, theta);
        module[i]->addTurnVector(turnMag);
        module[i]->calculatePosition();
        magnitudes[i] = module[i]->magnitude;
    }

    // Scale if any magnitude exceeds 1
    double maxMag = maxMagnitude(magnitudes);
    if (maxMag > 1) {
        for (int i = 0; i < 4; i++) {
            module[i]->magnitude /= maxMag;
            module[i]->xCord /= maxMag;
            module[i]->yCord /= maxMag;
        }
    }

    // Write to hardware
    for (int i = 0; i < 4; i++) {
        module[i]->write();
    }
}