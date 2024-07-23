#include "DamperSensor.h"
// #include <pointTransformations.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

// Points location in meters
const Matrix<2, 1> lowerFixationOrigin = {145.532 * 1e-3, 31.96 * 1e-3};
const Matrix<2, 1> upperFixationOrigin = {-88.022 * 1e-3, 139.206 * 1e-3};
const Matrix<2, 1> upperFixationOriginNegative = {88.022 * 1e-3, -139.206 * 1e-3};
const Matrix<2, 1> upperAxisOrigin = {-97.109  * 1e-3, 143.379 * 1e-3}; // must be fixed according to the original position of the system

/*
    Parameter: PointA, PointB
    return distance btw Point A & Point B
*/
double dampCalculateDistance(Matrix<2, 1> pointA, Matrix<2, 1> pointB) {
    return sqrt(pow(pointA(0, 0) - pointB(0, 0), 2) + 
                pow(pointA(1, 0) - pointB(1, 0), 2));
}

/*
    Parameter: Lower arm angle (Theta) - in rad
    returns location in meters
*/
Matrix<2, 1> dampGetLocationLowerFixation(float theta) {
    Matrix<2,2> RotationEncoderArm;

    RotationEncoderArm = {cos(theta), - sin(theta), sin(theta), cos(theta)};

    return RotationEncoderArm * lowerFixationOrigin; 
}

/*
    Parameter: Single link angle (Theta) - in rad
    returns location in meters
*/
Matrix<2, 1> dampGetLocationUpperFixation(float theta) {
    Matrix<2,2> RotationSingleLink;

    RotationSingleLink = {cos(theta), - sin(theta), sin(theta), cos(theta)};
    
    return upperFixationOrigin+(RotationSingleLink*(upperFixationOriginNegative+upperAxisOrigin));
}

void DamperSensor::updateZero() {
    Matrix<2,1> lowerFixation, upperFixation;

    lowerFixation = dampGetLocationLowerFixation(this->encoderArm->getPulsesRad());
    upperFixation = dampGetLocationUpperFixation(this->encoderSingleLink->getPulsesRadWithCorrection());

    this->zero = dampCalculateDistance(upperFixation, lowerFixation);
}

void DamperSensor::update() {
    Matrix<2,1> lowerFixation, upperFixation;
    unsigned long interval = millis() - this->previousTime;
    double slAngle = this->encoderSingleLink->getPulsesRadWithCorrection();

    lowerFixation = dampGetLocationLowerFixation(this->encoderArm->getPulsesRad());
    upperFixation = dampGetLocationUpperFixation(slAngle);

    this->previousDeflection = this->deflection;
    this->deflection = dampCalculateDistance(upperFixation, lowerFixation) - this->zero;
    this->deflectionSpeed = (this->deflection - this->previousDeflection) / (interval*1e-3);

    this->previousLinearDeflection = this->linearDeflection;
    if(slAngle <= PI/2 or slAngle >= 3/2 * PI) {
        // negative deflection - extension of the coil
        this->linearDeflection = SINGLE_LINK_LENGTH * cos(slAngle);
    }
    else {
        // positive deflection - compression of the coil
        this->linearDeflection = -1 * SINGLE_LINK_LENGTH * cos(PI - slAngle);
    }
    
    this->linearDeflectionSpeed = (this->linearDeflection - this->previousLinearDeflection) / (interval*1e-3);
    this->previousTime = millis();
}