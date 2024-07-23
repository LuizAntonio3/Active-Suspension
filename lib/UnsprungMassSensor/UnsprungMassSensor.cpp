#include "UnsprungMassSensor.h"
// #include <pointTransformations.h>

const Matrix<2, 1> unsprungMassOrigin = {250.863 * 1e-3, 0 * 1e-3}; // position in meters

/*
    Parameter: Lower Arm angle (Theta) - in rad
    returns location in meters
*/
Matrix<2, 1> getLocationUnsprungMass(float theta) {
    Matrix<2,2> RotationEncoderArm;

    RotationEncoderArm = {cos(theta), - sin(theta), sin(theta), cos(theta)};

    return RotationEncoderArm * unsprungMassOrigin;
}

/*
    Parameter: PointA, PointB
    return distance btw Point A & Point B
*/
double calculateDistance(Matrix<2, 1> pointA, Matrix<2, 1> pointB) {
    return sqrt(pow(pointA(0, 0) - pointB(0, 0), 2) + 
                pow(pointA(1, 0) - pointB(1, 0), 2));
}

void UnsprungMassSensor::update() {
    unsigned long interval = millis() - this->previousTime;

    this->unsprungMassPreviousLocation = this->unsprungMassLocation;
    this->unsprungMassLocation = getLocationUnsprungMass(this->encoderArm->getPulsesRad());
    
    // velocity in m/s
    this->unsprungMassPreviousVelocity = this->unsprungMassVelocity;
    this->unsprungMassVelocity = calculateDistance(this->unsprungMassLocation, this->unsprungMassPreviousLocation) / interval;

    // aceleration in m/s^2
    this->unsprungMassAceleration = (this->unsprungMassVelocity - this->unsprungMassPreviousVelocity) / (interval*1e-3);

    this->previousTime = millis();
}